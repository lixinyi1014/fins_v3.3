//
// 事件日志 + 遥测输出，说明见 EventLog.h
//
#include "EventLog.h"
#include "main.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

namespace
{
constexpr uint32_t kLogSize = 128;   // 保留最近 128 条，上位机缺号时可补发
constexpr uint32_t kTxSize = 2048;   // 串口发送环形缓冲
constexpr uint32_t kStatusEveryCycles = 15; // 150 Hz / 15 = 10 Hz 状态行
constexpr uint32_t kPlotEveryCycles = 5;    // 150 Hz / 5 = 30 Hz 曲线行（约 3 kB/s，串口 115200 8E1 约 10 kB/s）
constexpr uint32_t kEventsPerCycle = 6;

struct Record
{
    uint32_t seq;
    uint32_t t_ms;
    uint16_t code;
    int32_t arg;
    const char *text;
};

Record g_log[kLogSize];
volatile uint32_t g_last_seq = 0;

struct LimitSlot
{
    uint16_t code;
    uint32_t last_ms;
};
LimitSlot g_limits[32];

uint8_t g_tx[kTxSize];
volatile uint32_t g_tx_head = 0;     // 写入计数（单调递增）
volatile uint32_t g_tx_tail = 0;     // 已发送计数（单调递增）
volatile uint32_t g_tx_inflight = 0; // 当前 HAL 中断发送中的字节数，0 表示空闲
volatile uint32_t g_tx_start_ms = 0;  // 这一段开始发送的时间，用于发现发送卡住
volatile uint32_t g_tx_dropped = 0;

uint32_t g_next_send = 1;            // 下一条要发的事件序号（只在控制任务里用）
volatile uint32_t g_resend_from = 0; // 上位机补发请求，0 表示没有
uint32_t g_cycle = 0;
TelemetryStatusFn g_status_fn = nullptr;
TelemetryStatusFn g_plot_fn = nullptr;
TelemetryStatusFn g_pid_fn = nullptr;
FatalNeutralFn g_fatal_neutral = nullptr;

// PRIMASK 临界区：任务、任意优先级中断、调度器启动前都可用，持续时间只有几十个指令。
inline uint32_t Lock()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
inline void Unlock(uint32_t primask) { __set_PRIMASK(primask); }

void StoreLocked(uint16_t code, int32_t arg, const char *text)
{
    const uint32_t seq = g_last_seq + 1;
    Record &r = g_log[seq % kLogSize];
    r.seq = seq;
    r.t_ms = HAL_GetTick();
    r.code = code;
    r.arg = arg;
    r.text = text;
    g_last_seq = seq;
}

uint32_t OldestSeq(uint32_t last)
{
    return last >= kLogSize ? last - kLogSize + 1 : 1;
}

/*
 * 上报"事件已被覆盖"(211) 并返回此刻仍保留的最早序号。
 * 注意顺序：211 本身也要占一格，会把环形缓冲里最旧的那条挤掉，所以必须先记 211、再算最早序号。
 * 原来是先算再记，算出来的那一条马上被 211 覆盖，发送循环发现对不上又记一条 211……
 * 一直空转、一条事件也发不出去（上位机在下位机运行很久后才连上时必然出现）。
 */
uint32_t PostLostAndGetOldest()
{
    Event_Post(EV_WARN_EVENT_LOST, static_cast<int32_t>(OldestSeq(g_last_seq + 1)));
    return OldestSeq(g_last_seq);
}

// ---------- 发送 ----------
void StartTxLocked()
{
    if (g_tx_inflight != 0 || g_tx_head == g_tx_tail)
        return;
    const uint32_t start = g_tx_tail % kTxSize;
    uint32_t len = g_tx_head - g_tx_tail;
    if (len > kTxSize - start)
        len = kTxSize - start; // 到缓冲尾部为止，剩余部分下一次发
    if (HAL_UART_Transmit_IT(&huart6, &g_tx[start], static_cast<uint16_t>(len)) == HAL_OK)
    {
        g_tx_inflight = len;
        g_tx_start_ms = HAL_GetTick();
    }
}

uint32_t TxFree()
{
    const uint32_t primask = Lock();
    const uint32_t used = g_tx_head - g_tx_tail;
    Unlock(primask);
    return kTxSize - used;
}

bool WriteBytes(const char *data, uint32_t len)
{
    const uint32_t primask = Lock();
    const uint32_t free_bytes = kTxSize - (g_tx_head - g_tx_tail);
    if (len > free_bytes)
    {
        g_tx_dropped += len;
        Unlock(primask);
        return false;
    }
    for (uint32_t i = 0; i < len; ++i)
        g_tx[(g_tx_head + i) % kTxSize] = static_cast<uint8_t>(data[i]);
    g_tx_head += len;
    StartTxLocked();
    Unlock(primask);
    return true;
}

// 末尾补 "*校验\r\n"。buf 以 '$' 开头，len 为当前长度，返回总长度。
int FinishLine(char *buf, int len, int size)
{
    uint8_t checksum = 0;
    for (int i = 1; i < len; ++i)
        checksum ^= static_cast<uint8_t>(buf[i]);
    static const char hex[] = "0123456789ABCDEF";
    if (len + 5 > size)
        return 0;
    buf[len++] = '*';
    buf[len++] = hex[checksum >> 4];
    buf[len++] = hex[checksum & 0x0F];
    buf[len++] = '\r';
    buf[len++] = '\n';
    return len;
}

// 文本字段去掉分隔符，最多 40 字符
int AppendText(char *buf, int len, int size, const char *text)
{
    if (!text)
        return len;
    for (int i = 0; text[i] && i < 40 && len < size - 8; ++i)
    {
        char c = text[i];
        if (c == ',' || c == '*' || c == '$' || c == '\r' || c == '\n')
            c = '_';
        buf[len++] = c;
    }
    return len;
}

// 不用 snprintf 的十进制格式化（致命处理里也要用）
int AppendUint(char *buf, int len, uint32_t value)
{
    char tmp[11];
    int n = 0;
    do
    {
        tmp[n++] = static_cast<char>('0' + value % 10U);
        value /= 10U;
    } while (value);
    while (n)
        buf[len++] = tmp[--n];
    return len;
}
int AppendInt(char *buf, int len, int32_t value)
{
    if (value < 0)
    {
        buf[len++] = '-';
        return AppendUint(buf, len, static_cast<uint32_t>(-(value + 1)) + 1U);
    }
    return AppendUint(buf, len, static_cast<uint32_t>(value));
}

int FormatEvent(char *buf, int size, const Record &r)
{
    int len = 0;
    memcpy(buf, "$EV,", 4);
    len = 4;
    len = AppendUint(buf, len, r.seq);
    buf[len++] = ',';
    len = AppendUint(buf, len, r.t_ms);
    buf[len++] = ',';
    len = AppendUint(buf, len, r.code);
    buf[len++] = ',';
    len = AppendInt(buf, len, r.arg);
    buf[len++] = ',';
    len = AppendText(buf, len, size, r.text);
    return FinishLine(buf, len, size);
}

// ---------- 致命处理用的轮询发送（关中断后仍可用） ----------
void PollPut(const char *s, int n)
{
    for (int i = 0; i < n; ++i)
    {
        uint32_t guard = 0;
        while (!(USART6->SR & USART_SR_TXE) && ++guard < 200000U)
        {
        }
        USART6->DR = static_cast<uint8_t>(s[i]);
    }
}

void BusyDelay(uint32_t loops)
{
    for (volatile uint32_t i = 0; i < loops; ++i)
    {
    }
}

int AppendHex(char *buf, int len, uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    for (int shift = 28; shift >= 0; shift -= 4)
        buf[len++] = hex[(value >> shift) & 0xFU];
    return len;
}
} // namespace

extern "C" {

void Event_PostText(uint16_t code, int32_t arg, const char *static_text)
{
    const uint32_t primask = Lock();
    StoreLocked(code, arg, static_text);
    Unlock(primask);
}

void Event_Post(uint16_t code, int32_t arg)
{
    Event_PostText(code, arg, nullptr);
}

void Event_PostLimited(uint16_t code, int32_t arg, uint32_t min_interval_ms)
{
    const uint32_t now = HAL_GetTick();
    const uint32_t primask = Lock();
    LimitSlot *slot = nullptr;
    for (auto &s : g_limits)
    {
        if (s.code == code)
        {
            slot = &s;
            break;
        }
        if (!slot && s.code == 0)
            slot = &s;
    }
    if (slot && slot->code == code && now - slot->last_ms < min_interval_ms)
    {
        Unlock(primask);
        return; // 间隔内已经记过
    }
    if (slot)
    {
        slot->code = code;
        slot->last_ms = now;
    }
    StoreLocked(code, arg, nullptr);
    Unlock(primask);
}

uint32_t Event_LastSeq(void)
{
    return g_last_seq;
}

void Telemetry_SetStatusSource(TelemetryStatusFn fn)
{
    g_status_fn = fn;
}

void Telemetry_SetPlotSource(TelemetryStatusFn fn)
{
    g_plot_fn = fn;
}

void Telemetry_SetPidSource(TelemetryStatusFn fn)
{
    g_pid_fn = fn;
}

void Fins_SetFatalNeutral(FatalNeutralFn fn)
{
    g_fatal_neutral = fn;
}

void Telemetry_RequestResend(uint32_t from_seq)
{
    g_resend_from = from_seq ? from_seq : 1U;
}

void Telemetry_OnTxDone(void)
{
    const uint32_t primask = Lock();
    g_tx_tail += g_tx_inflight;
    g_tx_inflight = 0;
    StartTxLocked();
    Unlock(primask);
}

void Telemetry_Write(const char *data, uint16_t len)
{
    WriteBytes(data, len);
}

void Telemetry_Step(void)
{
    ++g_cycle;
    char line[256];   // $PL 加了水压计、加计 7 个字段，160 在极端值时会截断

    // 1. 补发请求
    const uint32_t request = g_resend_from;
    if (request)
    {
        g_resend_from = 0;
        uint32_t from = request;
        if (from < OldestSeq(g_last_seq))
            from = PostLostAndGetOldest();
        if (from < g_next_send)
            g_next_send = from;
    }

    // 2. 发送新事件（每周期限量，避免一次占满缓冲）
    for (uint32_t n = 0; n < kEventsPerCycle && g_next_send <= g_last_seq; ++n)
    {
        if (TxFree() < sizeof(line))
            break;
        Record r;
        const uint32_t primask = Lock();
        r = g_log[g_next_send % kLogSize];
        Unlock(primask);
        if (r.seq != g_next_send)
        {
            // 已被新事件覆盖（落后超过 128 条）
            g_next_send = PostLostAndGetOldest();
            continue;
        }
        const int len = FormatEvent(line, sizeof(line), r);
        if (len > 0 && WriteBytes(line, static_cast<uint32_t>(len)))
            ++g_next_send;
        else
            break;
    }

    // 3. 状态行
    if (g_cycle % kStatusEveryCycles == 0 && TxFree() > 2 * sizeof(line))
    {
        int len = 0;
        memcpy(line, "$ST,", 4);
        len = 4;
        len = AppendUint(line, len, HAL_GetTick());
        line[len++] = ',';
        len = AppendUint(line, len, g_last_seq);
        if (g_status_fn)
        {
            line[len++] = ',';
            const int avail = static_cast<int>(sizeof(line)) - len - 8;
            const int n = g_status_fn(line + len, avail);
            if (n > 0 && n < avail) // snprintf 截断时返回值会大于缓冲，不能直接加
                len += n;
            else
                --len;
        }
        len = FinishLine(line, len, sizeof(line));
        if (len > 0)
            WriteBytes(line, static_cast<uint32_t>(len));
    }

    // 4. 曲线行 $PL 和 PID 分解行 $PD（优先级最低：缓冲紧张时先不发）。两种行错开 2 个周期发，串口负载更均匀
    const bool plot_turn = g_cycle % kPlotEveryCycles == 0;
    const bool pid_turn = g_cycle % kPlotEveryCycles == 2;
    TelemetryStatusFn fn = plot_turn ? g_plot_fn : (pid_turn ? g_pid_fn : nullptr);
    if (fn && TxFree() > 3 * sizeof(line))
    {
        int len = 0;
        memcpy(line, plot_turn ? "$PL," : "$PD,", 4);
        len = 4;
        len = AppendUint(line, len, HAL_GetTick());
        line[len++] = ',';
        const int avail = static_cast<int>(sizeof(line)) - len - 8;
        const int n = fn(line + len, avail);
        if (n > 0 && n < avail)
        {
            len += n;
            len = FinishLine(line, len, sizeof(line));
            if (len > 0)
                WriteBytes(line, static_cast<uint32_t>(len));
        }
    }

    // 5. 发送卡住：一段最多 2048 字节，115200 下约 0.2 s 发完；超过 0.5 s 没有发送完成中断就强制重启
    {
        uint32_t stalled = 0;
        const uint32_t primask = Lock();
        if (g_tx_inflight != 0 && HAL_GetTick() - g_tx_start_ms > 500U)
        {
            HAL_UART_AbortTransmit(&huart6);
            stalled = g_tx_inflight;
            g_tx_tail += g_tx_inflight; // 这一段丢掉，事件可由上位机按序号补发
            g_tx_inflight = 0;
            StartTxLocked();
        }
        Unlock(primask);
        if (stalled)
            Event_PostLimited(EV_WARN_UART_TX_STALL, static_cast<int32_t>(stalled), 1000);
    }

    // 6. 发送缓冲溢出
    uint32_t dropped;
    {
        const uint32_t primask = Lock();
        dropped = g_tx_dropped;
        g_tx_dropped = 0;
        Unlock(primask);
    }
    if (dropped)
        Event_PostLimited(EV_WARN_TX_OVERFLOW, static_cast<int32_t>(dropped), 1000);
}

void Fins_ErrorHandlerHit(uint32_t caller)
{
    Event_Post(EV_WARN_ERROR_HANDLER, static_cast<int32_t>(caller));
}

void Fins_Fatal(uint16_t code, int32_t arg, const char *text)
{
    __disable_irq();
    static volatile uint32_t nesting = 0;
    ++nesting;
    StoreLocked(code, arg, text);
    const uint32_t fatal_seq = g_last_seq;

    // 停掉 HAL 的中断发送，改为寄存器轮询
    USART6->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);
    PollPut("\r\n", 2);

    static bool neutral_done = false;
    char line[160];
    for (;;)
    {
        // 补发最近 16 条（上位机按序号去重），保证刚连上的上位机也能看到停机前的经过
        const uint32_t oldest = OldestSeq(fatal_seq);
        const uint32_t from = fatal_seq >= 16U && fatal_seq - 15U > oldest ? fatal_seq - 15U : oldest;
        for (uint32_t seq = from; seq <= fatal_seq; ++seq)
        {
            const Record &r = g_log[seq % kLogSize];
            if (r.seq != seq)
                continue;
            const int len = FormatEvent(line, sizeof(line), r);
            if (len > 0)
                PollPut(line, len);
        }
        int len = 0;
        memcpy(line, "$FT,", 4);
        len = 4;
        len = AppendUint(line, len, HAL_GetTick());
        line[len++] = ',';
        len = AppendUint(line, len, fatal_seq);
        line[len++] = ',';
        len = AppendUint(line, len, code);
        len = FinishLine(line, len, sizeof(line));
        if (len > 0)
            PollPut(line, len);

        // 第一次报告之后再尝试推进器回中位（I2C 若卡死也至少已经报告过一次）
        if (!neutral_done && nesting == 1U && g_fatal_neutral)
        {
            neutral_done = true;
            g_fatal_neutral();
        }
        BusyDelay(12000000U); // 约 0.5 s
    }
}

void Fins_AssertFailed(const char *file, int line)
{
    const char *base = file;
    for (const char *p = file; p && *p; ++p)
        if (*p == '/' || *p == '\\')
            base = p + 1;
    Fins_Fatal(EV_FATAL_ASSERT, line, base);
}

void Fins_FaultReport(uint16_t code, uint32_t exc_return)
{
    __disable_irq();
    static char text[48];
    uint32_t pc = 0;
    if (exc_return & 0x4U) // 发生在任务里（PSP），栈帧第 6 个字是出错 PC
    {
        const uint32_t *frame = reinterpret_cast<const uint32_t *>(__get_PSP());
        pc = frame[6];
    }
    int len = 0;
    memcpy(text, "CFSR=", 5);
    len = 5;
    len = AppendHex(text, len, SCB->CFSR);
    memcpy(text + len, " HFSR=", 6);
    len += 6;
    len = AppendHex(text, len, SCB->HFSR);
    text[len] = '\0';
    Fins_Fatal(code, static_cast<int32_t>(pc), text);
}

void Fins_UnhandledIrq(void)
{
    const int32_t vector = static_cast<int32_t>(SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);
    Fins_Fatal(EV_FATAL_UNHANDLED_IRQ, vector - 16, nullptr);
}

} // extern "C"
