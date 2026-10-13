//
// 事件日志 + 遥测输出（UART6）
//
// 所有停机点、警告、关键状态变化都在这里有唯一编号。
// 每条事件带：序号 seq（上电从 1 开始递增）、上电毫秒时间 t_ms、编号 code、参数 arg、可选短文本。
// 上位机按 seq 排序和去重；发现缺号就发 "EVQ:<seq>" 让下位机补发。
//
// 输出格式（每行一条，NMEA 风格校验）：
//   $EV,<seq>,<t_ms>,<code>,<arg>,<text>*<XOR校验两位十六进制>\r\n
//   $ST,<t_ms>,<last_seq>,<状态字段...>*<校验>\r\n      （约 10 Hz）
//   $PL,<t_ms>,<曲线字段...>*<校验>\r\n                  （约 30 Hz，上位机画图用）
//
// 编号规则（与上位机 finsrov_console/codes.py 保持一致，修改时两边一起改）：
//   1xx 停机：推进器全部回中位并锁存，需重新发 ON / TES 才会再动
//   2xx 警告：不停机，只记录
//   3xx 信息：上电、解锁、模式切换等
//   9xx 致命：单片机停止运行（死循环），推进器已尽力写中位，必须复位/重新上电
//
#ifndef FINSROV_EVENT_LOG_H
#define FINSROV_EVENT_LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum FinsEventCode
{
    /* ---- 1xx 停机（锁存） ---- */
    EV_STOP_USER_OFF          = 101, // 收到 OFF 命令
    EV_STOP_HEARTBEAT         = 102, // 连续 100 个控制周期（约 0.67 s）没收到任何上位机数据；arg=静默周期数
    EV_STOP_NAN_VERTICAL      = 103, // 垂直推进器控制量连续 100 周期为 NaN/Inf
    EV_STOP_NAN_HORIZONTAL    = 104, // 水平推进器（偏航）控制量连续 100 周期为 NaN/Inf
    EV_STOP_PCA_WRITE         = 105, // 推进器 PCA9685 写入连续 100 周期失败；arg=最后失败通道
    EV_STOP_TCA_SELECT        = 106, // I2C 扩展器 TCA9548A 选通 4 号口连续 100 周期失败

    /* ---- 2xx 警告（不停机） ---- */
    EV_WARN_PRESSURE_READ     = 201, // 水压计读取失败，沿用上一次成功值；arg=通道号 0..3
    EV_WARN_MAG_READ          = 202, // 磁力计 IST8310 读取失败，沿用上一次成功值
    EV_WARN_PCA_WRITE         = 203, // 推进器 PCA9685 写入失败（连续失败的第一次）；arg=通道
    EV_WARN_TCA_SELECT        = 204, // TCA9548A 选通失败（连续失败的第一次）
    EV_WARN_NAN_VERTICAL      = 205, // 垂直控制量出现 NaN/Inf（第一次），本周期沿用上一次输出并复位相关 PID
    EV_WARN_NAN_HORIZONTAL    = 206, // 偏航控制量出现 NaN/Inf（第一次），同上
    EV_WARN_UART_RX_ERROR     = 207, // 串口接收错误（溢出/噪声/帧/校验），已自动重新开启接收；arg=HAL ErrorCode
    EV_WARN_CMD_OVERFLOW      = 208, // 命令缓冲满，丢弃一包；arg=包长度
    EV_WARN_UNKNOWN_CMD       = 209, // 收到无法识别的命令；text=命令前若干字符
    EV_WARN_TX_OVERFLOW       = 210, // 遥测发送缓冲满，丢弃输出；arg=丢弃字节数
    EV_WARN_EVENT_LOST        = 211, // 上位机请求补发的事件已被覆盖；arg=最早还能补发的序号
    EV_WARN_ERROR_HANDLER     = 212, // HAL 初始化 Error_Handler 被调用（程序继续运行）；arg=调用处地址，用 .map 查函数
    EV_WARN_BMI088_INIT       = 213, // BMI088 初始化失败（程序继续运行，姿态数据不可信）；arg=错误位
    EV_WARN_IST8310_INIT      = 214, // IST8310 初始化失败（程序继续运行）；arg=错误码
    EV_WARN_IMU_SPI_INIT      = 215, // IMU SPI1 初始化失败
    EV_WARN_SERVO_WRITE       = 216, // 舵机 PCA9685 写入失败；arg=舵机序号

    /* ---- 3xx 信息 ---- */
    EV_INFO_BOOT              = 301, // 上电/复位；arg=RCC->CSR 复位原因位（bit7 低功耗 6 窗口看门狗 5 独立看门狗 4 软件 3 上电 2 复位脚 1 欠压）
    EV_INFO_ARMED             = 302, // ON：深度闭环开启；arg=目标深度×10（mbar）
    EV_INFO_TES               = 303, // TES：手动 8 路 PWM 生效；arg=解析到的路数
    EV_INFO_ESKF_READY        = 304, // ESKF 完成重力标定和粗对准；arg=重力×1000（mm/s²）
    EV_INFO_ANGLE_ON          = 305, // RPY:ON 偏航闭环开启
    EV_INFO_ANGLE_OFF         = 306, // RPY:OFF / ACL:OF 偏航闭环关闭
    EV_INFO_YAW_LOCK          = 307, // ACL:ON 锁定当前航向；arg=目标偏航×100（度）；text=ESKF/IMU
    EV_INFO_CALIB_START       = 308, // CA：水压计零点标定开始（约 2 秒内控制暂停）
    EV_INFO_CALIB_RESULT      = 309, // 某一路标定结果；arg=零点×1000（mbar）；text=CH0..CH3
    EV_INFO_CALIB_DONE        = 310, // 水压计零点标定结束

    /* ---- 9xx 致命（单片机停止） ---- */
    EV_FATAL_HARDFAULT        = 901, // HardFault；arg=出错指令地址（任务中发生时有效）；text=CFSR/HFSR
    EV_FATAL_MEMMANAGE        = 902, // MemManage 内存保护错误
    EV_FATAL_BUSFAULT         = 903, // BusFault 总线错误
    EV_FATAL_USAGEFAULT       = 904, // UsageFault 非法指令/除零/未对齐
    EV_FATAL_UNHANDLED_IRQ    = 905, // 进入了没有处理函数的中断；arg=IRQ 编号
    EV_FATAL_ASSERT           = 906, // FreeRTOS configASSERT 失败；arg=行号；text=文件名
    EV_FATAL_STACK_OVERFLOW   = 907, // 任务栈溢出；text=任务名
    EV_FATAL_SCHEDULER_EXIT   = 908, // vTaskStartScheduler 返回（调度器启动失败）
};

/* 写入一条事件。任何上下文都可调用（任务、中断、调度器启动前）。
 * text 必须是静态字符串（只保存指针），可为 NULL。 */
void Event_Post(uint16_t code, int32_t arg);
void Event_PostText(uint16_t code, int32_t arg, const char *static_text);
/* 同一 code 在 min_interval_ms 内只记一次，避免故障抖动刷屏。 */
void Event_PostLimited(uint16_t code, int32_t arg, uint32_t min_interval_ms);
uint32_t Event_LastSeq(void);

/* 控制任务每个周期调用一次：发送新事件、补发请求、10 Hz 状态行。 */
void Telemetry_Step(void);
/* 上位机 "EVQ:<seq>"：从该序号开始重新发送。 */
void Telemetry_RequestResend(uint32_t from_seq);
/* HAL_UART_TxCpltCallback(huart6) 中调用。 */
void Telemetry_OnTxDone(void);
/* 原样写一段字符串到发送缓冲（任务上下文）。 */
void Telemetry_Write(const char *data, uint16_t len);

/* 状态行字段由推进器模块提供（不含 $ST、时间、序号和校验）。返回写入长度。 */
typedef int (*TelemetryStatusFn)(char *buf, int size);
void Telemetry_SetStatusSource(TelemetryStatusFn fn);
/* 曲线字段（不含 $PL 和时间），格式同上。 */
void Telemetry_SetPlotSource(TelemetryStatusFn fn);

/* 致命错误：关中断、尽力把推进器写回中位、反复通过串口发出事件，然后永远停在这里。 */
void Fins_Fatal(uint16_t code, int32_t arg, const char *text) __attribute__((noreturn));
void Fins_AssertFailed(const char *file, int line) __attribute__((noreturn));
void Fins_FaultReport(uint16_t code, uint32_t exc_return) __attribute__((noreturn));
void Fins_UnhandledIrq(void) __attribute__((noreturn));
void Fins_ErrorHandlerHit(uint32_t caller);
/* 由推进器模块注册：致命时把推进器写中位（不能阻塞太久）。 */
typedef void (*FatalNeutralFn)(void);
void Fins_SetFatalNeutral(FatalNeutralFn fn);

#ifdef __cplusplus
}
#endif

#endif // FINSROV_EVENT_LOG_H
