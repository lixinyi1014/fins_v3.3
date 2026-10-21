//
// Created by lixinyi on 2026/9/26.
//

#include "ControllerTask.h"
#include "Sensor.h"
#include "IMU.h"
#include "EventLog.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//下面这些变量只在本文件内部使用。

namespace
{
    //保存 Usermain.cpp 传进来的设备数组。
    Device **device_list = nullptr;
    // 保存设备数量。
    uint32_t device_count = 0;
    // 设备名（只用 ASCII，上报"某模块卡住"用），可为空
    const char *const *device_names = nullptr;
    // 单个模块 Handle() 超过这个时间就上报 218（正常整个周期 < 6.7 ms）
    constexpr uint32_t kSlowHandleMs = 20;
    uint32_t last_slow_report_ms = 0;
    // 模块预先声明"这一步会很久"（水压标定约 2.4 s），本周期不报 218
    volatile bool expect_long_step = false;

    /*
     * FreeRTOS 任务句柄。
     * 句柄可以理解为“这个任务的编号”。
     * 中断通过它找到 ControllerTask。
     */
    TaskHandle_t controller_task_handle = nullptr;
    QueueHandle_t pressure_queue = nullptr;
    QueueHandle_t imu_queue = nullptr;
    StaticQueue_t imu_queue_tcb;
    uint8_t imu_queue_storage[8 * sizeof(ImuSample)];

    StaticQueue_t pressure_queue_tcb;
    uint8_t pressure_queue_storage[sizeof(PressureSample)];
    /*
     * 静态任务需要两块内存：
     * 1. controller_task_stack：任务运行时使用的栈
     * 2. controller_task_tcb：FreeRTOS 保存任务状态的控制块
     */
    StaticTask_t controller_task_tcb;
    StackType_t controller_task_stack[1024];
    /*
 * FreeRTOS 自己也需要一个 Idle Task。
 * 因为关闭了动态内存，所以 Idle Task 的内存也必须静态提供。
 */
    StaticTask_t idle_task_tcb;
    StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

    /*
     * 串口命令缓冲：单生产者（USART6 中断）/ 单消费者（本任务）环形队列。
     * 不调用任何 FreeRTOS API，所以 USART6 中断优先级保持原来的 4 也没问题。
     */
    constexpr uint32_t kCommandSlots = 8;
    struct CommandPacket
    {
        uint16_t length;
        char data[SERIAL_LENGTH_MAX];
    };
    CommandPacket command_slots[kCommandSlots];
    volatile uint32_t command_write = 0; // 只由中断修改
    volatile uint32_t command_read = 0;  // 只由本任务修改
    volatile uint32_t silent_cycles = 0;

    // 已知命令前缀，只用于识别"未知命令"并上报警告，不影响各设备自己的解析
    const char *const kKnownCommands[] = {
        "ON", "OFF", "UP", "DN", "W", "A", "S", "D", "Q", "E", "Z",
        "H:", "RPY:", "ACL:", "ANG:", "TES:", "MOT:", "CA", "VA",
        "BEG", "END", "IVA:", "HB", "EVQ:", "FB:", "LVL"};

    bool IsKnownCommand(const char *line, size_t length)
    {
        for (const char *known : kKnownCommands)
        {
            const size_t n = strlen(known);
            if (n == 1)
            {
                if (length == 1 && line[0] == known[0])
                    return true;
            }
            else if (length >= n && strncmp(line, known, n) == 0)
            {
                return true;
            }
        }
        return false;
    }

    /*
     * 处理一行命令：每个设备都拿到一份完整、以 0 结尾的原始命令
     * （舵机等模块会用 strtok 改写缓冲，所以每个设备之前都重新复制）。
     */
    void DispatchLine(const char *line, size_t length)
    {
        if (length == 0)
            return;
        if (length >= SERIAL_LENGTH_MAX)
            length = SERIAL_LENGTH_MAX - 1;
        if (length == 2 && line[0] == 'H' && line[1] == 'B')
            return; // 心跳：只刷新存活时间
        if (length > 4 && strncmp(line, "EVQ:", 4) == 0)
        {
            char number[16] = {};
            memcpy(number, line + 4, length - 4 < sizeof(number) - 1 ? length - 4 : sizeof(number) - 1);
            Telemetry_RequestResend(strtoul(number, nullptr, 10));
            return;
        }
        if (!IsKnownCommand(line, length))
        {
            // 未知命令只上报，照常分发（设备自己会忽略）
            // 事件里只存指针，所以轮流使用 8 个静态缓冲，补发时文字不会被后来的命令覆盖
            static char unknown_text[8][12];
            static uint32_t unknown_index = 0;
            char *text = unknown_text[unknown_index++ % 8];
            const size_t n = length < 11 ? length : 11;
            memcpy(text, line, n);
            text[n] = '\0';
            Event_PostText(EV_WARN_UNKNOWN_CMD, static_cast<int32_t>(length), text);
        }
        for (uint32_t i = 0; i < device_count; ++i)
        {
            memset(RxBuffer, 0, SERIAL_LENGTH_MAX);
            memcpy(RxBuffer, line, length);
            device_list[i]->Receive();
        }
        memset(RxBuffer, 0, SERIAL_LENGTH_MAX);
    }

    /*
     * 取出本周期收到的所有数据包，按 \r \n ; 拆成多条命令。
     * 旧上位机发命令不带换行，整包就是一条命令，同样兼容。
     * 返回本周期是否收到过数据（用于心跳）。
     */
    bool DispatchReceivedCommands()
    {
        bool received = false;
        while (command_read != command_write)
        {
            const CommandPacket &packet = command_slots[command_read % kCommandSlots];
            received = true;
            size_t start = 0;
            for (size_t i = 0; i <= packet.length; ++i)
            {
                const bool end = i == packet.length;
                const char c = end ? '\n' : packet.data[i];
                if (c == '\r' || c == '\n' || c == ';' || c == '\0')
                {
                    DispatchLine(packet.data + start, i - start);
                    start = i + 1;
                }
            }
            __DMB();
            command_read = command_read + 1;
        }
        return received;
    }

    /*
     * 这就是实际运行的任务函数。
     * FreeRTOS 任务函数必须是：
     * void 函数，接收一个 void* 参数。
     */
    void ControllerTask(void *argument)
    {
        /*
         * 这次暂时没有使用参数。
         */
        (void)argument;

        /*
         * FreeRTOS 任务不能执行一次就返回。
         * 所以使用无限循环。
         */
        uint32_t last_pressure_frame = 0;
        uint32_t last_imu_sequence = 0;
        for (;;)
        {
            /*
             * 等待 TIM1 中断发来的通知。
             *
             * pdTRUE：
             *   醒来时把通知计数清零。
             *   如果任务处理期间来了多个定时器中断，
             *   这里只处理一次最新节拍，避免任务越积越多。
             *
             * portMAX_DELAY：
             *   没有通知时一直阻塞。
             *   阻塞时不占用 CPU。
             */
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            /*
             * 先处理上一周期内收到的命令，再执行设备 Handle()。
             * 收到任何数据都算一次心跳。
             */
            if (DispatchReceivedCommands())
                silent_cycles = 0;
            else if (silent_cycles < 0xFFFFFFFFU)
                silent_cycles = silent_cycles + 1;

            /*
             * 这部分就是原来 Usermain.cpp 中
             * while(1) 里面的设备循环。
             */
            // 串口接收若意外停了（重开失败等），这里补开，否则心跳会断
            Uart6_EnsureReceiving();

            for (uint32_t i = 0; i < device_count; ++i)
            {
                const uint32_t start_ms = HAL_GetTick();
                device_list[i]->Handle();
                const uint32_t used_ms = HAL_GetTick() - start_ms;
                // 某个模块把控制任务卡住（I2C 等待等）：卡住期间遥测、心跳检查、停机都不执行，事后补报是谁、多久
                if (expect_long_step)
                {
                    expect_long_step = false;
                }
                else if (used_ms >= kSlowHandleMs && HAL_GetTick() - last_slow_report_ms >= 1000U)
                {
                    last_slow_report_ms = HAL_GetTick();
                    Event_PostText(EV_WARN_LOOP_OVERRUN, static_cast<int32_t>(used_ms),
                                   device_names ? device_names[i] : "?");
                }
            }
            /*
 * 所有设备完成本次 Handle() 后，
 * 才尝试获取最新的压力快照。
 *
 * PressureSensor::Handle() 可能只执行了
 * GET_TEMPERATURE、GET_PRESSURE 或 CALCULATE
 * 其中一个阶段。
 *
 * 只有初始化第一帧阶段完成后，
 * CopyLatestSample() 才会返回 true。
 */
            PressureSample sample = {};

            if (PressureSensor::pressure_sensor.CopyLatestSample(&sample))
            {
                if (sample.frame_id != last_pressure_frame)
                {
                    if (pressure_queue != nullptr)
                    {
                        /*
                         * 队列保存数据副本。
                         * 长度为 1，新的帧替换尚未取走的旧帧。
                         */
                        xQueueOverwrite(pressure_queue, &sample);

                        /*
                         * 保存本次已经发布的帧号，
                         * 供下次任务循环比较。
                         */
                        last_pressure_frame = sample.frame_id;
                    }
                }
            }
            ImuSample imu_sample = {};
            if (IMU::imu.CopyLatestSample(&imu_sample))
            {
                if (imu_sample.sequence != last_imu_sequence)
                {
                    if (imu_queue != nullptr)
                    {
                        /*
                         * timeout 为 0，表示这里不能因为等待队列阻塞控制任务。
                         */
                        if (xQueueSend(imu_queue, &imu_sample, 0) == pdPASS)
                        {
                            /*
                             * 只有成功放入队列之后，
                             * 才认为这个序号已经发布。
                             *
                             * 如果队列已满，暂时不修改 last_imu_sequence，
                             * 下一个控制周期还会尝试发送。
                             */
                            last_imu_sequence = imu_sample.sequence;
                        }
                    }
                }
            }
            /*
             * 发送事件和状态行（只写入发送缓冲，不会阻塞）。
             */
            Telemetry_Step();
        }
    }
}

bool CommandQueue_PushFromISR(const uint8_t *data, uint16_t length)
{
    if (command_write - command_read >= kCommandSlots)
        return false; // 满了，丢弃这一包
    CommandPacket &packet = command_slots[command_write % kCommandSlots];
    if (length > SERIAL_LENGTH_MAX - 1)
        length = SERIAL_LENGTH_MAX - 1;
    memcpy(packet.data, data, length);
    packet.data[length] = '\0';
    packet.length = length;
    __DMB();
    command_write = command_write + 1;
    return true;
}

uint32_t CommandSilentCycles(void)
{
    return silent_cycles;
}

/*
 * 任务栈溢出（configCHECK_FOR_STACK_OVERFLOW = 2 时由内核调用）。
 */
extern "C" void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    Fins_Fatal(EV_FATAL_STACK_OVERFLOW, 0, task_name);
}
/*
 * FreeRTOS 启动调度器时会调用这个函数，
 * 取得 Idle Task 的控制块和栈。z这是优先级最低的任务
 *
 * extern "C" 很重要：
 * FreeRTOS 内核是 C 文件编译的，
 * 必须让链接器看到没有 C++ 名字改编的函数名。
 */
extern "C" void vApplicationGetIdleTaskMemory(
    StaticTask_t **task_tcb,
    StackType_t **task_stack,
    uint32_t *stack_size)
{
    *task_tcb = &idle_task_tcb;
    *task_stack = idle_task_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}

/*
 * 在 main() 中调用这个函数创建任务。
 */
void Controller_ExpectLongStep(void)
{
    expect_long_step = true;
}

void StartControllerTask(Device *devices[], uint32_t count, const char *const *names)
{
    device_names = names;
    /*
     * 保存设备数组地址和设备数量。
     * 这些值在启动调度器前设置好，之后不会改变。
     */
    device_list = devices;
    device_count = count;

    pressure_queue = xQueueCreateStatic(
    1,
    sizeof(PressureSample),
    pressure_queue_storage,
    &pressure_queue_tcb
);

    configASSERT(pressure_queue != nullptr);
    imu_queue = xQueueCreateStatic(
    8,
    sizeof(ImuSample),
    imu_queue_storage,
    &imu_queue_tcb
);

    configASSERT(imu_queue != nullptr);
    /*
     * 创建静态任务。
     *
     * 参数依次是：
     *
     * 1. ControllerTask：任务函数
     * 2. "device_task"：任务名字，调试时查看
     * 3. 1024：栈深度，单位是 StackType_t 个元素
     * 4. nullptr：不向任务传参数
     * 5. 3：任务优先级，当前配置允许 0 到 5
     * 6. controller_task_stack：任务栈
     * 7. controller_task_tcb：任务控制块
     */

    controller_task_handle = xTaskCreateStatic(
        ControllerTask,
        "device_task",
        1024,
        nullptr,
        3,
        controller_task_stack,
        &controller_task_tcb
    );

    /*
     * 如果创建失败，就停在 configASSERT。
     * 失败通常表示栈或任务控制块配置有问题。
     */
    configASSERT(controller_task_handle != nullptr);
}

/*
 * 这个函数由 TIM1 的中断回调调用。
 */
void ControllerTickFromISR(void)
{
    //在任务创建之前，定时器可能已经启动。句柄为空时不能发送通知。
    if (controller_task_handle == nullptr) return;
    /*
     * 调度器还没启动时，不能使用正常的任务调度动作。
     */
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return;
    }

    /*
     * 这个变量由 FreeRTOS 修改。
     * 如果通知让更高优先级任务变为就绪，
     * 它会被设置为 pdTRUE。
     */
    BaseType_t higher_priority_task_woken = pdFALSE;

    /*
     * 给 ControllerTask 增加一个通知。
     * 函数名中的 FromISR 表示“从中断中调用”。
     */
    vTaskNotifyGiveFromISR(
        controller_task_handle,
        &higher_priority_task_woken
    );

    /*
     * 如果刚才的通知唤醒了更高优先级任务，
     * 请求 FreeRTOS 在退出中断时立即切换过去。
     */
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

bool WaitPressureSample(PressureSample *sample, uint32_t timeout_ms)
{
    if (sample == nullptr)
    {
        return false;
    }


    if (pressure_queue == nullptr)
    {
        return false;
    }

    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms);

    return xQueueReceive(
        pressure_queue,
        sample,
        wait_ticks
    ) == pdPASS;
}
bool WaitImuSample(ImuSample *sample, uint32_t timeout_ms)
{

    if (sample == nullptr)
    {
        return false;
    }

    /*
     * ControllerTask 创建完成之前，队列句柄为空。
     */
    if (imu_queue == nullptr)
    {
        return false;
    }

    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms);

    return xQueueReceive(
        imu_queue,
        sample,
        wait_ticks
    ) == pdPASS;
}