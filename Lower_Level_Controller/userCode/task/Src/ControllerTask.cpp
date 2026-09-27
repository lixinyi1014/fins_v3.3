//
// Created by lixinyi on 2026/9/26.
//

#include "ControllerTask.h"
#include "Sensor.h"

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

    /*
     * FreeRTOS 任务句柄。
     * 句柄可以理解为“这个任务的编号”。
     * 中断通过它找到 ControllerTask。
     */
    TaskHandle_t controller_task_handle = nullptr;
    QueueHandle_t pressure_queue = nullptr;

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
             * 这部分就是原来 Usermain.cpp 中
             * while(1) 里面的设备循环。
             */
            for (uint32_t i = 0; i < device_count; ++i)
            {
                device_list[i]->Handle();
            }
            /*
 * 所有设备完成本次 Handle() 后，
 * 才尝试获取最新的压力快照。
 *
 * PressureSensor::Handle() 可能只执行了
 * GET_TEMPERATURE、GET_PRESSURE 或 CALCULATE
 * 其中一个阶段。
 *
 * 只有 CALCULATE 阶段完成后，
 * CopyLatestSample() 才会返回 true。
 */
            PressureSample sample;

            if (PressureSensor::pressure_sensor.CopyLatestSample(&sample))
            {
                if (pressure_queue != nullptr)
                {
                    xQueueOverwrite(pressure_queue, &sample);
                }
            }
        }
    }
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
void StartControllerTask(Device *devices[], uint32_t count)
{
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