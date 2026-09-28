//
// Created by lixin on 2026/9/28.
//
#include "SensorTask.h"
#include "ControllerTask.h"

#include "IMU.h"
#include "Sensor.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

namespace
{
    /*
     * 保存 SensorTask 的任务句柄。
     * 创建成功以后，句柄指向 FreeRTOS 管理的任务对象。
     */
    TaskHandle_t sensor_task_handle = nullptr;

    /*
     * 静态任务需要任务控制块和任务栈。
     *
     * 当前 STM32F407 工程中，一个 StackType_t 占 4 字节。
     * 因此这里的 1024 个元素对应 4096 字节任务栈。
     * 后续加入 ESKF 后，还需要检查实际栈使用情况。
     */
    StaticTask_t sensor_task_tcb;
    StackType_t sensor_task_stack[1024];

    /*
     * 保存本任务已经收到的数据。
     *
     * IMU 和压力的更新速度不同，因此分别保存最新一帧。
     * 收到新的 IMU 时，不会清除之前收到的压力数据。
     *
     * 这里保存的是传感器输入，还不是 ESKF 的融合结果。
     */
    struct SensorTaskData
    {
        ImuSample latest_imu;
        PressureSample latest_pressure;

        uint32_t imu_received_count;
        uint32_t pressure_received_count;

        float accel_norm_m_s2;
        uint32_t gravity_sample_count;
        float gravity_sum;//m/s2
        float gravity;//m/s2
        bool gravity_ready;
    };
    /*
 * 启动时使用 200 帧加速度数据估计重力模长。
 *
 * 如果 IMU 有效数据频率约为 100 Hz，
 * 200 帧大约需要 2 秒。
 */
    constexpr uint32_t GRAVITY_CALIBRATION_SAMPLES = 200;

    /*
     * 加速度模长的粗略有效范围。
     *
     * 这里只用于排除明显错误值，
     * 不是最终的静止判断。
     */
    constexpr float GRAVITY_MIN_VALID_M_S2 = 8.0f;
    constexpr float GRAVITY_MAX_VALID_M_S2 = 11.0f;
    /*
     * 所有成员初始为 0。
     *
     * 这个对象只由 SensorTask 访问和修改。
     * 当前可以在调试器中查看，后续对外输出会提供独立接口。
     */
    SensorTaskData sensor_data = {};

    void ProcessImuSample(const ImuSample &sample)
    {
        /*
         * 保存这次收到的完整 IMU 数据。
         *
         * 这是结构体复制，gyro_、accel_、mag_、temp_
         * 以及 sequence 等成员都会一起复制。
         */
        sensor_data.latest_imu = sample;

        /*
         * 记录本任务成功接收并处理过多少份 IMU 数据。
         * 这个数量不等于传感器的帧编号。
         */
        ++sensor_data.imu_received_count;

        /*
         * 计算当前加速度向量的模长。
         *
         * 模长 = sqrt(ax² + ay² + az²)。
         * 这里只记录当前一帧的结果，还没有进行静止判断和多帧平均，
         * 因此这个值暂时不能当作已经标定好的 gravity。
         */
        const float ax = sample.accel_[0];
        const float ay = sample.accel_[1];
        const float az = sample.accel_[2];

        sensor_data.accel_norm_m_s2 =
            sqrtf(ax * ax + ay * ay + az * az);

        /*
         * 系统启动阶段，根据多帧加速度模长估计重力。
         *
         * 这里假设潜器启动时处于静止状态。
         * 每一帧只累计模长，不直接累计 x、y、z，
         * 因为不同姿态下三个轴的分量会变化，
         * 但静止时加速度向量的模长仍然接近重力。
         */
        if (!sensor_data.gravity_ready)
        {
            const float accel_norm = sensor_data.accel_norm_m_s2;

            /*
             * 排除 NaN、无穷大和明显不合理的读数。
             */
            if (isfinite(accel_norm) &&
                accel_norm >= GRAVITY_MIN_VALID_M_S2 &&
                accel_norm <= GRAVITY_MAX_VALID_M_S2)
            {
                sensor_data.gravity_sum += accel_norm;
                ++sensor_data.gravity_sample_count;

                /*
                 * 收集足够样本后计算平均值。
                 */
                if (sensor_data.gravity_sample_count >=
                    GRAVITY_CALIBRATION_SAMPLES)
                {
                    sensor_data.gravity =
                        sensor_data.gravity_sum/
                        static_cast<float>(sensor_data.gravity_sample_count);

                    sensor_data.gravity_ready = true;
                }
            }

            /*
             * 初始化阶段尚未完成。
             * 当前帧不进入 ESKF 预测和修正。
             */
            return;
        }

        /*
         * 只有 gravity_ready 为 true 后，
         * 才在这里加入 ESKF 的正式处理。
         */
    }

    void ProcessPressureSample(const PressureSample &sample)
    {
        /*
         * 保存本次收到的完整压力数据。
         *
         * 包括四路压力、有效通道掩码、深度和帧编号。
         * 当前只是保存输入，没有再次调用压力传感器驱动。
         */
        sensor_data.latest_pressure = sample;

        /*
         * 记录本任务实际接收到的压力帧数量。
         * 队列长度为 1，尚未取出的旧帧可能被新帧覆盖，
         * 因此接收数量不一定等于 frame_id。
         */
        ++sensor_data.pressure_received_count;

        /*
         * 后续在这里接入压力观测处理。
         * 压力姿态修正与深度估计会在这里连接到融合算法。
         */
    }

    void SensorTask(void *argument)
    {
        /*
         * 创建任务时没有传入额外参数。
         */
        (void)argument;

        for (;;)
        {
            /*
             * 尝试取出一份 IMU 数据。
             *
             * 队列已有数据时，立即返回。
             * 队列为空时，最多等待 1 ms 对应的系统节拍。
             *
             * 等待期间任务处于阻塞状态，不会一直占用 CPU。
             */
            ImuSample imu_sample = {};

            if (WaitImuSample(&imu_sample, 1))
            {
                ProcessImuSample(imu_sample);
            }

            /*
             * 每轮都检查压力队列。
             *
             * 这段代码放在上面的 if 外面，
             * 因此即使没有收到 IMU，也会检查压力数据。
             *
             * timeout 为 0，表示压力队列为空时立即返回。
             */
            PressureSample pressure_sample = {};

            if (WaitPressureSample(&pressure_sample, 0))
            {
                ProcessPressureSample(pressure_sample);
            }
        }
    }
}

/*
 * 对外提供的任务创建函数。
 * Usermain.cpp 通过这个函数创建 SensorTask。
 */
void StartSensorTask()
{
    /*
     * 创建静态任务。
     *
     * SensorTask 是任务入口函数。
     * "sensor_task" 是调试器中显示的任务名称。
     * 1024 是任务栈的元素数量。
     * nullptr 表示不传递额外参数。
     *
     * 当前使用优先级 2，低于 ControllerTask 的优先级 3。
     * 两个任务同时就绪时，ControllerTask 优先运行。
     */
    sensor_task_handle = xTaskCreateStatic(
        SensorTask,
        "sensor_task",
        1024,
        nullptr,
        2,
        sensor_task_stack,
        &sensor_task_tcb
    );

    configASSERT(sensor_task_handle != nullptr);
}