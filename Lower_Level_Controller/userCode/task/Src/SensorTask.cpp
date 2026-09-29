//
// Created by lixin on 2026/9/28.
//
#include "SensorTask.h"
#include "ControllerTask.h"
#include "AttitudeEskf.h"

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
    struct BodyImuSample
    {
        float gyro_body[3];
        float down_m_s2[3];
        float mag_body[3];
    };
    struct SensorTaskData
    {
        uint32_t last_pressure_frame_id;
        ImuSample latest_imu;
        PressureSample latest_pressure;

        uint32_t imu_received_count;
        uint32_t pressure_received_count;
        uint32_t last_imu_sequence;

        float accel_norm_m_s2;
        uint32_t gravity_sample_count;
        float gravity_sum;//m/s2
        float gravity;//m/s2
        bool gravity_ready;
        FusionState fusion_state;
        /*
        * 保存当前最新一份已经转换到船体坐标的数据。
        */
        BodyImuSample body_imu;

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
    AttitudeEskf attitude_eskf;
    SensorTaskData sensor_data = {};
    BodyImuSample ConvertImuToBody(const ImuSample &sample)
    {
        BodyImuSample converted = {};

        converted.gyro_body[0] = sample.gyro_[0];
        converted.gyro_body[1] = -sample.gyro_[1];
        converted.gyro_body[2] = -sample.gyro_[2];

        const float accel_body[3] =
        {
            sample.accel_[0],
            -sample.accel_[1],
            -sample.accel_[2]
        };

        converted.down_m_s2[0] = -accel_body[0];
        converted.down_m_s2[1] = -accel_body[1];
        converted.down_m_s2[2] = -accel_body[2];

        converted.mag_body[0] = -sample.mag_[1];
        converted.mag_body[1] = -sample.mag_[0];
        converted.mag_body[2] = -sample.mag_[2];

        return converted;
    }
    void ProcessImuSample(const ImuSample &sample)
    {
        if (sample.sequence == 0)
        {
            return;
        }

        /*
         * 如果序号与上一帧相同，
         * 说明这不是新的 IMU 数据。
         */
        if (sample.sequence == sensor_data.last_imu_sequence)
        {
            return;
        }
        /*
         * 保存这次收到的完整 IMU 数据。
         *
         * 这是结构体复制，gyro_、accel_、mag_、temp_
         * 以及 sequence 等成员都会一起复制。
         */
        sensor_data.last_imu_sequence = sample.sequence;
        sensor_data.latest_imu = sample;
        sensor_data.body_imu = ConvertImuToBody(sample);
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
        const float dx = sensor_data.body_imu.down_m_s2[0];
        const float dy = sensor_data.body_imu.down_m_s2[1];
        const float dz = sensor_data.body_imu.down_m_s2[2];

        sensor_data.accel_norm_m_s2 =
            sqrtf(dx * dx + dy * dy + dz * dz);

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
         * 重力初始化完成后，使用当前静止加速度
         * 对 ESKF 做第一次姿态粗对准。
         */
        if (!attitude_eskf.IsInitialized())
        {
            /*
             * 当前 received_ms 是 ControllerTask 复制数据的时间。
             * 它暂时只用于让初始化接口拥有一个时间参数。
             *
             * 后续接入真正的 BMI088 传感器时间后，
             * 这里必须替换成统一的 sample_us。
             */
            const uint32_t temporary_time_us =
                sample.received_ms * 1000U;

            if (attitude_eskf.Initialize(
                sensor_data.body_imu.down_m_s2,
                sensor_data.gravity,
                temporary_time_us))
            {
                /*
                 * 保存一次初始化后的融合状态，
                 * 方便调试器查看姿态是否合理。
                 */
                sensor_data.fusion_state =
                    attitude_eskf.State();
            }

            /*
             * 初始化调用结束后，本帧不再执行其他融合步骤。
             */
            return;
        }

        /*
         * 当前 received_ms 是 ControllerTask 复制数据的时间。
         *
         * 目前先把它转换成微秒，
         * 作为陀螺仪预测的临时时间轴。
         *
         * 后续接入真正的 BMI088 采样时间后，
         * 这里必须改成统一的 sample_us。
         */
        const uint32_t temporary_time_us =
            sample.received_ms * 1000U;

        /*
         * 使用已经转换到船体坐标的陀螺仪数据，
         * 对 ESKF 当前四元数进行一次预测。
         */
        if (attitude_eskf.PredictGyroscope(
                sensor_data.body_imu.gyro_body,
                temporary_time_us))
        {
            /*
             * 第一步：陀螺仪把姿态向前预测。
             *
             * 陀螺仪擅长描述短时间旋转，
             * 但长期积分会产生漂移。
             */
            attitude_eskf.CorrectAccelerometer(
                sensor_data.body_imu.down_m_s2
            );

            /*
             * 第二步：加速度计利用重力方向修正姿态。
             *
             * 如果当前存在明显平动，
             * CorrectAccelerometer() 会拒绝本帧，
             * 这时仍然保留陀螺仪预测结果。
             */
            sensor_data.fusion_state =
                attitude_eskf.State();
        }
    }

    void ProcessPressureSample(const PressureSample &sample)
    {
        if (sample.frame_id == 0)
        {
            return;
        }

        if (sample.frame_id ==
            sensor_data.last_pressure_frame_id)
        {
            return;
        }

        /*
         * 保存原始快照供调试查看。
         */
        sensor_data.latest_pressure =
            sample;

        float depth_sum = 0.0f;
        uint32_t valid_count = 0;

        for (uint32_t i = 0;
             i < SENSOR_NUM;
             ++i)
        {
            /*
             * valid_mask 的第 i 位表示第 i 路压力计是否有效。
             */
            const uint8_t valid_bit =
                static_cast<uint8_t>(1U << i);

            if ((sample.valid_mask & valid_bit) == 0)
            {
                continue;
            }

            const float current_depth =
                sample.depth_m[i];

            /*
             * 排除异常深度。
             */
            if (!isfinite(current_depth) ||
                current_depth < -0.5f ||
                current_depth > 100.0f)
            {
                continue;
            }

            depth_sum += current_depth;
            ++valid_count;
        }

        /*
         * 没有任何有效通道时，
         * 当前压力帧不能更新深度。
         */
        if (valid_count == 0)
        {
            return;
        }

        float depth_m =
            depth_sum /
            static_cast<float>(valid_count);

        /*
         * 水面零偏可能得到很小的负值，
         * 将其限制到 0 m。
         */
        if (depth_m < 0.0f)
        {
            depth_m = 0.0f;
        }

        if (!attitude_eskf.UpdateDepthEstimate(depth_m))
        {
            return;
        }

        sensor_data.last_pressure_frame_id =
            sample.frame_id;

        ++sensor_data.pressure_received_count;

        /*
         * 重新读取统一融合状态。
         * 其中包括姿态、陀螺仪零偏和深度。
         */
        sensor_data.fusion_state =
            attitude_eskf.State();
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