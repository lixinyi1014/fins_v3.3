//
// Created by lixin on 2026/9/28.
//
#include "AttitudeEskf.h"

#include <math.h>
#include <string.h>
#include <cstring>
namespace
{
    /*
     * 将 roll、pitch、yaw 转换为 [w, x, y, z] 四元数。
     */
    void EulerToQuaternion(
        float roll,
        float pitch,
        float yaw,
        float quaternion[4])
    {
        const float half_roll = roll * 0.5f;
        const float half_pitch = pitch * 0.5f;
        const float half_yaw = yaw * 0.5f;

        const float cr = cosf(half_roll);
        const float sr = sinf(half_roll);
        const float cp = cosf(half_pitch);
        const float sp = sinf(half_pitch);
        const float cy = cosf(half_yaw);
        const float sy = sinf(half_yaw);

        quaternion[0] =
            cr * cp * cy +
            sr * sp * sy;

        quaternion[1] =
            sr * cp * cy -
            cr * sp * sy;

        quaternion[2] =
            cr * sp * cy +
            sr * cp * sy;

        quaternion[3] =
            cr * cp * sy -
            sr * sp * cy;
    }

    /*
     * 将 [w, x, y, z] 四元数转换为：
     *
     * euler_rad[0]：yaw
     * euler_rad[1]：pitch
     * euler_rad[2]：roll
     */
    void QuaternionToEuler(
        const float quaternion[4],
        float euler_rad[3])
    {
        const float w = quaternion[0];
        const float x = quaternion[1];
        const float y = quaternion[2];
        const float z = quaternion[3];

        euler_rad[0] =
            atan2f(
                2.0f * (w * z + x * y),
                2.0f * (w * w + x * x) - 1.0f
            );

        float pitch_argument =
            -2.0f * (x * z - w * y);

        /*
         * 由于浮点误差，反三角函数输入可能略微超出 [-1, 1]。
         */
        if (pitch_argument > 1.0f)
        {
            pitch_argument = 1.0f;
        }
        else if (pitch_argument < -1.0f)
        {
            pitch_argument = -1.0f;
        }

        euler_rad[1] = asinf(pitch_argument);

        euler_rad[2] =
            atan2f(
                2.0f * (w * x + y * z),
                2.0f * (w * w + z * z) - 1.0f
            );
    }
    constexpr float kGyroNoise = 0.01f;     // 陀螺仪噪声，rad/√s
    constexpr float kGyroBiasWalk = 0.001f; // 零偏漂移，rad/s/√s
    /*
     * 根据向量 v 构造反对称矩阵 [v]x。
     */
    void BuildSkewMatrix(
        const float v[3],
        float matrix[3][3])
    {
        matrix[0][0] = 0.0f;
        matrix[0][1] = -v[2];
        matrix[0][2] = v[1];

        matrix[1][0] = v[2];
        matrix[1][1] = 0.0f;
        matrix[1][2] = -v[0];

        matrix[2][0] = -v[1];
        matrix[2][1] = v[0];
        matrix[2][2] = 0.0f;
    }
    bool NormalizeVector3(float vector[3])
    {
        const float norm =
            sqrtf(
                vector[0] * vector[0] +
                vector[1] * vector[1] +
                vector[2] * vector[2]
            );

        if (!isfinite(norm) || norm <= 1.0e-6f)
        {
            return false;
        }

        vector[0] /= norm;
        vector[1] /= norm;
        vector[2] /= norm;

        return true;
    }
    //根据四元数，预测向下方向
    /*当前四元数认为的姿态
            ↓
        预测加速度计应该看到的重力方向
            ↓
        和真实加速度计方向进行比较
     */
    void CalculatePredictedDown(const float quaternion[4],float predicted_down[3]){
        const float w = quaternion[0];
        const float x = quaternion[1];
        const float y = quaternion[2];
        const float z = quaternion[3];

        predicted_down[0] =
            2.0f * (x * z - w * y);

        predicted_down[1] =
            2.0f * (y * z + w * x);

        predicted_down[2] =
            1.0f - 2.0f * (x * x + y * y);
    }
    //四元数相乘
    /*
    * MultiplyQuaternion(quaternion_,correction_quaternion,
    *   corrected_quaternion);
     */
    void MultiplyQuaternion(
    const float left[4],
    const float right[4],
    float out[4])
    {
        const float lw = left[0];
        const float lx = left[1];
        const float ly = left[2];
        const float lz = left[3];

        const float rw = right[0];
        const float rx = right[1];
        const float ry = right[2];
        const float rz = right[3];

        out[0] =
            lw * rw -
            lx * rx -
            ly * ry -
            lz * rz;

        out[1] =
            lw * rx +
            lx * rw +
            ly * rz -
            lz * ry;

        out[2] =
            lw * ry -
            lx * rz +
            ly * rw +
            lz * rx;

        out[3] =
            lw * rz +
            lx * ry -
            ly * rx +
            lz * rw;
    }
    bool RotationVectorToQuaternion(const float rotation_vector[3],float quaternion[4])
    {
        const float angle =
            sqrtf(
                rotation_vector[0] * rotation_vector[0] +
                rotation_vector[1] * rotation_vector[1] +
                rotation_vector[2] * rotation_vector[2]
            );

        if (!isfinite(angle))
        {
            return false;
        }

        if (angle <= 1.0e-6f)
        {
            quaternion[0] = 1.0f;
            quaternion[1] = 0.5f * rotation_vector[0];
            quaternion[2] = 0.5f * rotation_vector[1];
            quaternion[3] = 0.5f * rotation_vector[2];
            return true;
        }

        const float half_angle =
            0.5f * angle;

        const float scale =
            sinf(half_angle) / angle;

        quaternion[0] = cosf(half_angle);
        quaternion[1] = scale * rotation_vector[0];
        quaternion[2] = scale * rotation_vector[1];
        quaternion[3] = scale * rotation_vector[2];

        return true;
    }
    bool InvertMatrix3(
    const float matrix[3][3],
    float inverse[3][3])
    {
        const float determinant =
            matrix[0][0] *
            (matrix[1][1] * matrix[2][2] -
             matrix[1][2] * matrix[2][1])
            -
            matrix[0][1] *
            (matrix[1][0] * matrix[2][2] -
             matrix[1][2] * matrix[2][0])
            +
            matrix[0][2] *
            (matrix[1][0] * matrix[2][1] -
             matrix[1][1] * matrix[2][0]);

        if (!isfinite(determinant) ||
            fabsf(determinant) < 1.0e-9f)
        {
            return false;
        }

        const float inv_determinant =
            1.0f / determinant;

        inverse[0][0] =
            (matrix[1][1] * matrix[2][2] -
             matrix[1][2] * matrix[2][1]) *
            inv_determinant;

        inverse[0][1] =
            (matrix[0][2] * matrix[2][1] -
             matrix[0][1] * matrix[2][2]) *
            inv_determinant;

        inverse[0][2] =
            (matrix[0][1] * matrix[1][2] -
             matrix[0][2] * matrix[1][1]) *
            inv_determinant;

        inverse[1][0] =
            (matrix[1][2] * matrix[2][0] -
             matrix[1][0] * matrix[2][2]) *
            inv_determinant;

        inverse[1][1] =
            (matrix[0][0] * matrix[2][2] -
             matrix[0][2] * matrix[2][0]) *
            inv_determinant;

        inverse[1][2] =
            (matrix[0][2] * matrix[1][0] -
             matrix[0][0] * matrix[1][2]) *
            inv_determinant;

        inverse[2][0] =
            (matrix[1][0] * matrix[2][1] -
             matrix[1][1] * matrix[2][0]) *
            inv_determinant;

        inverse[2][1] =
            (matrix[0][1] * matrix[2][0] -
             matrix[0][0] * matrix[2][1]) *
            inv_determinant;

        inverse[2][2] =
            (matrix[0][0] * matrix[1][1] -
             matrix[0][1] * matrix[1][0]) *
            inv_determinant;

        return true;
    }
    /*
     * next = F * P * Fᵀ + Q
     */
    bool PropagateCovariance(
        const float p[6][6],
        const float omega[3],
        float dt,
        float next[6][6])
    {
        // ---- 1. F = 单位矩阵 ----
        float f[6][6] = {};
        for (unsigned i = 0; i < 6; ++i)
        {
            f[i][i] = 1.0f;
        }

        // 左上：I - [ω]x·dt（转动时误差在三轴之间转移）
        f[0][1] =  omega[2] * dt;
        f[0][2] = -omega[1] * dt;
        f[1][0] = -omega[2] * dt;
        f[1][2] =  omega[0] * dt;
        f[2][0] =  omega[1] * dt;
        f[2][1] = -omega[0] * dt;

        // 右上：-I·dt（零偏误差被积分成角度误差）
        f[0][3] = -dt;
        f[1][4] = -dt;
        f[2][5] = -dt;

        // ---- 2. fp = F * P ----
        float fp[6][6] = {};
        for (unsigned i = 0; i < 6; ++i)
            for (unsigned j = 0; j < 6; ++j)
                for (unsigned k = 0; k < 6; ++k)
                    fp[i][j] += f[i][k] * p[k][j];

        // ---- 3. next = fp * Fᵀ（Fᵀ[k][j] 就是 f[j][k]）----
        for (unsigned i = 0; i < 6; ++i)
            for (unsigned j = 0; j < 6; ++j)
            {
                float sum = 0.0f;
                for (unsigned k = 0; k < 6; ++k)
                    sum += fp[i][k] * f[j][k];
                next[i][j] = sum;
            }

        // ---- 4. 加噪声 Q ----
        /*
         * 陀螺仪白噪声和零偏随机游走的连续噪声强度。
         */
        const float gyro_q =
            kGyroNoise * kGyroNoise;

        const float bias_q =
            kGyroBiasWalk * kGyroBiasWalk;

        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;

        for (unsigned i = 0; i < 3; ++i)
        {
            /*
             * 陀螺仪测量噪声直接影响姿态误差。
             */
            next[i][i] += gyro_q * dt;

            /*
             * 零偏随机游走会影响零偏本身，
             * 同时也会通过积分影响姿态。
             */
            next[i][i] += bias_q * dt3 / 3.0f;

            next[i + 3][i + 3] += bias_q * dt;

            /*
             * 姿态误差和零偏误差之间存在相关性。
             * 零偏越大，经过时间积分后，
             * 姿态误差也会越大。
             */
            next[i][i + 3] -=
                bias_q * dt2 / 2.0f;

            next[i + 3][i] -=
                bias_q * dt2 / 2.0f;
        }

        // ---- 5. 强制对称 + 检查 ----
        for (unsigned i = 0; i < 6; ++i)
        {
            for (unsigned j = i + 1; j < 6; ++j)
            {
                const float avg = 0.5f * (next[i][j] + next[j][i]);
                next[i][j] = avg;
                next[j][i] = avg;
            }
        }

        for (unsigned i = 0; i < 6; ++i)
        {
            if (!isfinite(next[i][i]) || next[i][i] <= 0.0f)
                return false;
        }

        return true;
    }
}

AttitudeEskf::AttitudeEskf()
{
    Reset();
}

void AttitudeEskf::Reset()
{
    /*
     * 单位四元数表示没有旋转。
     */
    quaternion_[0] = 1.0f;
    quaternion_[1] = 0.0f;
    quaternion_[2] = 0.0f;
    quaternion_[3] = 0.0f;

    /*
     * 初始时还不知道陀螺仪剩余零偏。
     */
    gyro_bias_[0] = 0.0f;
    gyro_bias_[1] = 0.0f;
    gyro_bias_[2] = 0.0f;
    float temp[6][6] = {
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    };//方差：rpyxyz
    std::memcpy(covariance_, temp, sizeof(covariance_));
    const float attitude_sigma_rad = 6.0f * 0.01745329252f;
    covariance_[0][0] = attitude_sigma_rad * attitude_sigma_rad; // roll
    covariance_[1][1] = attitude_sigma_rad * attitude_sigma_rad; // pitch
    covariance_[2][2] = 1.0f;  // yaw：没有磁力计，粗对准时直接设成 0，约 57°，几乎不知道

    covariance_[3][3] = 0.1f * 0.1f;  // 零偏 x，rad²/s²
    covariance_[4][4] = 0.1f * 0.1f;  // 零偏 y
    covariance_[5][5] = 0.1f * 0.1f;  // 零偏 z
    gravity_m_s2_ = 0.0f;
    depth_m_ = 0.0f;
    time_us_ = 0;
    sequence_ = 0;
    initialized_ = false;
}

bool AttitudeEskf::Initialize(
    const float down_m_s2[3],
    float gravity_m_s2,
    uint32_t time_us)
{
    if (down_m_s2 == nullptr)
    {
        return false;
    }

    if (!isfinite(down_m_s2[0]) ||
        !isfinite(down_m_s2[1]) ||
        !isfinite(down_m_s2[2]) ||
        !isfinite(gravity_m_s2) ||
        gravity_m_s2 <= 0.0f)
    {
        return false;
    }

    const float ax = down_m_s2[0];
    const float ay = down_m_s2[1];
    const float az = down_m_s2[2];

    const float norm =
        sqrtf(ax * ax + ay * ay + az * az);

    /*
     * 如果当前加速度模长与启动阶段得到的重力差异过大，
     * 说明潜器可能正在运动，不能用于粗对准。
     */
    if (fabsf(norm - gravity_m_s2) > 2.0f)
    {
        return false;
    }

    /*
     * 使用重力方向计算横滚和俯仰。
     *
     * 当前约定：
     * euler_rad[0]：yaw
     * euler_rad[1]：pitch
     * euler_rad[2]：roll
     *
     * 没有磁力计航向参考时，yaw 暂时设为 0。
     */
    const float roll =
        atan2f(ay, az);

    const float pitch =
        atan2f(
            -ax,
            sqrtf(ay * ay + az * az)
        );

    const float yaw = 0.0f;

    EulerToQuaternion(
        roll,
        pitch,
        yaw,
        quaternion_
    );

    gravity_m_s2_ = gravity_m_s2;
    depth_m_ = 0.0f;
    time_us_ = time_us;
    initialized_ = true;
    ++sequence_;

    return true;
}
bool AttitudeEskf::PredictGyroscope(
    const float gyro_body_rad_s[3],
    uint32_t time_us)
{
    if (!initialized_)
    {
        return false;
    }

    if (gyro_body_rad_s == nullptr ||
        !isfinite(gyro_body_rad_s[0]) ||
        !isfinite(gyro_body_rad_s[1]) ||
        !isfinite(gyro_body_rad_s[2]))
    {
        return false;
    }
    const uint32_t elapsed_us = time_us - time_us_;

    if (elapsed_us == 0U)          // 同一时间，不预测
    {
        return false;
    }

    if (elapsed_us > 100000U)      // 断帧过长，重新计时
    {
        time_us_ = time_us;
        return false;
    }

    const float dt = static_cast<float>(elapsed_us) * 1.0e-6f;

    // 去掉零偏的角速度
    const float omega[3] = {
        gyro_body_rad_s[0] - gyro_bias_[0],
        gyro_body_rad_s[1] - gyro_bias_[1],
        gyro_body_rad_s[2] - gyro_bias_[2]
    };

    const float w = quaternion_[0];
    const float x = quaternion_[1];
    const float y = quaternion_[2];
    const float z = quaternion_[3];
    const float h = 0.5f * dt;

    // ---- 候选四元数（先不写回）----
    float q_new[4] = {
        w + (-x * omega[0] - y * omega[1] - z * omega[2]) * h,
        x + ( w * omega[0] + y * omega[2] - z * omega[1]) * h,
        y + ( w * omega[1] - x * omega[2] + z * omega[0]) * h,
        z + ( w * omega[2] + x * omega[1] - y * omega[0]) * h
    };

    const float norm = sqrtf(q_new[0] * q_new[0] + q_new[1] * q_new[1] +
                             q_new[2] * q_new[2] + q_new[3] * q_new[3]);

    if (!isfinite(norm) || norm <= 1.0e-6f)
    {
        return false;
    }

    for (unsigned i = 0; i < 4; ++i)
    {
        q_new[i] /= norm;
    }

    // ---- 候选协方差（先不写回）----
    float p_new[6][6];

    if (!PropagateCovariance(covariance_, omega, dt, p_new))
    {
        return false;
    }

    // ---- 两个都成功，一起写回 ----
    memcpy(quaternion_, q_new, sizeof(quaternion_));
    memcpy(covariance_, p_new, sizeof(covariance_));

    time_us_ = time_us;
    ++sequence_;

    return true;
}
bool AttitudeEskf::CorrectAccelerometer(
    const float down_m_s2[3])
{
    if (!initialized_ ||
        down_m_s2 == nullptr ||
        !isfinite(down_m_s2[0]) ||
        !isfinite(down_m_s2[1]) ||
        !isfinite(down_m_s2[2]))
    {
        return false;
    }

    float measured_down[3] =
    {
        down_m_s2[0],
        down_m_s2[1],
        down_m_s2[2]
    };

    const float measured_norm =
        sqrtf(
            measured_down[0] * measured_down[0] +
            measured_down[1] * measured_down[1] +
            measured_down[2] * measured_down[2]
        );

    /*
     * 如果加速度模长明显偏离启动阶段的重力值，
     * 说明当前可能存在大幅运动或碰撞。
     *
     * 这时加速度计不再只表示重力方向，
     * 不能直接用于姿态修正。
     */
    if (!isfinite(measured_norm) ||
        fabsf(measured_norm - gravity_m_s2_) > 2.0f)
    {
        return false;
    }

    /*
     * 姿态修正只使用方向， 不使用加速度大小。
     */
    if (!NormalizeVector3(measured_down))
    {
        return false;
    }

    float predicted_down[3] = {};

    CalculatePredictedDown(
        quaternion_,
        predicted_down
    );

    if (!NormalizeVector3(predicted_down))
    {
        return false;
    }

    /*
     * residual = 实际重力方向 - 理论重力方向。
     */
    float residual[3] =
    {
        measured_down[0] - predicted_down[0],
        measured_down[1] - predicted_down[1],
        measured_down[2] - predicted_down[2]
    };

    /*
     * H 的前 3 列对应姿态误差，
     * 后 3 列对应陀螺仪零偏。
     *
     * 加速度方向不会测量陀螺仪零偏，
     * 因此后 3 列为 0。
     */
    float h[3][6] = {};
    float skew[3][3] = {};

    BuildSkewMatrix(
        predicted_down,
        skew
    );

    for (unsigned i = 0; i < 3; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            h[i][j] = skew[i][j];
        }
    }

    /*
     * ph = P * Hᵀ。
     *
     * ph 是 6×3 矩阵。
     */
    float ph[6][3] = {};

    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            for (unsigned k = 0; k < 6; ++k)
            {
                ph[i][j] +=
                    covariance_[i][k] * h[j][k];
            }
        }
    }

    /*
     * S = H * P * Hᵀ + R。
     *
     * S 表示这次观测误差理论上应该有多大。
     */
    float innovation[3][3] = {};
    const float direction_sigma = 0.08f;
    const float direction_variance =
        direction_sigma * direction_sigma;

    for (unsigned i = 0; i < 3; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            for (unsigned k = 0; k < 6; ++k)
            {
                innovation[i][j] +=
                    h[i][k] * ph[k][j];
            }
        }

        innovation[i][i] +=
            direction_variance;
    }

    float innovation_inverse[3][3] = {};

    if (!InvertMatrix3(
            innovation,
            innovation_inverse))
    {
        return false;
    }

    /*
     * K = P * Hᵀ * S⁻¹。
     */
    float gain[6][3] = {};

    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            for (unsigned k = 0; k < 3; ++k)
            {
                gain[i][j] +=
                    ph[i][k] *
                    innovation_inverse[k][j];
            }
        }
    }

    /*
     * error = K * residual。
     *
     * error[0~2]：姿态小误差。
     * error[3~5]：陀螺仪零偏修正量。
     */
    float error[6] = {};

    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            error[i] +=
                gain[i][j] * residual[j];
        }
    }

    const float error_angle =
        sqrtf(
            error[0] * error[0] +
            error[1] * error[1] +
            error[2] * error[2]
        );

    /*
     * 单帧修正不能过大。
     *
     * 如果突然算出很大的姿态修正，
     * 通常说明传感器数据、坐标变换或时间顺序有问题。
     */
    if (!isfinite(error_angle) ||
        error_angle > 0.5f)
    {
        return false;
    }

    /*
     * A = I - K * H。
     */
    float a[6][6] = {};

    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 6; ++j)
        {
            a[i][j] =
                (i == j) ? 1.0f : 0.0f;

            for (unsigned k = 0; k < 3; ++k)
            {
                a[i][j] -=
                    gain[i][k] * h[k][j];
            }
        }
    }

    /*
     * 使用 Joseph 形式更新协方差：
     *
     * P = A P Aᵀ + K R Kᵀ
     *
     * 这种形式比直接写成
     * P = (I-KH)P 更不容易破坏对称性和正定性。
     */
    float ap[6][6] = {};
    float next_covariance[6][6] = {};

    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 6; ++j)
        {
            for (unsigned k = 0; k < 6; ++k)
            {
                ap[i][j] +=
                    a[i][k] * covariance_[k][j];
            }
        }
    }

    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 6; ++j)
        {
            for (unsigned k = 0; k < 6; ++k)
            {
                next_covariance[i][j] +=
                    ap[i][k] * a[j][k];
            }

            for (unsigned k = 0; k < 3; ++k)
            {
                next_covariance[i][j] +=
                    gain[i][k] *
                    direction_variance *
                    gain[j][k];
            }
        }
    }

    /*
     * 注入姿态误差后，重新设置局部误差坐标。
     */
    float error_skew[3][3] = {};

    BuildSkewMatrix(
        error,
        error_skew
    );

    float reset_matrix[6][6] = {};

    for (unsigned i = 0; i < 6; ++i)
    {
        reset_matrix[i][i] = 1.0f;
    }

    for (unsigned i = 0; i < 3; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            reset_matrix[i][j] -=
                0.5f * error_skew[i][j];
        }
    }

    float reset_left[6][6] = {};
    float corrected_covariance[6][6] = {};

    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 6; ++j)
        {
            for (unsigned k = 0; k < 6; ++k)
            {
                reset_left[i][j] +=
                    reset_matrix[i][k] *
                    next_covariance[k][j];
            }
        }
    }

    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 6; ++j)
        {
            for (unsigned k = 0; k < 6; ++k)
            {
                corrected_covariance[i][j] +=
                    reset_left[i][k] *
                    reset_matrix[j][k];
            }
        }
    }

    /*
     * 误差四元数右乘到当前名义四元数。
     */
    float correction_quaternion[4] = {};
    float corrected_quaternion[4] = {};

    if (!RotationVectorToQuaternion(
            error,
            correction_quaternion))
    {
        return false;
    }

    MultiplyQuaternion(
        quaternion_,
        correction_quaternion,
        corrected_quaternion
    );

    const float quaternion_norm =
        sqrtf(
            corrected_quaternion[0] * corrected_quaternion[0] +
            corrected_quaternion[1] * corrected_quaternion[1] +
            corrected_quaternion[2] * corrected_quaternion[2] +
            corrected_quaternion[3] * corrected_quaternion[3]
        );

    if (!isfinite(quaternion_norm) ||
        quaternion_norm <= 1.0e-6f)
    {
        return false;
    }

    for (unsigned i = 0; i < 4; ++i)
    {
        corrected_quaternion[i] /=
            quaternion_norm;
    }

    /*
     * 最后检查协方差。
     *
     * 所有计算都成功后，才一次性写回成员变量。
     */
    for (unsigned i = 0; i < 6; ++i)
    {
        if (!isfinite(corrected_covariance[i][i]) ||
            corrected_covariance[i][i] <= 0.0f)
        {
            return false;
        }

        for (unsigned j = i + 1; j < 6; ++j)
        {
            const float average =
                0.5f *
                (corrected_covariance[i][j] +
                 corrected_covariance[j][i]);

            corrected_covariance[i][j] = average;
            corrected_covariance[j][i] = average;
        }
    }

    memcpy(
        quaternion_,
        corrected_quaternion,
        sizeof(quaternion_)
    );

    memcpy(
        covariance_,
        corrected_covariance,
        sizeof(covariance_)
    );

    for (unsigned i = 0; i < 3; ++i)
    {
        gyro_bias_[i] += error[i + 3];
    }

    ++sequence_;

    return true;
}
FusionState AttitudeEskf::State() const
{
    FusionState state = {};

    memcpy(
        state.quaternion,
        quaternion_,
        sizeof(quaternion_)
    );

    memcpy(
        state.gyro_bias,
        gyro_bias_,
        sizeof(gyro_bias_)
    );

    QuaternionToEuler(
        quaternion_,
        state.euler_rad
    );

    state.gravity_m_s2 = gravity_m_s2_;
    state.depth_m = depth_m_;
    state.sequence = sequence_;
    state.initialized = initialized_;

    return state;
}

bool AttitudeEskf::IsInitialized() const
{
    return initialized_;
}
bool AttitudeEskf::UpdateDepthEstimate(
    float depth_m)
{
    if (!isfinite(depth_m) ||
        depth_m < 0.0f)
    {
        return false;
    }

    /*
     * 保存已经通过 SensorTask 检查的深度。
     */
    depth_m_ = depth_m;

    /*
     * 表示融合输出状态发生了一次更新。
     */
    ++sequence_;

    return true;
}