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
    /*
 * 将磁力计测得的机体坐标向量转换到世界坐标系。
 *
 * 当前四元数表示：
 * 世界坐标系向量经过旋转后得到机体坐标系向量。
 * 因此这里使用对应的机体到世界的旋转矩阵。
 */
    void RotateBodyToWorld(
        const float quaternion[4],
        const float body[3],
        float world[3])
    {
        const float w = quaternion[0];
        const float x = quaternion[1];
        const float y = quaternion[2];
        const float z = quaternion[3];

        const float r00 =
            1.0f - 2.0f * (y * y + z * z);

        const float r01 =
            2.0f * (x * y - w * z);

        const float r02 =
            2.0f * (x * z + w * y);

        const float r10 =
            2.0f * (x * y + w * z);

        const float r11 =
            1.0f - 2.0f * (x * x + z * z);

        const float r12 =
            2.0f * (y * z - w * x);

        const float r20 =
            2.0f * (x * z - w * y);

        const float r21 =
            2.0f * (y * z + w * x);

        const float r22 =
            1.0f - 2.0f * (x * x + y * y);

        world[0] =
            r00 * body[0] +
            r01 * body[1] +
            r02 * body[2];

        world[1] =
            r10 * body[0] +
            r11 * body[1] +
            r12 * body[2];

        world[2] =
            r20 * body[0] +
            r21 * body[1] +
            r22 * body[2];
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
    magnetic_reference_world_[0] = 0.0f;
    magnetic_reference_world_[1] = 0.0f;
    magnetic_reference_world_[2] = 0.0f;
    magnetic_reference_uT_ = 0.0f;
    magnetic_reference_ready_ = false;
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
    //depth_m_ = 0.0f;
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
/*
 * 使用磁力计修正 yaw。
 *
 * 磁力计只提供相对航向参考：
 *
 * 1. 第一帧有效磁力计数据只用于保存参考方向。
 * 2. 后续数据用于计算 yaw 误差。
 * 3. 数据异常时只拒绝当前观测，不改变当前姿态。
 * 4. 不因为磁力计异常停止推进器。
 */
bool AttitudeEskf::CorrectMagnetometer(
    const float mag_body_uT[3])
{
    /*
     * 地磁场通常在几十 uT 范围内。
     *
     * 这里的范围用于发现断线、未初始化
     * 或受到强磁干扰的情况。
     */
    constexpr float minimum_magnitude_uT = 5.0f;
    constexpr float maximum_magnitude_uT = 100.0f;

    /*
     * 如果磁场水平分量太小，
     * 就无法可靠判断 yaw。
     */
    constexpr float minimum_horizontal_ratio = 0.1f;

    /*
     * 磁场模长变化超过这个范围时，
     * 认为当前磁力计可能受到了外部磁干扰。
     */
    constexpr float magnitude_gate_uT = 25.0f;

    /*
     * 单次 yaw 观测的标准差。
     *
     * 磁力计容易受到电机、电源线和金属结构影响，
     * 所以这里不能设置得过小。
     */
    constexpr float heading_sigma_rad =
        10.0f * 0.01745329252f;

    const float heading_variance =
        heading_sigma_rad * heading_sigma_rad;

    /*
     * ESKF 尚未初始化时，
     * 当前四元数还没有物理意义。
     */
    if (!initialized_ ||
        mag_body_uT == nullptr ||
        !isfinite(mag_body_uT[0]) ||
        !isfinite(mag_body_uT[1]) ||
        !isfinite(mag_body_uT[2]))
    {
        return false;
    }

    /*
     * 计算磁场模长。
     */
    const float measured_magnitude_uT =
        sqrtf(
            mag_body_uT[0] * mag_body_uT[0] +
            mag_body_uT[1] * mag_body_uT[1] +
            mag_body_uT[2] * mag_body_uT[2]
        );

    /*
     * 模长不合理时拒绝当前观测。
     *
     * 这里只返回 false，
     * 不修改 ESKF，也不会让任务停止。
     */
    if (!isfinite(measured_magnitude_uT) ||
        measured_magnitude_uT < minimum_magnitude_uT ||
        measured_magnitude_uT > maximum_magnitude_uT)
    {
        return false;
    }

    /*
     * 磁力计原始数据是在机体坐标系中测量的。
     *
     * 先根据当前姿态估计，
     * 把它转换到世界坐标系。
     */
    float magnetic_world[3] = {};

    RotateBodyToWorld(
        quaternion_,
        mag_body_uT,
        magnetic_world
    );

    /*
     * yaw 只能通过水平面上的磁场分量判断。
     */
    const float horizontal_magnitude =
        sqrtf(
            magnetic_world[0] * magnetic_world[0] +
            magnetic_world[1] * magnetic_world[1]
        );

    if (!isfinite(horizontal_magnitude) ||
        horizontal_magnitude <
            minimum_horizontal_ratio * measured_magnitude_uT)
    {
        return false;
    }

    /*
     * 后面只使用方向，
     * 所以将磁场向量归一化。
     */
    if (!NormalizeVector3(magnetic_world))
    {
        return false;
    }

    /*
     * 第一次有效磁力计数据只建立相对参考。
     *
     * 这里不修改 quaternion_，
     * 所以不会因为上电时船头朝向不同而突然转动。
     */
    if (!magnetic_reference_ready_)
    {
        magnetic_reference_world_[0] =
            magnetic_world[0];

        magnetic_reference_world_[1] =
            magnetic_world[1];

        magnetic_reference_world_[2] =
            magnetic_world[2];

        magnetic_reference_uT_ =
            measured_magnitude_uT;

        magnetic_reference_ready_ = true;

        return true;
    }

    /*
     * 检查当前磁场模长是否与启动时接近。
     *
     * 金属物体、电机电流和电源线可能改变磁场模长。
     */
    if (fabsf(
            measured_magnitude_uT -
            magnetic_reference_uT_
        ) > magnitude_gate_uT)
    {
        return false;
    }

    /*
     * 计算启动参考方向和当前方向的水平航向角。
     */
    const float reference_heading_rad =
        atan2f(
            magnetic_reference_world_[1],
            magnetic_reference_world_[0]
        );

    const float current_heading_rad =
        atan2f(
            magnetic_world[1],
            magnetic_world[0]
        );

    /*
     * 计算“参考方向 - 当前方向”。
     *
     * 这个方向的符号很重要：
     *
     * 当前 yaw 如果向正方向漂移，
     * current_heading 会变大，
     * residual 就会变成负值，
     * ESKF 会向反方向修正。
     */
    const float raw_residual_rad =
        reference_heading_rad -
        current_heading_rad;

    /*
     * 将角度误差限制到 [-pi, pi]。
     */
    const float residual_rad =
        atan2f(
            sinf(raw_residual_rad),
            cosf(raw_residual_rad)
        );

    /*
     * 一次观测不允许产生过大的 yaw 跳变。
     *
     * 这只是拒绝异常观测，
     * 不会停止控制任务。
     */
    if (!isfinite(residual_rad) ||
        fabsf(residual_rad) > 0.8f)
    {
        return false;
    }

    /*
     * 磁力计这一帧只观测 yaw。
     *
     * 状态顺序是：
     *
     * error[0]：roll
     * error[1]：pitch
     * error[2]：yaw
     * error[3]：gyro bias x
     * error[4]：gyro bias y
     * error[5]：gyro bias z
     */
    float h[6] = {};
    h[2] = 1.0f;

    /*
     * ph = P * H^T。
     *
     * 因为 h 只有 yaw 位置为 1，
     * 所以 ph 实际上就是 covariance_ 的第 3 列。
     */
    float ph[6] = {};

    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned j = 0; j < 6; ++j)
        {
            ph[i] +=
                covariance_[i][j] * h[j];
        }
    }

    /*
     * S = H * P * H^T + R。
     *
     * S 是当前 yaw 误差预测方差
     * 加上磁力计测量噪声方差。
     */
    const float innovation_variance =
        ph[2] + heading_variance;

    if (!isfinite(innovation_variance) ||
        innovation_variance <= 1.0e-6f)
    {
        return false;
    }

    /*
     * K = P * H^T * S^-1。
     */
    float gain[6] = {};

    for (unsigned i = 0; i < 6; ++i)
    {
        gain[i] =
            ph[i] /
            innovation_variance;
    }

    /*
     * error = K * residual。
     *
     * 这是本次需要注入名义状态的误差。
     */
    float error[6] = {};

    for (unsigned i = 0; i < 6; ++i)
    {
        error[i] =
            gain[i] * residual_rad;
    }

    const float error_angle =
        sqrtf(
            error[0] * error[0] +
            error[1] * error[1] +
            error[2] * error[2]
        );

    /*
     * 单帧姿态修正不能过大。
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

            a[i][j] -=
                gain[i] * h[j];
        }
    }

    /*
     * 使用 Joseph 形式更新协方差：
     *
     * P = A P A^T + K R K^T
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

            next_covariance[i][j] +=
                gain[i] *
                heading_variance *
                gain[j];
        }
    }

    /*
     * 姿态误差注入后，
     * 将误差协方差转换回新的局部误差坐标。
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
     * 将三个姿态误差转换为误差四元数。
     */
    float correction_quaternion[4] = {};
    float corrected_quaternion[4] = {};

    if (!RotationVectorToQuaternion(
            error,
            correction_quaternion))
    {
        return false;
    }

    /*
     * 将小误差注入当前名义四元数。
     */
    MultiplyQuaternion(
        quaternion_,
        correction_quaternion,
        corrected_quaternion
    );

    const float quaternion_norm =
        sqrtf(
            corrected_quaternion[0] *
                corrected_quaternion[0] +
            corrected_quaternion[1] *
                corrected_quaternion[1] +
            corrected_quaternion[2] *
                corrected_quaternion[2] +
            corrected_quaternion[3] *
                corrected_quaternion[3]
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
     * 检查协方差是否仍然有效。
     */
    for (unsigned i = 0; i < 6; ++i)
    {
        if (!isfinite(corrected_covariance[i][i]) ||
            corrected_covariance[i][i] <= 0.0f)
        {
            return false;
        }

        /*
         * 强制恢复协方差矩阵对称性。
         */
        for (unsigned j = i + 1; j < 6; ++j)
        {
            const float average =
                0.5f *
                (
                    corrected_covariance[i][j] +
                    corrected_covariance[j][i]
                );

            corrected_covariance[i][j] =
                average;

            corrected_covariance[j][i] =
                average;
        }
    }

    /*
     * 所有计算成功后，
     * 最后一次性写回 ESKF 内部状态。
     */
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

    /*
     * 磁力计主要修正 yaw，
     * 但如果协方差中存在交叉相关，
     * 也可能间接修正陀螺仪零偏。
     */
    for (unsigned i = 0; i < 3; ++i)
    {
        gyro_bias_[i] +=
            error[i + 3];
    }

    ++sequence_;

    return true;
}
bool AttitudeEskf::CorrectPressure(const float depth_m[4],uint8_t valid_mask) {
    constexpr uint8_t all_pressure_mask = 0x0FU;

    if (!initialized_ || depth_m == nullptr || (valid_mask & all_pressure_mask) != all_pressure_mask)
    {
        return false;
    }
    for (uint32_t i = 0; i < 4; ++i)
    {
        if (!isfinite(depth_m[i]))
        {
            return false;
        }
    }
    /*
     * 空气中四路之间没有水柱差，读数相同会被误当成"水平"，
     * 把台架上的真实倾角往 0 拉。所有通道至少 5 cm 水深才用压差修正姿态。
     */
    constexpr float minimum_attitude_depth_m = 0.05f;
    for (uint32_t i = 0; i < 4; ++i)
    {
        if (depth_m[i] < minimum_attitude_depth_m)
        {
            return false;
        }
    }
    /*
     * 水压计相对舱体位置（V3.3 设计图 p7 / 外形.STEP，与 fins_v3 FusionConfiguration 一致）：
     *   0 左前 (+x,-y)   1 左后 (-x,-y)   2 右后 (-x,+y)   3 右前 (+x,+y)
     * 机体系 FRD：x 前、y 右、z 下。
     * 深度 d_i = d0 - x_i*sin(pitch) + y_i*cos(pitch)*sin(roll)，
     * 预测的"向下"方向 down = (-sin(pitch), cos(pitch)sin(roll), cos(pitch)cos(roll))。
     */
    constexpr float sensor_half_x_m = 0.1247f;
    constexpr float sensor_half_y_m = 0.0691f;

    // 前(0,3) 减 后(1,2)：抬头时前面变浅，结果为 -4*hx*sin(pitch)，正好等于 4*hx*down_x
    // （原来写成 后减前，俯仰修正方向反了）
    const float down_x =
    (depth_m[0] +
     depth_m[3] -
     depth_m[1] -
     depth_m[2]) /
    (4.0f * sensor_half_x_m);

    // 右(2,3) 减 左(0,1)：右倾时右边变深
    const float down_y =
    (depth_m[2] +
     depth_m[3] -
     depth_m[0] -
     depth_m[1]) /
    (4.0f * sensor_half_y_m);

    const float down_z_square =
    1.0f -
    down_x * down_x -
    down_y * down_y;

    if (!(down_z_square > 0.0f))
    {
        return false; // 压差对应的倾角超出物理范围（噪声或波浪），本次不修正
    }

    const float down_z =
    sqrtf(down_z_square);
    //恢复单位
    const float pressure_down_m_s2[3] =
    {
        gravity_m_s2_ * down_x,
        gravity_m_s2_ * down_y,
        gravity_m_s2_ * down_z
    };

    return CorrectAccelerometer(pressure_down_m_s2);
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