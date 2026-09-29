//
// Created by lixin on 2026/9/28.
//

#ifndef RM_FRAME_C_ATTITUDEESKF_H
#define RM_FRAME_C_ATTITUDEESKF_H
#include <stdint.h>

/*
 * quaternion 使用 [w, x, y, z] 顺序。
 * euler_rad 使用 [yaw, pitch, roll] 顺序，
 */
struct FusionState
{
    float quaternion[4];
    float gyro_bias[3];
    float euler_rad[3];
    /*
     * 启动阶段根据多帧加速度模长得到的重力值。
     */
    float gravity_m_s2;
    float depth_m;
    uint32_t sequence;
    bool initialized;
};

/*
 * 陀螺仪预测、加速度计卡尔曼修正、
 * 磁力计修正和压力修正后续逐步加入。
 */
class AttitudeEskf
{
public:
    AttitudeEskf();
    void Reset();

    /*
     * 使用静止时的加速度计数据初始化姿态。
     *
     */
    bool Initialize(
        const float down_m_s2[3],//初始化重力，0船体前方，1船体右方，2船体下方
        float gravity_m_s2,
        uint32_t time_us
    );

    bool PredictGyroscope(const float gyro_body_rad_s[3],uint32_t time_us);
    bool CorrectAccelerometer(const float down_m_s2[3]);//主要修正roll,pitch;使用磁力计后可以修正yaw

    FusionState State() const;
    bool IsInitialized() const;
    bool UpdateDepthEstimate(float depth_m);
private:
    float quaternion_[4];
    float gyro_bias_[3];
    float covariance_[6][6];//0~5 姿态误差:roll pitch yaw bias误差:bx by bz
    float gravity_m_s2_;
    float depth_m_;

    uint32_t time_us_;
    uint32_t sequence_;

    bool initialized_;
};

#endif // RM_FRAME_C_ATTITUDEESKF_H
