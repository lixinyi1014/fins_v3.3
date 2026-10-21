//
// Created by admin on 2023/10/30.
//
#include "Propeller.h"
#include "Sensor.h"
#include "IMU.h"
#include <algorithm>
#include <cmath>
#include "SensorTask.h" //提供GetFusionState()
#include "AttitudeEskf.h" //提供FusionState
#include "ControllerTask.h" //提供CommandSilentCycles()
#include "EventLog.h"

namespace
{
Propeller_I2C *g_propeller = nullptr;
int StatusFieldsThunk(char *buf, int size) { return g_propeller ? g_propeller->FormatStatusFields(buf, size) : 0; }
int PlotFieldsThunk(char *buf, int size) { return g_propeller ? g_propeller->FormatPlotFields(buf, size) : 0; }
int PidFieldsThunk(char *buf, int size) { return g_propeller ? g_propeller->FormatPidFields(buf, size) : 0; }
void FatalNeutralThunk() { if (g_propeller) g_propeller->FatalNeutral(); }
// 水压计相对舱体中心的位置（与 AttitudeEskf::CorrectPressure 相同，V3.3 设计图/STEP）
constexpr float kSensorHalfX_m = 0.1247f;
constexpr float kSensorHalfY_m = 0.0691f;
// 积分暂停：目标深度在变、水平运动中，以及之后 0.8 s，深度和俯仰外环都不积分；深度误差超过 6 mbar 也不积分。
// 这些时候的误差是暂时的（拖缆、推力线不过重心），积进去松手后会过冲。6 mbar 时 P 已有 60 us，够顶住悬停所需
constexpr uint32_t kIntegralHoldAfter_ms = 800;
constexpr float kDepthIntegralBand_mbar = 6.0f;
}

// V33
std::array<int8_t, 8> Sign = {1, -1, 1, 1, -1, -1, 1, -1}; // Mark propeller direction: Clockwise = 1, Counter-Clockwise = -1.; the index corresponds to the propeller ID. // 推进器正反桨，正1反-1，序号为推进器序号
std::array<uint8_t, 4> InID = {1, 2, 6, 5};                // V33-1,2 Channel indices on the expansion board for the four internal thrusters: front-left, rear-left, front-right, rear-right. //内部的4个推进器接到扩展板上的序号，左前-左后-右前-右后
std::array<uint8_t, 4> OutID = {0, 3, 7, 4};               // V33-1,2 Channel indices on the expansion board for the four external thrusters: front-left, rear-left, front-right, rear-right. //外部的4个推进器接到扩展板上的序号，左前-左后-右前-右后
int32_t InitPWM = 1550; // Initial PWM value for thruster initialization. // 推进器初始化的PWM（V33-0,1）
//int32_t InitPWM = 1540; // Initial PWM value for thruster initialization. //推进器初始化的PWM（V33-2）
// 死区补偿：控制量离开中位时直接跳过电调死区。2026-10-19 水中实测（ch1）：PWM ≥1605 开始正转、≤1490 开始反转，
// 死区 1490～1605 相对中位 1550 不对称（-60 / +55）。原来两边都补 50，两侧各有 5～10 us 落在死区里没推力，
// 控制量过零时推进器在"不转"和"突然出力"之间来回跳，前面两个垂直推进器约 1 Hz 抖动就是这个
std::array<int32_t, 8> Compensation = {55, 55, 55, 55, 55, 55, 55, 55};       // PWM 高于中位时加：1605 - 1550
std::array<int32_t, 8> CompensationDown = {60, 60, 60, 60, 60, 60, 60, 60};   // PWM 低于中位时减：1550 - 1490
// Vertical thrusters.
// 垂直推进器
std::array<int32_t, 4> FloatPWM = {
    InitPWM,
    InitPWM - Sign[InID[1]] * 50,
    InitPWM,
    InitPWM - Sign[InID[3]] * 45
};
// Horizontal thrusters.
// 水平推进器
// 2026-10-20 遥控速度全部减半（原 80 / 80 / 40）：拖缆时推力线不过重心造成的俯仰、深度干扰和电池压降都小一半
uint8_t longitudinal_speed = 60;   // 2026-10-21 前后 40→60（向前走不动）；左右、旋转不变
uint8_t lateral_speed = 40;
uint8_t rotate_speed = 20;
std::array<int32_t, 4> StopPWM = {InitPWM, InitPWM, InitPWM, InitPWM};
std::array<int32_t, 4> FrontPWM = {
    InitPWM - Sign[OutID[0]] * longitudinal_speed,
    InitPWM - Sign[OutID[1]] * longitudinal_speed,
    InitPWM - Sign[OutID[2]] * longitudinal_speed,
    InitPWM - Sign[OutID[3]] * longitudinal_speed
};
std::array<int32_t, 4> BackPWM = {
    InitPWM + Sign[OutID[0]] * longitudinal_speed,
    InitPWM + Sign[OutID[1]] * longitudinal_speed,
    InitPWM + Sign[OutID[2]] * longitudinal_speed,
    InitPWM + Sign[OutID[3]] * longitudinal_speed
};
std::array<int32_t, 4> LeftPWM = {
    InitPWM + Sign[OutID[0]] * lateral_speed,
    InitPWM - Sign[OutID[1]] * lateral_speed,
    InitPWM - Sign[OutID[2]] * lateral_speed,
    InitPWM + Sign[OutID[3]] * lateral_speed
};
std::array<int32_t, 4> RightPWM = {
    InitPWM - Sign[OutID[0]] * lateral_speed,
    InitPWM + Sign[OutID[1]] * lateral_speed,
    InitPWM + Sign[OutID[2]] * lateral_speed,
    InitPWM - Sign[OutID[3]] * lateral_speed
};
std::array<int32_t, 4> ClockwisePWM = {
    InitPWM - Sign[OutID[0]] * rotate_speed,
    InitPWM - Sign[OutID[1]] * rotate_speed,
    InitPWM + Sign[OutID[2]] * rotate_speed,
    InitPWM + Sign[OutID[3]] * rotate_speed
};
std::array<int32_t, 4> AntiClockwisePWM = {
    InitPWM + Sign[OutID[0]] * rotate_speed,
    InitPWM + Sign[OutID[1]] * rotate_speed,
    InitPWM - Sign[OutID[2]] * rotate_speed,
    InitPWM - Sign[OutID[3]] * rotate_speed
};
// PID
PID_Regulator_t DepthPID(10, 0.02, 10, 100, 50, 50, 200);
PID_Regulator_t PitchPID(10, 0.02, 50, 50, 25, 25, 100);
PID_Regulator_t RollPID(1.8, 0.02, 100, 50, 25, 25, 100);
PID_Regulator_t YawPID(12, 0.06, 300, 10, 100, 50, 300);
PID_Regulator_t YawInPID(5, 0, 0, 200, 100, 100, 400);
PID_Regulator_t YawOutPID(2, 0.01, 2, 10, 5, 5, 20);
PID_Regulator_t RollInPID(30, 0, 0, 100, 50, 50, 200); 
PID_Regulator_t RollOutPID(0.2, 0.002, 0.06, 1.0, 0.5, 0.5, 10); 
PID_Regulator_t PitchInPID(40, 0, 0, 100, 50, 50, 200);
PID_Regulator_t PitchOutPID(0.22, 0.002, 0.06, 1.4, 1.2, 0.5, 10); 
// IMU
// PID_Regulator_t PitchInPID(2, 0, 40, 100, 50, 50, 200);
// PID_Regulator_t PitchOutPID(7, 0.5, 100, 5, 2.5, 2.5, 10);

Propeller_Parameter_t Parameter_V33 = {
    Sign, InID, OutID, InitPWM,
    Compensation,
    FloatPWM, FrontPWM, BackPWM, LeftPWM, RightPWM,
    ClockwisePWM, AntiClockwisePWM, StopPWM,
    DepthPID, PitchPID, RollPID, YawPID,
    YawInPID, YawOutPID, RollInPID, RollOutPID, PitchInPID, PitchOutPID
};

Propeller_Command_t Propeller_I2C::propeller_command[] = {
    {3, "ACL", &Propeller_I2C::command_set_angle_closeloop},
    {3, "ANG", &Propeller_I2C::command_set_angle_value},
    {3, "TES", &Propeller_I2C::command_test}
};

void Propeller_I2C::Init()
{
    //--TODO=这里的初始化报错还没有改过，未来有可能看一下，servo也没加
    Parameter = Parameter_V33;

    DepthPID.PIDInfo = Parameter.DepthPID_P;
    PitchPID.PIDInfo = Parameter.PitchPID_P;
    RollPID.PIDInfo = Parameter.RollPID_P;
    YawPID.PIDInfo = Parameter.YawPID_P;

    PitchAnglePID.PIDInfo = Parameter.PitchPID_P;
    RollAnglePID.PIDInfo = Parameter.RollPID_P;
    YawAnglePID.PIDInfo = Parameter.YawPID_P;

    YawInPID.PIDInfo = Parameter.YawInPID_P;
    YawOutPID.PIDInfo = Parameter.YawOutPID_P;

    RollInPID.PIDInfo = Parameter.RollInPID_P;
    RollOutPID.PIDInfo = Parameter.RollOutPID_P;

    PitchInPID.PIDInfo = Parameter.PitchInPID_P;
    PitchOutPID.PIDInfo = Parameter.PitchOutPID_P;

    Target_speed[0] = 0;
    Target_speed[1] = 0;
    flag_float = false;
    flag_roll = false;
    flag_PWM_output = false;
    flag_angle = false;
    roll_state = 0;
    roll_state_total = 1;
    motion_state = STOP;

    msg_state_map['W'] = FRONT;
    msg_state_map['S'] = BACK;
    msg_state_map['A'] = LEFT;
    msg_state_map['D'] = RIGHT;
    msg_state_map['Z'] = FLOAT;
    msg_state_map['E'] = CLOCKWISE;
    msg_state_map['Q'] = ANTICLOCKWISE;

    state_PWM_map[STOP]          = Parameter.StopPWM;
    state_PWM_map[FRONT]         = Parameter.FrontPWM;
    state_PWM_map[BACK]          = Parameter.BackPWM;
    state_PWM_map[LEFT]          = Parameter.LeftPWM;
    state_PWM_map[RIGHT]         = Parameter.RightPWM;
    state_PWM_map[FLOAT]         = Parameter.StopPWM;
    state_PWM_map[CLOCKWISE]     = Parameter.ClockwisePWM;
    state_PWM_map[ANTICLOCKWISE] = Parameter.AnticlockwisePWM;

    g_propeller = this;
    Telemetry_SetStatusSource(StatusFieldsThunk);
    Telemetry_SetPlotSource(PlotFieldsThunk);
    Telemetry_SetPidSource(PidFieldsThunk);
    Fins_SetFatalNeutral(FatalNeutralThunk);

    TCA_SetChannel(4);  // Expand a single I2C bus into eight channels. // 1路 I2C 扩展为 8路
    HAL_Delay(5);
    PCA_Write(PCA9685_MODE1, 0x0);
    PCA_Setfreq(50); // Hz
    for (int i = 0; i < PROPELLER_NUM; ++i)
    {
        current_PWM[i] = Parameter.InitPWM;
        PCA_Setpwm(i, 0, floor(current_PWM[i] * 4096 / 20000 + 0.5f));
    }

    // 串口接收统一在 main() 里开启（使用独立的中断接收缓冲）
};

// Configure the PWM parameters according to UART commands.
// 根据串口指令设置PWM参数
void Propeller_I2C::Receive()
{
    int32_t data_receive[8];
    (void)data_receive;
    // RxBuffer 由控制任务按行拆好、以 0 结尾
    size_t rx_length = 0;
    while (rx_length < SERIAL_LENGTH_MAX && RxBuffer[rx_length] != '\0')
        ++rx_length;  
    // OFF 在任何模式下都生效（原来只在深度闭环开启时才处理，TES 手动输出关不掉）
    if (rx_length == 3 && strncmp((char *)RxBuffer, "OFF", 3) == 0)
    {
        Stop(EV_STOP_USER_OFF, 0);
        return;
    }

    // 反馈来源开关：任何模式下都可切换。切换时清掉横滚/俯仰外环，避免两种反馈的微小差异让积分跳变
    const bool fb_prs = rx_length == 6 && strncmp((char *)RxBuffer, "FB:PRS", 6) == 0;
    const bool fb_eskf = rx_length == 7 && strncmp((char *)RxBuffer, "FB:ESKF", 7) == 0;
    if (fb_prs || fb_eskf)
    {
        feedback_fusion_ = fb_eskf;
        RollOutPID.Reset();
        PitchOutPID.Reset();
        Event_Post(EV_INFO_FEEDBACK_SOURCE, feedback_fusion_ ? 1 : 0);
        return;
    }

    uint8_t command_num = sizeof(propeller_command) / sizeof(Propeller_Command_t);
    for (uint8_t i=0; i<command_num; i++)
    {
        if (strncmp((char *)RxBuffer, propeller_command[i].str, propeller_command[i].len) == 0)
        {
            (this->*(propeller_command[i].handler))();
        }
    }
    if (flag_float)
    {
			// Update the PWM outputs of the four outer-ring motors for front/back/left/right motion (open-loop motion is possible when yaw-angle control is disabled).
        // 外圈四个电机更新为前后左右运动的PWM(在没有yaw角控制的情况下可开环运动)
        if (strncmp((char *)RxBuffer, "DN", 2) == 0)
        {
            Target_depth += 1;
            return;
        }

        else if (strncmp((char *)RxBuffer, "UP", 2) == 0)
        {
            Target_depth -= 1;
            return;
        }

        // 运动命令只认单个字符（原来只看首字母，"ACL"/"ANG" 会被当成 A=左移，"END" 被当成 E=顺时针）
        else if (rx_length == 1 && msg_state_map.count(RxBuffer[0]))
        {
            motion_state = msg_state_map[RxBuffer[0]];
            // if(flag_angle){
            //     for (int i = 0; i < 4; ++i)
            //     {
            //         current_PWM[Parameter.OutID[i]] = current_PWM[Parameter.OutID[i]] + state_PWM_map[motion_state][i] - Parameter.InitPWM;
            //     }
            // }
            // else{
            //     for (int i = 0; i < 4; ++i)
            //     {
            //         current_PWM[Parameter.OutID[i]] = state_PWM_map[motion_state][i];
            //     }
            // }
        }

				// Disable PID control and reset the PWM outputs to their initial values.
        // 关闭PID控制，更新为初始的PWM
        // OFF 已移到函数开头统一处理（Stop）

        // // 外圈四个电机更新为给定的PWM，更新深度
        // if (strncmp((char *)RxBuffer, "PRO:", 4) == 0)
        // {
        //     char *data_str = (char *)RxBuffer + 4;
        //     char *token = strtok(data_str, ",");
        //     int i = 0;
        //     while (token != NULL && i < 5)
        //     {
        //         data_receive[i] = atoi(token);
        //         token = strtok(NULL, ",");
        //         i++;
        //     }
        //     for (int i = 0; i < 4; ++i)
        //     {
        //         current_PWM[Parameter.OutID[i]] = data_receive[i];
        //     }
        //     Target_depth = data_receive[4] / 10.0;
        // }

        // // 更新速度与角速度
        // if (strncmp((char *)RxBuffer, "VEL:", 4) == 0)
        // {
        //     char *data_str = (char *)RxBuffer + 4;
        //     char *token = strtok(data_str, ",");
        //     int i = 0;
        //     while (token != NULL && i < 3)
        //     {
        //         data_receive[i] = atoi(token);
        //         token = strtok(NULL, ",");
        //         i++;
        //     }
        //     Target_speed[0] = data_receive[0];
        //     Target_speed[1] = data_receive[1];
        //     Target_yaw = data_receive[2] * 3.14 / 180;
        // }

        // Update Yaw. //更新Yaw角度
        else if (strncmp((char *)RxBuffer, "RPY:", 4) == 0)
        {
            if (strncmp((char *)RxBuffer, "RPY:ON", 6) == 0){
                flag_angle = true;
                Event_Post(EV_INFO_ANGLE_ON, 0);
            }
            else if (strncmp((char *)RxBuffer, "RPY:OFF", 7) == 0){
                flag_angle = false;
                Event_Post(EV_INFO_ANGLE_OFF, 0);
                current_PWM[Parameter.OutID[0]] = current_PWM[Parameter.OutID[1]] = current_PWM[Parameter.OutID[2]] = current_PWM[Parameter.OutID[3]] = Parameter.InitPWM;
            }
        //     else {
        //         char *data_str = (char *)RxBuffer + 4;
        //         char *token = strtok(data_str, ",");
        //         int i = 0;
        //         while (token != NULL && i < 3)
        //         {
        //             data_receive[i] = atoi(token);
        //             token = strtok(NULL, ",");
        //             i++;
        //         }
        //         Target_roll = data_receive[0] * 3.14 / 180;
        //         Target_pitch = data_receive[1] * 3.14 / 180;
        //         Target_yaw = data_receive[2] * 3.14 / 180;
        //     }
        }

        // // 当开启角度控制时先默认输出角度数据，可以通过串口输入改为输出PWM数据
        // if(flag_angle){
        //     if (strncmp((char *)RxBuffer, "PWM:BEG", 7) == 0){
        //         flag_PWM_output = true;
        //     }
        //     else if (strncmp((char *)RxBuffer, "PWM:END", 7) == 0){
        //         flag_PWM_output = false;
        //     }
        // }

        // Update Depth. //更新深度
        else if (strncmp((char *)RxBuffer, "H:", 2) == 0)
        {
            char *data_str = (char *)RxBuffer + 2;
            Target_depth = atoi(data_str) / 10.0;
        }
    }

    else
    {
				// Enable PID control and initialize the PWM outputs to their default values.
        // 开启PID控制，更新为初始PWM
        if (strncmp((char *)RxBuffer, "ON", 2) == 0)
        {
            flag_float = true;
            for (int i = 0; i < 8; ++i)
            {
                current_PWM[i] = Parameter.InitPWM;
            }
            // ON 时的目标深度（与原版仓库相同）：已经比 30 mbar（约 30 cm）深就保持当前深度（减 1 mbar），否则下潜到 30 mbar
            Target_depth = (PressureSensor::pressure_sensor.data_depth - 1.0 > 30.0) ? (PressureSensor::pressure_sensor.data_depth - 1.0) : 30.0; // 与原版仓库相同
            stop_code_ = 0;
            nan_vertical_streak_ = nan_horizontal_streak_ = 0;
            Event_Post(EV_INFO_ARMED, static_cast<int32_t>(Target_depth * 10.0f));
        }
    }
}

void Propeller_I2C::Handle()
{
    //推进器在出力时，连续 100 个周期（约 0.67 s）没收到上位机任何数据就停机
    const uint32_t silent = CommandSilentCycles();
    if (IsActive() && silent >= kFailLimit)
    {
        Stop(EV_STOP_HEARTBEAT, static_cast<int32_t>(silent));
    }

    Power_Sample(IsActive()); // 供电监测（输入电压、单片机 3.3 V），跌落时报 221
    I2C2_RecoverIfStuck(); // 水压计之后总线若卡住，写推进器前先复位
    const bool tca_ok = TCA_SetChannel(4) == ExtensionResult::Ok;
    // HAL_Delay(5);
    vertical_PWM_allocation();
    horizontal_PWM_allocation();

    bool pca_ok = true;
    int32_t failed_channel = -1;
    for (int i = 0; i < PROPELLER_NUM; ++i)
    {
        // int32_t pwm = Parameter.InitPWM;
        int32_t pwm = current_PWM[i];
        pwm = clamp<int32_t>(pwm, 1000, 2000);
        int32_t duty_count = (pwm * 4096 + 10000) / 20000;
        if (PCA_Setpwm(i, 0, duty_count) != ExtensionResult::Ok)
        {
            pca_ok = false;
            failed_channel = i;
        }
    }
    TrackFailure(tca_ok, tca_fail_streak_, EV_WARN_TCA_SELECT, EV_STOP_TCA_SELECT, 4);
    TrackFailure(pca_ok, pca_fail_streak_, EV_WARN_PCA_WRITE, EV_STOP_PCA_WRITE, failed_channel);
}

void Propeller_I2C::TrackFailure(bool ok, uint32_t &streak, uint16_t warn_code, uint16_t stop_code, int32_t arg)
{
    if (ok)
    {
        streak = 0;
        return;
    }
    ++streak;
    if (streak == 1)
    {
        Event_PostLimited(warn_code, arg, 1000);
    }
    if (streak >= kFailLimit && IsActive())
    {
        Stop(stop_code, arg);
    }
}

bool Propeller_I2C::IsActive() const
{
    if (flag_float || flag_angle)
        return true;
    for (int i = 0; i < PROPELLER_NUM; ++i)
    {
        if (current_PWM[i] != Parameter.InitPWM)
            return true;
    }
    return false;
}

void Propeller_I2C::ResetVerticalControl()
{
    DepthPID.Reset();
    RollOutPID.Reset();
    RollInPID.Reset();
    PitchOutPID.Reset();
    PitchInPID.Reset();
    Target_roll_v = 0.0f;
    Target_pitch_v = 0.0f;
    PWM_component.Depth = PWM_component.Roll = PWM_component.Pitch = 0.0f;
}

void Propeller_I2C::ResetHorizontalControl()
{
    YawOutPID.Reset();
    YawInPID.Reset();
    Target_yaw_v = 0.0f;
    PWM_component.Yaw = 0.0f;
}

void Propeller_I2C::Stop(uint16_t code, int32_t arg)
{
    // OFF 总是记录；自动停机只在确实停了东西时记录（调用处已判断 IsActive）
    Event_Post(code, arg);
    stop_code_ = code;
    flag_float = false;
    flag_angle = false;
    motion_state = STOP;
    Target_depth = 30;
    for (int i = 0; i < PROPELLER_NUM; ++i)
    {
        current_PWM[i] = Parameter.InitPWM;
    }
    ResetVerticalControl();
    ResetHorizontalControl();
    nan_vertical_streak_ = nan_horizontal_streak_ = 0;
    pca_fail_streak_ = tca_fail_streak_ = 0;
}

int Propeller_I2C::FormatPlotFields(char *buf, int size)
{
    /*
     * 全部换算成 FRD（x前 y右 z下）方向，和状态行的姿态一致：抬头/右倾/顺时针为正。
     * 三个内环用的反馈 neg_roll_v / pitch_v / yaw_v 都恰好是 FRD 角速度的相反数，
     * 所以反馈和目标一起取负，画在同一张图上可以直接比较。
     */
    float roll = IMU::imu.attitude.roll;
    float pitch = -IMU::imu.attitude.pitch;
    float yaw = -IMU::imu.attitude.yaw;
    FusionState fusion = {};
    if (GetFusionState(&fusion) && isfinite(fusion.euler_rad[0]) && isfinite(fusion.euler_rad[1]) &&
        isfinite(fusion.euler_rad[2]))
    {
        roll = fusion.euler_rad[2];
        pitch = fusion.euler_rad[1];
        yaw = fusion.euler_rad[0];
    }
    auto scaled = [](float value, float scale) -> int {
        const float v = value * scale;
        return isfinite(v) ? static_cast<int>(v) : -999999;
    };
    constexpr float kCentiDegree = 18000.0f / PI; // rad -> 0.01 度
    const float depth = PressureSensor::pressure_sensor.data_depth;
    const float u_yaw = flag_angle ? PWM_component.Yaw : 0.0f;
    /*
     * 传感器原始读数（排查用）：
     * 四路水压计去零偏后的值（0 左前、1 左后、2 右后、3 右前），×10 mbar；
     * 加计（已减零偏）换到 FRD 机体系 = diag(1,-1,-1)×IMU，×100 m/s²。静止水平时约 (0, 0, -9.8)，抬头 X 变正、右倾 Y 变负。
     */
    const float *p = PressureSensor::pressure_sensor.data_pressure;
    const float *a = IMU::imu.pro_data.accel;
    return snprintf(buf, size, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    scaled(roll, kCentiDegree), scaled(pitch, kCentiDegree), scaled(yaw, kCentiDegree),
                    scaled(-Target_yaw, kCentiDegree),
                    scaled(-IMU::imu.attitude.neg_roll_v, kCentiDegree),
                    scaled(-IMU::imu.attitude.pitch_v, kCentiDegree),
                    scaled(-IMU::imu.attitude.yaw_v, kCentiDegree),
                    scaled(-Target_roll_v, kCentiDegree), scaled(-Target_pitch_v, kCentiDegree),
                    scaled(-Target_yaw_v, kCentiDegree),
                    scaled(depth, 10.0f), scaled(Target_depth, 10.0f),
                    scaled(PWM_component.Depth, 10.0f), scaled(PWM_component.Roll, 10.0f),
                    scaled(PWM_component.Pitch, 10.0f), scaled(u_yaw, 10.0f),
                    scaled(p[0], 10.0f), scaled(p[1], 10.0f), scaled(p[2], 10.0f), scaled(p[3], 10.0f),
                    scaled(a[0], 100.0f), scaled(-a[1], 100.0f), scaled(-a[2], 100.0f),
                    scaled(Power_Vin(), 100.0f), scaled(Power_Vdda(), 1000.0f));   // 输入电压 ×100 V，单片机 3.3 V ×1000
}

int Propeller_I2C::FormatPidFields(char *buf, int size)
{
    /*
     * 把控制量拆成各项的贡献（单位 us，和 $PL 里的深度/横滚/俯仰控制量同号、可以直接相加）：
     *   深度控制量 = P + I + D（DepthPID 本身的三项）
     *   横滚/俯仰控制量 = 内环kp × (外环P + 外环I + 外环D) + 内环kp × (-角速度反馈)
     * 最后一项就是内环的角速度阻尼。外环输出的三项已是各自限幅后的值；内环限幅（P 上限 100、总输出 200）只在打满时才让总和对不上。
     * 外环 50 Hz 更新，内环 150 Hz；这里取发送时刻的值。闭环没开时全为 0。
     */
    if (!flag_float)
        return snprintf(buf, size, "0,0,0,0,0,0,0,0,0,0,0");
    auto us10 = [](float v) -> int {
        const float x = v * 10.0f;
        return isfinite(x) ? static_cast<int>(x) : -999999;
    };
    const PID_Regulator_t &d = DepthPID.PIDInfo;
    const PID_Regulator_t &ro = RollOutPID.PIDInfo;
    const PID_Regulator_t &po = PitchOutPID.PIDInfo;
    const float kr = RollInPID.PIDInfo.kp;
    const float kp = PitchInPID.PIDInfo.kp;
    return snprintf(buf, size, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    us10(d.componentKp), us10(d.componentKi), us10(d.componentKd),
                    us10(kr * ro.componentKp), us10(kr * ro.componentKi), us10(kr * ro.componentKd),
                    us10(-kr * RollInPID.PIDInfo.fdb),
                    us10(kp * po.componentKp), us10(kp * po.componentKi), us10(kp * po.componentKd),
                    us10(-kp * PitchInPID.PIDInfo.fdb));
}

void Propeller_I2C::FatalNeutral()
{
    // 关中断状态下调用：总线卡住（BUSY）就不尝试，避免永远卡在 HAL 里
    if (hi2c2.Instance->SR2 & I2C_SR2_BUSY)
        return;
    hi2c2.Lock = HAL_UNLOCKED;
    hi2c2.State = HAL_I2C_STATE_READY;
    hi2c2.Mode = HAL_I2C_MODE_NONE;
    TCA_SetChannel(4);
    const int32_t duty_count = (Parameter.InitPWM * 4096 + 10000) / 20000;
    for (int i = 0; i < PROPELLER_NUM; ++i)
    {
        PCA_Setpwm(i, 0, duty_count);
    }
}

int Propeller_I2C::FormatStatusFields(char *buf, int size)
{
    // 姿态统一按 FRD（x前 y右 z下）显示：抬头 pitch 为正，右倾 roll 为正，顺时针 yaw 为正
    float roll = IMU::imu.attitude.roll;
    float pitch = -IMU::imu.attitude.pitch;
    float yaw = -IMU::imu.attitude.yaw;
    int source = 0;
    FusionState fusion = {};
    if (GetFusionState(&fusion) && isfinite(fusion.euler_rad[0]) && isfinite(fusion.euler_rad[1]) &&
        isfinite(fusion.euler_rad[2]))
    {
        roll = fusion.euler_rad[2];
        pitch = fusion.euler_rad[1];
        yaw = fusion.euler_rad[0];
        source = 1;
    }
    auto centi_degree = [this](float rad) -> int {
        const float value = rad2deg(rad) * 100.0f;
        return isfinite(value) ? static_cast<int>(value) : -99999;
    };
    const int mode = flag_float ? (flag_angle ? 2 : 1) : (IsActive() ? 3 : 0);
    const float depth = PressureSensor::pressure_sensor.data_depth;
    return snprintf(buf, size, "%d,%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%d",
                    mode, static_cast<unsigned>(stop_code_), static_cast<unsigned long>(CommandSilentCycles()),
                    static_cast<int>(Target_depth * 10.0f), isfinite(depth) ? static_cast<int>(depth * 10.0f) : -99999,
                    centi_degree(roll), centi_degree(pitch), centi_degree(yaw), source,
                    static_cast<int>(motion_state),
                    static_cast<int>(current_PWM[0]), static_cast<int>(current_PWM[1]),
                    static_cast<int>(current_PWM[2]), static_cast<int>(current_PWM[3]),
                    static_cast<int>(current_PWM[4]), static_cast<int>(current_PWM[5]),
                    static_cast<int>(current_PWM[6]), static_cast<int>(current_PWM[7]),
                    static_cast<unsigned>(PressureSensor::pressure_sensor.ValidMask()),
                    feedback_fusion_ ? 1 : 0); // 最后一项：闭环反馈来源 0=水压差 1=ESKF
}

void Propeller_I2C::float_ctrl()
{
        constexpr float pressure_to_depth_m_per_mbar = 100.0f / (1000.0f * 9.80665f);
        constexpr float sensor_x_m = kSensorHalfX_m; // 原 0.12 / 0.054，改为 CAD 实测，与 ESKF 一致
        constexpr float sensor_y_m = kSensorHalfY_m;

        FusionState fusion_state = {};
        const bool fusion_ready = GetFusionState(&fusion_state);

        float depth_feedback_mbar = PressureSensor::pressure_sensor.data_depth;
        float roll_feedback_mbar = PressureSensor::pressure_sensor.data_roll;
        float pitch_feedback_mbar = PressureSensor::pressure_sensor.data_pitch;

        // 默认（FB:PRS）与原版相同用四路水压差；FB:ESKF 时才用融合结果
        if (feedback_fusion_ && fusion_ready && isfinite(fusion_state.depth_m) && isfinite(fusion_state.euler_rad[1]) && isfinite(fusion_state.euler_rad[2])) {
            depth_feedback_mbar = fusion_state.depth_m / pressure_to_depth_m_per_mbar;
            roll_feedback_mbar = -4.0f * sensor_y_m * sinf(fusion_state.euler_rad[2]) / pressure_to_depth_m_per_mbar;
            pitch_feedback_mbar = -4.0f * sensor_x_m * sinf(fusion_state.euler_rad[1]) / pressure_to_depth_m_per_mbar;
        }

        // 积分暂停条件（见 kIntegralHoldAfter_ms）
        const uint32_t now_ms = HAL_GetTick();
        if (Target_depth != last_target_depth_)
        {
            last_target_depth_ = Target_depth;
            target_change_ms_ = now_ms;
        }
        if (motion_state != STOP && motion_state != FLOAT)
        {
            motion_active_ms_ = now_ms;
            motion_seen_ = true;
        }
        const bool motion_hold = motion_seen_ && now_ms - motion_active_ms_ < kIntegralHoldAfter_ms;
        const bool target_hold = now_ms - target_change_ms_ < kIntegralHoldAfter_ms;
        DepthPID.PIDInfo.integralHold = motion_hold || target_hold ||
                                        !(fabsf(Target_depth - depth_feedback_mbar) <= kDepthIntegralBand_mbar);
        PitchOutPID.PIDInfo.integralHold = motion_hold;

        PWM_component.Depth = DepthPID.PIDCalc(Target_depth, depth_feedback_mbar);

        // pid for roll and pitch
        // `ps_state` sets the frequency ratio between the outer and inner control loops to 3:1. //ps_state控制双环频率比为3:1
        // 外环始终 50 Hz（原来 ESKF 就绪后变成每周期 150 Hz，PID 的积分/微分等效增益随之改变）
        if (PressureSensor::pressure_sensor.ps_state == PS_HANDLE_STATE::CALCULATE)
        {
            // roll_diff = PressureSensor::pressure_sensor.data_roll;
            // roll_diff = IMU::imu.attitude.roll;
            // Target_roll_v = RollOutPID.PIDCalc(Target_roll, IMU::imu.attitude.roll);
            Target_roll_v = RollOutPID.PIDCalc(Target_roll, roll_feedback_mbar);
            // pitch_diff = PressureSensor::pressure_sensor.data_pitch;
            // pitch_diff = IMU::imu.attitude.pitch;
            // Target_pitch_v = PitchOutPID.PIDCalc(Target_pitch, IMU::imu.attitude.pitch);
            Target_pitch_v = PitchOutPID.PIDCalc(Target_pitch, pitch_feedback_mbar);
        }
        // PWM_component.Roll = RollInPID.PIDCalc(Target_roll_v, IMU::imu.attitude.roll_v); // IMU
        PWM_component.Roll = RollInPID.PIDCalc(Target_roll_v, IMU::imu.attitude.neg_roll_v); 
        PWM_component.Pitch = PitchInPID.PIDCalc(Target_pitch_v, IMU::imu.attitude.pitch_v);
}

void Propeller_I2C::speed_ctrl()
{
}

// Dual-loop PID control for yaw angle.
// 双环pid控制Yaw角
void Propeller_I2C::angle_ctrl()
{
    /*
     * 内环用的是 IMU 原始轴的 yaw_v（+gyro_z，z 朝上），
     * ESKF 机体系 z 朝下，ESKF 偏航角增加方向与之相反。
     * 原来外环直接用 ESKF 偏航，内外环符号相反，航向保持会越拉越偏。
     * 这里把 ESKF 偏航取负，换算回 IMU 的方向；目标值用同一来源设置。
     */
    float yaw_feedback = IMU::imu.attitude.yaw;
    if (yaw_from_fusion_)
    {
        FusionState fusion_state = {};
        if (GetFusionState(&fusion_state) && isfinite(fusion_state.euler_rad[0]))
            yaw_feedback = -fusion_state.euler_rad[0];
    }

    if (PressureSensor::pressure_sensor.ps_state == PS_HANDLE_STATE::CALCULATE)
    {
        yaw_diff = normalize_angle(yaw_feedback - Target_yaw);
        Target_yaw_v = YawOutPID.PIDCalc(0.0f, yaw_diff);
    }
    PWM_component.Yaw = YawInPID.PIDCalc(Target_yaw_v, IMU::imu.attitude.yaw_v);
}

float Propeller_I2C::deg2rad(float degree){
    return degree * PI / 180;
}

float Propeller_I2C::rad2deg(float rad){
    return rad * 180 / PI;
}

// PWM allocation for the internal (vertical-axis) thrusters.
// 垂直方向（内）推进器PWM分配
void Propeller_I2C::vertical_PWM_allocation()
{
    if(flag_float){
        float_ctrl();

        constexpr int8_t factors[4][3] = {
        {-1, -1, -1},  // Motor0
        {-1, -1,  1},  // Motor1
        {-1,  1, -1},  // Motor2
        {-1,  1,  1}   // Motor3
        };

        float depth = PWM_component.Depth;
        float pitch = PWM_component.Pitch;
        float roll = PWM_component.Roll;

        // NaN/Inf 保护：本周期沿用上一次的垂直输出，并清掉被污染的 PID
        if (!isfinite(depth) || !isfinite(pitch) || !isfinite(roll))
        {
            ResetVerticalControl();
            TrackFailure(false, nan_vertical_streak_, EV_WARN_NAN_VERTICAL, EV_STOP_NAN_VERTICAL, 0);
            return;
        }
        nan_vertical_streak_ = 0;

        for(int i=0; i<4; i++)
        {
            uint8_t idx = Parameter.InID[i];
            int8_t sign = Parameter.Sign[idx];
            int32_t base = Parameter.FloatPWM[i];
            int32_t pwm = base - sign * (depth * factors[i][0] + roll * factors[i][1] + pitch * factors[i][2]);
            if (pwm > Parameter.InitPWM) pwm += Parameter.Compensation[idx];
            else if(pwm < Parameter.InitPWM) pwm -= CompensationDown[idx]; // Compensate for the motor dead zone. // 补偿电机死区（两侧不对称）
            current_PWM[idx] = pwm;
        }
    }
    else{
        for(int i=0; i<PROPELLER_NUM; i++){
            //current_PWM[i] = Parameter.InitPWM;
        }
    }
}

// PWM allocation for the horizontal-axis thrusters.
// 水平方向推进器PWM分配
void Propeller_I2C::horizontal_PWM_allocation()
{
    float yaw;
	  if(flag_float){
			if(flag_angle){
        angle_ctrl();
        yaw = PWM_component.Yaw;
			}
			else{
					yaw = 0;
			}
			// NaN/Inf 保护：本周期沿用上一次的水平输出，并清掉偏航 PID
			if (!isfinite(yaw))
			{
				ResetHorizontalControl();
				TrackFailure(false, nan_horizontal_streak_, EV_WARN_NAN_HORIZONTAL, EV_STOP_NAN_HORIZONTAL, 0);
				return;
			}
			nan_horizontal_streak_ = 0;
			for (int i=0; i<4; i++)
			{
					uint8_t idx = Parameter.OutID[i];
					int32_t sign = Parameter.Sign[idx];
					int32_t base = state_PWM_map[motion_state][i];
					int32_t pwm = base + ((i<2) ? sign : - sign) * yaw;
					if (pwm > Parameter.InitPWM) pwm += Parameter.Compensation[idx];
					else if(pwm < Parameter.InitPWM) pwm -= CompensationDown[idx]; // Compensate for the motor dead zone. // 补偿电机死区（两侧不对称）
					current_PWM[idx] = pwm;
			}
		}
}


float Propeller_I2C::normalize_angle(float angle)
{
    angle = std::fmod(angle + PI, 2 * PI);
    if (angle < 0) angle += 2 * PI;
    return angle - PI;
}

void Propeller_I2C::command_set_angle_value(){
    if (flag_angle && flag_float)
    {
        char* data_str = (char *)RxBuffer + 4;
        Target_yaw = deg2rad(atoi(data_str));
    }
}

void Propeller_I2C::command_set_angle_closeloop(){
    if (flag_float)
    {
        char* data_str = (char *)RxBuffer + 4;
        if (strncmp(data_str, "ON", 2) == 0)
        {
            FusionState fusion_state = {};

            if (feedback_fusion_ && GetFusionState(&fusion_state) &&
                isfinite(fusion_state.euler_rad[0]))
            {
                // 换算到 IMU 方向（见 angle_ctrl），之后反馈也固定用 ESKF
                Target_yaw = -fusion_state.euler_rad[0];
                yaw_from_fusion_ = true;
            }
            else
            {
                /*
                 * ESKF 尚未初始化时，
                 * 回退到原来的 IMU yaw，之后反馈也固定用 IMU。
                 */
                Target_yaw =
                    IMU::imu.attitude.yaw;
                yaw_from_fusion_ = false;
            }
            Event_PostText(EV_INFO_YAW_LOCK, static_cast<int32_t>(rad2deg(Target_yaw) * 100.0f),
                           yaw_from_fusion_ ? "ESKF" : "IMU");
        }
        else if (strncmp(data_str, "OF", 2) == 0)
        {
            flag_angle = false;
            Event_Post(EV_INFO_ANGLE_OFF, 1);
        }
    }
}

void Propeller_I2C::command_test(){
    if (!flag_float && !flag_angle)
    {
        char* data_str = (char *)RxBuffer + 4;
        char *token = strtok(data_str, ",");
        int i = 0;
        while (token != NULL && i < 8)
        {
            current_PWM[i] = atoi(token);
            token = strtok(NULL, ",");
            i++;
        }
        stop_code_ = 0;
        Event_Post(EV_INFO_TES, i);
    }
}