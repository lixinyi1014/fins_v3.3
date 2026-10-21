#include "Sensor.h"
#include "Kalman_filter.h"
#include "EventLog.h"
#include "ControllerTask.h"
#include "IMU.h"
#include <math.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c2;

PressureSensor PressureSensor::pressure_sensor;

KalmanFilter Pressure_Kf[4] = {KalmanFilter(0.01, 0.05, 1.0, 0.0),
                               KalmanFilter(0.01, 0.05, 1.0, 0.0),
                               KalmanFilter(0.01, 0.05, 1.0, 0.0),
                               KalmanFilter(0.01, 0.05, 1.0, 0.0)};

void PressureSensor::Init()
{
    Sensor_Site_t _site = {
        .x = {-12, 12, 12, -12},
        .y = {-5.4, -5.4, 5.4, 5.4},
        .z = {0, 0, 0, 0}
        /*.x={0},
        .y={1},
        .z={2},*/
    };
    site = _site;

    CurrentID = 0;

    // flag_Busy = 0;

    bsp_flash_read(&flashData);

    for (int i = 0; i < SENSOR_NUM; ++i)
    {
        TCA_SetChannel(i);
        HAL_Delay(5);
        Init_single(i);
    }

    data_plane[0] = 0;
    data_plane[1] = 0;
    data_plane[2] = 1;
    data_plane[3] = 0;
    data_plane[4] = 0;

    flag_calibrate = false;
    flag_read_pres = false;

    pressure_frame_id = 0;
    pressure_valid_mask_ = 0;
    LevelLoad(); // 水下偏置：Flash 里有 LVL 标定结果就用，否则用程序默认值
    /*quat[0]=1.0f;
    quat[1]=0.0f;
    quat[2]=0.0f;
    quat[3]=0.0f;
*/
}

void PressureSensor::Handle()
{
    // if(tim_id == 1){
    // flag_Busy = 1;

    if (flag_calibrate)
    {
        if (level_cal_active_)
            LevelCalFail(7, "CA");
        Controller_ExpectLongStep(); // 标定本来就要约 2.4 s，不当成"控制任务被卡住"报 218
        Calibrate(); // Calibration 校准
        flag_calibrate = false;
        // 标定时每一路的转换结果都已被读走，三段采样必须从头开始；
        // 否则若标定前停在"读压力"阶段，下一帧读到的是 0，四路都报 201
        ps_state = PS_HANDLE_STATE::GET_TEMPERATURE;
    }
    else
    {
        Handle_all();
        LevelCalStep();
    }
    /*
    Update_plane();//计算机器人参考系下水面方程
    data_depth=-data_plane[3];//计算深度
    pitch=atan(-data_plane[0]/(sqrt(data_plane[1]*data_plane[1]+data_plane[2]*data_plane[2])));//计算俯仰角
    roll=atan(data_plane[1]/data_plane[2]);//计算横滚角
     */
    data_depth = (data_pressure[3] + data_pressure[2] + data_pressure[1] + data_pressure[0]) / 4;
    data_roll = data_pressure[0] + data_pressure[1] - data_pressure[2] - data_pressure[3];
    data_pitch = data_pressure[0] + data_pressure[3] - data_pressure[1] - data_pressure[2];
}

void PressureSensor::Receive()
{
    if (strncmp((char *)RxBuffer, "LVL:DEF", 7) == 0) LevelResetDefault();
    else if (strncmp((char *)RxBuffer, "LVL", 3) == 0) LevelCalStart();
    else if (strncmp((char *)RxBuffer, "CA", 2) == 0) flag_calibrate = true;
    else if (strncmp((char *)RxBuffer, "VA", 2) == 0) flag_read_pres = true;
}
bool PressureSensor::CopyLatestSample(
    PressureSample *sample) const
{
    if (sample == nullptr)
    {
        return false;
    }

    if (pressure_frame_id == 0)
    {
        return false;
    }

    const float millibar_to_pa = 100.0f;
    const float water_density = 1000.0f;
    const float gravity = 9.80665f;

    const float pressure_to_depth =
        millibar_to_pa / (water_density * gravity);

    /*
     * 帧号和有效掩码都来自最近完成的 CALCULATE 阶段。
     *
     * 这里直接复制本帧检查结果，
     * 不再根据初始化时的 flag_ok 重新生成掩码。
     */
    sample->frame_id = pressure_frame_id;
    sample->valid_mask = pressure_valid_mask_;

    for (int i = 0; i < SENSOR_NUM; ++i)
    {
        const uint8_t current_bit =
            static_cast<uint8_t>(1U << i);

        /*
         * 只有本帧通过设备层检查的通道，
         * 才向快照中复制对应读数。
         */
        if ((sample->valid_mask & current_bit) != 0)
        {
            sample->pressure_diff_mbar[i] =
                data_pressure[i];

            sample->pressure_pa[i] =
                data_pressure_raw[i] * millibar_to_pa;

            sample->depth_m[i] =
                data_pressure[i] * pressure_to_depth;
        }
        else
        {
            /*
             * 无效通道的快照数值清零。
             *
             * 是否能使用这一通道，仍然由掩码决定。
             * 这里的零不能被当作一次有效的水面测量。
             */
            sample->pressure_diff_mbar[i] = 0.0f;
            sample->pressure_pa[i] = 0.0f;
            sample->depth_m[i] = 0.0f;
        }
    }

    /*
     * 保留已有快照字段的含义。
     * 新的深度计算由 SensorTask 根据有效通道完成。
     */
    sample->mean_depth_m =
        data_depth * pressure_to_depth;

    sample->legacy_roll_error_mbar =
        data_roll;

    sample->legacy_pitch_error_mbar =
        data_pitch;

    /*
     * true 表示成功复制了一份已完成的采样帧。
     * 即使 valid_mask 为 0，也可以成功复制，
     * 由任务层判断这一帧没有可用通道。
     */
    return true;
}
void PressureSensor::Delay_us(uint32_t us)
{
    uint32_t i;
    for (i = 0; i < us; i++)
    {
        int a = 10; // delay based on main clock, 168Mhz
        while (a--);
    }
}

unsigned char PressureSensor::MS5837_30BA_Crc4(int id)
{
    int cnt;
    int t;
    unsigned int n_rem = 0;
    unsigned char n_bit;
    unsigned char a = 0;
    unsigned char b = 0;
    unsigned short int n_prom[8];

    for (t = 0; t < 7; t++)
    {
        n_prom[t] = Cal_C[id][t];
    }
    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;
    for (cnt = 0; cnt < 16; cnt++)
    {
        if (cnt % 2 == 1)
            n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else
            n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F);
    a = (n_rem ^ 0x00);
    b = Cal_C[id][0] >> 12;
    if (a == b)
    {
        return 1;
    }
    else
        return 0;
}

void PressureSensor::MS5837_30BA_ReSet(void)
{
    uint8_t data[1];
    data[0] = MS5837_30BA_ResetCommand;
    HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, data, 1, EXT_I2C_TIMEOUT_MS);
}

uint8_t PressureSensor::MS5837_30BA_PROM(int id)
{
    uint8_t memaddr[1];
    uint8_t data[2];
    int i;

    MS5837_30BA_ReSet(); // ��λMS5837
    HAL_Delay(20);
    for (i = 0; i < 7; i++)
    {
        memaddr[0] = MS5837_30BA_PROM_RD + (i * 2);
        HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, memaddr, 1, EXT_I2C_TIMEOUT_MS);
        HAL_I2C_Master_Receive(&hi2c2, B02_IIC_ADDRESS, data, 2, EXT_I2C_TIMEOUT_MS);
        Cal_C[id][i] = (((uint16_t)data[0] << 8) | data[1]);
    }
    return !Cal_C[id][0];
}

unsigned long PressureSensor::MS5837_30BA_GetConversion(uint8_t command)
{
    unsigned long conversion = 0;
    uint8_t data = MS5837_30BA_ADC_RD;
    uint8_t temp[3];
    HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, &command, 1, EXT_I2C_TIMEOUT_MS);
    HAL_Delay(2);
    // Delay_us(15000);
    HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, &data, 1, EXT_I2C_TIMEOUT_MS);

    HAL_I2C_Master_Receive(&hi2c2, B02_IIC_ADDRESS, temp, 3, EXT_I2C_TIMEOUT_MS);
    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return conversion;
}

/*signed int PressureSensor::MS5837_30BA_GetTemp(int id)
{
    if(flag_ok[id]) {
        D2_Temp = MS5837_30BA_GetConversion(MS5837_30BA_D2_OSR_8192);
        HAL_Delay(20);
        dT = D2_Temp - (((uint32_t) Cal_C[id][5]) * 256l);
        return dT;
    }
    else return -1;
}*/

float PressureSensor::MS5837_30BA_GetData(int id)
{

    if (flag_ok[id])
    {
        D2_Temp = MS5837_30BA_GetConversion(MS5837_30BA_D2_OSR1024);
        // HAL_Delay(1);
        // Delay_us(30000);
        D1_Pres = MS5837_30BA_GetConversion(MS5837_30BA_D1_OSR1024);
        // HAL_Delay(3);
        // Delay_us(30000);
        dT = D2_Temp - (((uint32_t)Cal_C[id][5]) * 256l);
        SENS = (int64_t)Cal_C[id][1] * 65536l + ((int64_t)Cal_C[id][3] * dT) / 128l;
        OFF_ = (int64_t)Cal_C[id][2] * 131072l + ((int64_t)Cal_C[id][4] * dT) / 64l;

        TEMP = 2000l + (int64_t)(dT)*Cal_C[id][6] / 8388608LL;

        // �����¶Ȳ���
        if (TEMP < 2000) // low temp
        {

            Ti = (11 * (int64_t)(dT) * (int64_t)(dT) / (34359738368LL));
            OFFi = (31 * (TEMP - 2000) * (TEMP - 2000)) / 8;
            SENSi = (63 * (TEMP - 2000) * (TEMP - 2000)) / 32;
        }
        else
        { // high temp
            Ti = 2 * (dT * dT) / (137438953472LL);
            OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
            SENSi = 0;
        }
        OFF2 = OFF_ - OFFi;
        SENS2 = SENS - SENSi;
        /*if(D1_Pres==0||D2_Temp==0) return last_measure[id];
        else{
            float pressure = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
            last_measure[id]=pressure;
            return pressure;

        }*/
        float pressure = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
        return pressure;

        // data_pressure = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;          //У׼��ѹ������

        // Temperature = (TEMP - Ti) / 100.0;                                //У׼���¶�����
    }
    else
        return -1;
}

void PressureSensor::Init_single(int id)
{
    data_pressure_offset[id] = 0;
    if (!flag_ok[id])
    {
        MS5837_30BA_PROM(id);
        flag_ok[id] = MS5837_30BA_Crc4(id);
    }
		// ------ TODO: Update the bias values here after receiving the calibration data.
    //------TODO:收到校准数据后修改此处零偏值
    /*
        for(int i=0;i<4;++i){
            data_pressure_offset[i] = flashData.pressure_offset[i]/1000.0;
        }
        */
    switch (Robot_Version)
    {
    case V30:
        data_pressure_offset[0] = PRESSURE_0_V30;
        data_pressure_offset[1] = PRESSURE_1_V30;
        data_pressure_offset[2] = PRESSURE_2_V30;
        data_pressure_offset[3] = PRESSURE_3_V30;
        break;
    case V31:
        data_pressure_offset[0] = PRESSURE_0_V31;
        data_pressure_offset[1] = PRESSURE_1_V31;
        data_pressure_offset[2] = PRESSURE_2_V31;
        data_pressure_offset[3] = PRESSURE_3_V31;
        break;
    case V32:
        data_pressure_offset[0] = PRESSURE_0_V32;
        data_pressure_offset[1] = PRESSURE_1_V32;
        data_pressure_offset[2] = PRESSURE_2_V32;
        data_pressure_offset[3] = PRESSURE_3_V32;
		break;
    case V33:
        data_pressure_offset[0] = PRESSURE_0_V33;
        data_pressure_offset[1] = PRESSURE_1_V33;
        data_pressure_offset[2] = PRESSURE_2_V33;
        data_pressure_offset[3] = PRESSURE_3_V33;
		break;
    case V40:
        data_pressure_offset[0] = PRESSURE_0_V40;
        data_pressure_offset[1] = PRESSURE_1_V40;
        data_pressure_offset[2] = PRESSURE_2_V40;
        data_pressure_offset[3] = PRESSURE_3_V40;
		break;
    }
}

void PressureSensor::Calibrate()
{
    Event_Post(EV_INFO_CALIB_START, 0);
    for (int i = 0; i < SENSOR_NUM; ++i)
    {
        TCA_SetChannel(i);
        // HAL_Delay(5);
        Calibrate_single(i);
    }
    Event_Post(EV_INFO_CALIB_DONE, 0);
    // bsp_flash_write(&flashData);
}

void PressureSensor::Calibrate_single(int id)
{
    uint8_t TxBuffer[SENSOR_NUM * 9] = {0};
    data_pressure_offset[id] = 0;
    for (int j = 0; j < 100; ++j)
    {
        data_pressure_raw[id] = MS5837_30BA_GetData(id);
        data_pressure_offset[id] += data_pressure_raw[id] / 100;
    }
    int data_temp = (int)(data_pressure_offset[id] * 1000);
    // flashData.pressure_offset[id] = data_temp;
    int data_digit[7];
    for (int j = 0; j < 7; ++j)
    {
        data_digit[j] = data_temp % 10;
        data_temp /= 10;
    }
    TxBuffer[id * 9] = '0' + data_digit[6];
    TxBuffer[id * 9 + 1] = '0' + data_digit[5];
    TxBuffer[id * 9 + 2] = '0' + data_digit[4];
    TxBuffer[id * 9 + 3] = '0' + data_digit[3];
    TxBuffer[id * 9 + 4] = '.';
    TxBuffer[id * 9 + 5] = '0' + data_digit[2];
    TxBuffer[id * 9 + 6] = '0' + data_digit[1];
    TxBuffer[id * 9 + 7] = '0' + data_digit[0];
    if (id == 3)
        TxBuffer[id * 9 + 8] = '\n';
    else
        TxBuffer[id * 9 + 8] = ',';
    // 原来用阻塞 HAL_UART_Transmit 输出，会和中断发送冲突；改为事件上报
    static const char *const channel_names[SENSOR_NUM] = {"CH0", "CH1", "CH2", "CH3"};
    Event_PostText(EV_INFO_CALIB_RESULT, static_cast<int32_t>(data_pressure_offset[id] * 1000.0f), channel_names[id]);
    (void)TxBuffer;
}

void PressureSensor::OutputData()
{
    for (int i = 0; i < SENSOR_NUM; ++i)
    {
        OutputData_single(i);
    }
}

void PressureSensor::OutputData_single(int id)
{
    uint8_t TxBuffer[10] = {0};
    bool IsPositive = true;
    int data_temp = (int)(data_pressure[id] * 1000);
    if (data_pressure[id] < 0)
    {
        IsPositive = false;
        data_temp = -data_temp;
    }
    int data_digit[7];
    for (int j = 0; j < 7; ++j)
    {
        data_digit[j] = data_temp % 10;
        data_temp /= 10;
    }
    if (IsPositive)
        TxBuffer[0] = ' ';
    else
        TxBuffer[0] = '-';
    TxBuffer[1] = '0' + data_digit[6];
    TxBuffer[2] = '0' + data_digit[5];
    TxBuffer[3] = '0' + data_digit[4];
    TxBuffer[4] = '0' + data_digit[3];
    TxBuffer[5] = '.';
    TxBuffer[6] = '0' + data_digit[2];
    TxBuffer[7] = '0' + data_digit[1];
    TxBuffer[8] = '0' + data_digit[0];
    if (id == 3)
        TxBuffer[9] = '\n';
    else
        TxBuffer[9] = ',';
    HAL_UART_Transmit(&huart6, TxBuffer, sizeof(TxBuffer), 0x00ff);
}

void PressureSensor::Handle_single(int id)
{

    data_pressure_raw[id] = MS5837_30BA_GetData(id);
    float temp = data_pressure_raw[id] - data_pressure_offset[id];
    temp = Pressure_Kf[id].update(temp); // First-order low-pass filtering. // 一阶低通滤波
    data_pressure[id] = temp;

    // if (temp < 500 && temp > -10)
    // {
    //     data_pressure[id] = temp;
    // }
    // if(data_pressure[id]>200||data_pressure[id]<-10) data_pressure[id]=last_measure[id]- data_pressure_offset[id];
}

// Following the approach of `Handle_single`, overall efficiency is improved by overlapping the wait time after the first I2C request.
// 仿照Handle_single的思路，通过重叠I2C第一次请求后的等待时间实现整体效率提高
void PressureSensor::Handle_all()
{
    static uint8_t data = MS5837_30BA_ADC_RD;
    static uint8_t command_tmp = MS5837_30BA_D2_OSR1024;
    static uint8_t command_pres = MS5837_30BA_D1_OSR1024;
    static uint8_t temp[3];
    static float tmp_pres;
    static unsigned long conversion[8]; // The upper four bits of `conversion` store temperature data, and the lower four bits store pressure information. //conversion的前四位存储温度数据，后四位存储水压信息
    /*
     * 本帧每一路是否所有 I2C 步骤都成功。
     * 任何一步失败（选通、发命令、读数），这一路本帧作废，
     * data_pressure 沿用上一次成功的值；conversion 也只在读成功时更新。
     */
    static bool channel_ok[SENSOR_NUM] = {};
    static uint32_t fail_streak[SENSOR_NUM] = {};
    /*
     * ps_state 保存压力传感器当前所处的采样阶段。
     *
     * 每次调用 Handle_all() 只执行一个阶段：
     *
     * 1. GET_TEMPERATURE：
     *    启动四个传感器的温度转换。
     *
     * 2. GET_PRESSURE：
     *    读取上一阶段的温度结果，
     *    然后启动四个传感器的压力转换。
     *
     * 3. CALCULATE：
     *    读取压力转换结果，
     *    计算并滤波四个压力值。
     *
     * 三个阶段完成后，才算得到一帧完整的压力数据。
     */
    I2C2_RecoverIfStuck(); // 上一周期总线若被干扰卡住，先复位再采样
    switch (ps_state) {
        case PS_HANDLE_STATE::GET_TEMPERATURE:
            // Notify the sensor to prepare temperature data.
            // 告知传感器准备温度数据
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                channel_ok[i] = TCA_SetChannel(i) == ExtensionResult::Ok &&
                    HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, &command_tmp, 1, 1) == HAL_OK;
            }
            ps_state = PS_HANDLE_STATE::GET_PRESSURE;
            break;

        case PS_HANDLE_STATE::GET_PRESSURE:
            // Acquire temperature information.
            // 收集温度信息
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                const bool ok = TCA_SetChannel(i) == ExtensionResult::Ok &&
                    HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, &data, 1, 1) == HAL_OK &&
                    HAL_I2C_Master_Receive(&hi2c2, B02_IIC_ADDRESS, temp, 3, 1) == HAL_OK;
                const unsigned long value = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
                if (ok && value != 0)
                    conversion[i] = value;  // 0 表示转换未完成，不算成功
                else
                    channel_ok[i] = false;
            }

            // Notify the sensor to prepare pressure data.
            // 告知传感器准备压强数据
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                if (!(TCA_SetChannel(i) == ExtensionResult::Ok &&
                      HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, &command_pres, 1, 1) == HAL_OK))
                    channel_ok[i] = false;
            }
            ps_state = PS_HANDLE_STATE::CALCULATE;
            break;

        case PS_HANDLE_STATE::CALCULATE:
            // Retrieve the pressure readings.
            // 获取压强数值
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                const bool ok = TCA_SetChannel(i) == ExtensionResult::Ok &&
                    HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, &data, 1, 1) == HAL_OK &&
                    HAL_I2C_Master_Receive(&hi2c2, B02_IIC_ADDRESS, temp, 3, 1) == HAL_OK;
                const unsigned long value = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
                if (ok && value != 0)
                    conversion[i+4] = value;
                else
                    channel_ok[i] = false;
            }

            // Refer to `float PressureSensor::MS5837_30BA_GetData(int id)` to compute the four raw pressure values.
            // 参考float PressureSensor::MS5837_30BA_GetData(int id)计算四个原始压强
            float pressure[SENSOR_NUM] = {};
            uint8_t frame_valid_mask = 0;
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                if (flag_ok[i] && channel_ok[i])
                {
                    D2_Temp = conversion[i];
                    D1_Pres = conversion[i+4];
                    dT = D2_Temp - (((uint32_t)Cal_C[i][5]) * 256l);
                    SENS = (int64_t)Cal_C[i][1] * 65536l + ((int64_t)Cal_C[i][3] * dT) / 128l;
                    OFF_ = (int64_t)Cal_C[i][2] * 131072l + ((int64_t)Cal_C[i][4] * dT) / 64l;

                    TEMP = 2000l + (int64_t)(dT)*Cal_C[i][6] / 8388608LL;
                    if (TEMP < 2000) // low temp
                    {

                        Ti = (11 * (int64_t)(dT) * (int64_t)(dT) / (34359738368LL));
                        OFFi = (31 * (TEMP - 2000) * (TEMP - 2000)) / 8;
                        SENSi = (63 * (TEMP - 2000) * (TEMP - 2000)) / 32;
                    }
                    else
                    { // high temp
                        Ti = 2 * (dT * dT) / (137438953472LL);
                        OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
                        SENSi = 0;
                    }
                    OFF2 = OFF_ - OFFi;
                    SENS2 = SENS - SENSi;
                    pressure[i] = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
                    const float pressure_diff_mbar = pressure[i] - data_pressure_offset[i];
                    // 合理范围：MS5837-30BA 量程 0~30 bar，水面约 1000 mbar；超出视为读错
                    if (isfinite(pressure[i]) && isfinite(pressure_diff_mbar) &&
                        pressure[i] > 300.0f && pressure[i] < 31000.0f)
                    {
                        frame_valid_mask |=
                            static_cast<uint8_t>(1U << i);
                    }
                }
                // else
                // return -1;
            }

            /*
             * 只有有效压力计才更新当前压力值。
             *
             * 无效通道保持上一帧结果（沿用上一次成功值），
             * 这样不会破坏原有闭环控制接口。
             */
            // 水下偏置权重：按上一帧平均深度，2 mbar 以下 0（空气中没有水柱差，不该扣），5 mbar 以上 1
            // （ESKF 从 5 cm 水深开始用压差）。四路用同一个权重、偏置之和为 0，所以深度不变
            {
                const float w = (data_depth - 2.0f) / 3.0f;
                level_weight_ = isfinite(w) ? (w < 0.0f ? 0.0f : (w > 1.0f ? 1.0f : w)) : 0.0f;
            }
            for (int i = 0; i < SENSOR_NUM; ++i)
            {
                const uint8_t current_bit =
                    static_cast<uint8_t>(1U << i);

                /*
                 * 初始化无效，或者本次读取不合理时，
                 * 保留上一帧旧控制使用的数据。
                 */
                if (!flag_ok[i] ||
                    (frame_valid_mask & current_bit) == 0)
                {
                    // 每路连续失败的第一次上报警告（传感器不参与停机）
                    if (flag_ok[i] && ++fail_streak[i] == 1)
                        Event_PostLimited(static_cast<uint16_t>(EV_WARN_PRESSURE_READ), i, 1000);
                    continue;
                }
                fail_streak[i] = 0;

                data_pressure_raw[i] =
                    pressure[i];

                tmp_pres =
                    data_pressure_raw[i] -
                    data_pressure_offset[i];

                tmp_pres =
                    Pressure_Kf[i].update(tmp_pres);

                pressure_no_level_[i] = tmp_pres;
                data_pressure[i] =
                    tmp_pres - level_weight_ * level_offset_mbar[i];
            }
            pressure_valid_mask_ = frame_valid_mask;
            ++pressure_frame_id;
            ps_state = PS_HANDLE_STATE::GET_TEMPERATURE;
            break;
    }
}

void PressureSensor::Solve_plane_3(float *data, float *h, float *x, float *y, float *z)
{
    float pos_data[9];
    for (int i = 0; i < 3; ++i)
    {
        pos_data[i * 3] = x[i];
        pos_data[i * 3 + 1] = y[i];
        pos_data[i * 3 + 2] = z[i];
    }

    double a = 0, b = 0, c = 0, d = 0;
    Matrix pos(3, 3, pos_data);

    Matrix pos_inv;
    if (pos_inv.inv(pos))
    {

        float a1 = 0, a2 = 0, b1 = 0, b2 = 0, c1 = 0, c2 = 0, ax = 0, bx = 0, cx = 0;

        for (int i = 1; i < 4; ++i)
        {
            a1 -= pos_inv.mat[1][i];
            b1 -= pos_inv.mat[2][i];
            c1 -= pos_inv.mat[3][i];
            a2 -= pos_inv.mat[1][i] * h[i - 1];
            b2 -= pos_inv.mat[2][i] * h[i - 1];
            c2 -= pos_inv.mat[3][i] * h[i - 1];
        }

        ax = a1 * a1 + b1 * b1 + c1 * c1;
        bx = 2 * (a1 * a2 + b1 * b2 + c1 * c2);
        cx = a2 * a2 + b2 * b2 + c2 * c2 - 1;
        data[0] = (-bx - sqrt(bx * bx - 4 * ax * cx)) / (2 * ax);
        data[1] = a1 * d + a2;
        data[2] = b1 * d + b2;
        data[3] = c1 * d + c2;
    }
}

void PressureSensor::Solve_plane(float *data, float *h, float *x, float *y, float *z, int num)
{
    float xi = 0, yi = 0, zi = 0, hi = 0, xi2 = 0, yi2 = 0, zi2 = 0, xiyi = 0, xizi = 0, yizi = 0, xihi = 0, yihi = 0, zihi = 0;
    for (int i = 0; i < num; ++i)
    {
        xi += x[i];
        yi += y[i];
        zi += z[i];
        hi += h[i];
        xi2 += x[i] * x[i];
        yi2 += y[i] * y[i];
        zi2 += z[i] * z[i];
        xiyi += x[i] * y[i];
        xizi += x[i] * z[i];
        yizi += y[i] * z[i];
        xihi += x[i] * h[i];
        yihi += y[i] * h[i];
        zihi += z[i] * h[i];
    }
    float gradient[5] = {0};
    gradient[0] = xi2 * data[0] + xiyi * data[1] + xizi * data[2] + xi * data[3] + xihi + data[0] * data[4];
    gradient[1] = xiyi * data[0] + yi2 * data[1] + yizi * data[2] + yi * data[3] + yihi + data[1] * data[4];
    gradient[2] = xizi * data[0] + yizi * data[1] + zi2 * data[2] + zi * data[3] + zihi + data[2] * data[4];
    gradient[3] = xi * data[0] + yi * data[1] + zi * data[2] + num * data[3] + hi;
    gradient[4] = data[0] * data[0] + data[1] * data[1] + data[2] * data[2] - 1;

    float jacobi_data[25] = {xi2 + data[4], xiyi, xizi, xi, data[0],
                             xiyi, yi2 + data[4], yizi, yi, data[1],
                             xizi, yizi, zi2 + data[4], zi, data[2],
                             xi, yi, zi, (float)num, 0,
                             data[0], data[1], data[2], 0, 0};
    Matrix jacobi(5, 5, jacobi_data);
    Matrix jacobi_inv;
    if (jacobi_inv.inv(jacobi))
    {
        float data_dx[5] = {0};
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 5; ++j)
            {
                data_dx[i] += jacobi_inv.mat[i + 1][j + 1] * gradient[j];
            }
            data[i] -= 0.5 * data_dx[i];
        }
    }
}

void PressureSensor::Update_plane()
{
    Solve_plane(data_plane, data_pressure, site.x, site.y, site.z, SENSOR_NUM);

    for (int i = 0; i < 3; ++i)
    {
        data_level[i] = data_plane[i];
    }
}
/*
void PressureSensor::get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}
*/

void Sonar::Init()
{
    data_offset = 0;
    for (int i = 0; i < 100; ++i)
    {
        Handle();
        HAL_Delay(20);
        if (RxBuffer[0] == 0xFF)
            data_offset += (RxBuffer[1] * 256 + RxBuffer[2]) / 100;
    }
}

void Sonar::Handle()
{
    // if(tim_id == 1){
    uint8_t rqst[1] = {' '};
    HAL_UART_Transmit_IT(&huart6, rqst, 1);
    //}
}

void Sonar::Receive()
{
    if (RxBuffer[0] == 0xFF)
    {
        data = RxBuffer[1] * 256 + RxBuffer[2] - data_offset;
    }
    else
        data = -1;

    // HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}
/* ======================================================================
 * 水下偏置 / LVL 水下水平标定（2026-10-20）
 *
 * 2026-10-20 沉底 14 分钟的数据：以加速度计姿态为准，四路水压计各自多读了
 * -0.14 / -0.27 / +0.03 / +0.38 mbar（约等于安装高度差 1.4 / 2.8 / 0.3 / 3.9 mm），
 * 18 mbar 悬停和 90 mbar 沉底时基本一样大，是固定偏差而不是灵敏度差。
 * 它让水压差算出的横滚多 1.8°、俯仰多 -0.57°，ESKF 也被拉偏约 25%。
 * ====================================================================== */
namespace
{
    // 程序默认值（上面那次数据 900~1270 s 的平均），之和为 0
    const float kLevelDefaultMbar[SENSOR_NUM] = {-0.142f, -0.270f, 0.028f, 0.384f};
    constexpr uint32_t kLevelMagic = 0x4C564C31u;  // "LVL1"
    constexpr float kLevelMaxAbsMbar = 2.0f;       // 超过 2 mbar（约 2 cm 高度差）肯定不对
    constexpr uint32_t kLevelCalMs = 5000;         // 标定时长
    constexpr float kLevelMinDepthMbar = 10.0f;    // 至少这么深才标
    constexpr float kLevelMaxGyroRad = 0.05f;      // 约 3°/s，超过算车在动
    // 水压计相对舱体位置，与 ESKF / Propeller 相同：0 左前 1 左后 2 右后 3 右前，FRD
    constexpr float kLevelHalfX_m = 0.1247f;
    constexpr float kLevelHalfY_m = 0.0691f;
    const float kLevelX[SENSOR_NUM] = {kLevelHalfX_m, -kLevelHalfX_m, -kLevelHalfX_m, kLevelHalfX_m};
    const float kLevelY[SENSOR_NUM] = {-kLevelHalfY_m, -kLevelHalfY_m, kLevelHalfY_m, kLevelHalfY_m};
    constexpr float kMbarPerMeter = 1000.0f * 9.80665f / 100.0f; // 98.07 mbar/m（与 pressure_to_depth 一致）
    const char *const kChannelText[SENSOR_NUM] = {"CH0", "CH1", "CH2", "CH3"};

    uint32_t LevelCheckWord(const flash_data_t &d)
    {
        uint32_t c = d.level_magic ^ 0xA5A5A5A5u;
        for (int i = 0; i < SENSOR_NUM; ++i)
        {
            uint32_t w;
            memcpy(&w, &d.level_offset_mbar[i], sizeof(w));
            c = ((c << 5) | (c >> 27)) ^ w;
        }
        return c;
    }

    // 四路减去平均值，保证之和为 0（深度不受影响）
    void CenterOffsets(float v[SENSOR_NUM])
    {
        float mean = 0.0f;
        for (int i = 0; i < SENSOR_NUM; ++i) mean += v[i];
        mean /= SENSOR_NUM;
        for (int i = 0; i < SENSOR_NUM; ++i) v[i] -= mean;
    }
}

void PressureSensor::LevelLoad()
{
    // flashData 已在 Init 开头 bsp_flash_read
    bool from_flash = flashData.level_magic == kLevelMagic && flashData.level_check == LevelCheckWord(flashData);
    for (int i = 0; from_flash && i < SENSOR_NUM; ++i)
    {
        const float v = flashData.level_offset_mbar[i];
        if (!isfinite(v) || fabsf(v) > kLevelMaxAbsMbar)
            from_flash = false;
    }
    for (int i = 0; i < SENSOR_NUM; ++i)
        level_offset_mbar[i] = from_flash ? flashData.level_offset_mbar[i] : kLevelDefaultMbar[i];
    CenterOffsets(level_offset_mbar);
    for (int i = 0; i < SENSOR_NUM; ++i)
        Event_PostText(EV_INFO_LEVEL_RESULT, static_cast<int32_t>(lroundf(level_offset_mbar[i] * 1000.0f)), kChannelText[i]);
    Event_PostText(EV_INFO_LEVEL_DONE, from_flash ? 1 : 0, "LOAD");
}

bool PressureSensor::LevelSave(bool from_calibration)
{
    if (LevelBlocked())
        return false;
    flashData.level_magic = from_calibration ? kLevelMagic : 0u; // 恢复默认时让 Flash 里的结果失效
    for (int i = 0; i < SENSOR_NUM; ++i)
        flashData.level_offset_mbar[i] = level_offset_mbar[i];
    flashData.level_check = LevelCheckWord(flashData);
    Controller_ExpectLongStep(); // 擦扇区约 1~2 s，不当成控制任务卡住报 218
    return bsp_flash_write(&flashData);
}

void PressureSensor::LevelCalFail(int32_t reason, const char *text)
{
    level_cal_active_ = false;
    Event_PostText(EV_WARN_LEVEL_FAIL, reason, text);
}

void PressureSensor::LevelCalStart()
{
    if (LevelBlocked()) { Event_PostText(EV_WARN_LEVEL_FAIL, 1, "ACTIVE"); return; }
    if (flag_calibrate) { Event_PostText(EV_WARN_LEVEL_FAIL, 7, "CA"); return; }
    if (!(data_depth >= kLevelMinDepthMbar)) { Event_PostText(EV_WARN_LEVEL_FAIL, 2, "SHALLOW"); return; }
    if ((pressure_valid_mask_ & 0x0Fu) != 0x0Fu) { Event_PostText(EV_WARN_LEVEL_FAIL, 3, "INVALID"); return; }

    for (int i = 0; i < SENSOR_NUM; ++i) level_sum_p_[i] = 0.0;
    for (int i = 0; i < 3; ++i) level_sum_down_[i] = 0.0;
    level_cal_frames_ = 0;
    level_cal_bad_frames_ = 0;
    level_cal_last_frame_ = pressure_frame_id;
    level_cal_start_ms_ = HAL_GetTick();
    level_cal_active_ = true;
    Event_Post(EV_INFO_LEVEL_START, static_cast<int32_t>(kLevelCalMs));
}

void PressureSensor::LevelResetDefault()
{
    if (LevelBlocked()) { Event_PostText(EV_WARN_LEVEL_FAIL, 1, "ACTIVE"); return; }
    level_cal_active_ = false;
    for (int i = 0; i < SENSOR_NUM; ++i)
        level_offset_mbar[i] = kLevelDefaultMbar[i];
    CenterOffsets(level_offset_mbar);
    for (int i = 0; i < SENSOR_NUM; ++i)
        Event_PostText(EV_INFO_LEVEL_RESULT, static_cast<int32_t>(lroundf(level_offset_mbar[i] * 1000.0f)), kChannelText[i]);
    if (!LevelSave(false))
        Event_PostText(EV_WARN_LEVEL_FAIL, 6, "FLASH");
    Event_PostText(EV_INFO_LEVEL_DONE, 0, "DEF");
}

void PressureSensor::LevelCalStep()
{
    if (!level_cal_active_ || pressure_frame_id == level_cal_last_frame_)
        return;
    level_cal_last_frame_ = pressure_frame_id;

    if (LevelBlocked()) { LevelCalFail(1, "ACTIVE"); return; }
    if (!(data_depth >= kLevelMinDepthMbar)) { LevelCalFail(2, "SHALLOW"); return; }

    ImuSample imu = {};
    if ((pressure_valid_mask_ & 0x0Fu) != 0x0Fu || !IMU::imu.CopyLatestSample(&imu))
    {
        ++level_cal_bad_frames_;
    }
    else
    {
        const float gyro = sqrtf(imu.gyro_[0] * imu.gyro_[0] + imu.gyro_[1] * imu.gyro_[1] + imu.gyro_[2] * imu.gyro_[2]);
        const float a_norm = sqrtf(imu.accel_[0] * imu.accel_[0] + imu.accel_[1] * imu.accel_[1] + imu.accel_[2] * imu.accel_[2]);
        if (!isfinite(gyro) || gyro > kLevelMaxGyroRad || !isfinite(a_norm) || fabsf(a_norm - 9.8f) > 0.5f)
        {
            LevelCalFail(4, "MOVING");
            return;
        }
        // IMU 原始轴 -> FRD"向下"方向，与 SensorTask::ConvertImuToBody 相同：down = (-a0, a1, a2)
        level_sum_down_[0] += -imu.accel_[0];
        level_sum_down_[1] += imu.accel_[1];
        level_sum_down_[2] += imu.accel_[2];
        for (int i = 0; i < SENSOR_NUM; ++i)
            level_sum_p_[i] += pressure_no_level_[i];
        ++level_cal_frames_;
    }

    if (HAL_GetTick() - level_cal_start_ms_ < kLevelCalMs)
        return;

    level_cal_active_ = false;
    if (level_cal_frames_ < 50 || level_cal_bad_frames_ * 5 > level_cal_frames_)
    {
        Event_PostText(EV_WARN_LEVEL_FAIL, 3, "INVALID");
        return;
    }

    const double n = static_cast<double>(level_cal_frames_);
    const float dx = static_cast<float>(level_sum_down_[0] / n);
    const float dy = static_cast<float>(level_sum_down_[1] / n);
    const float dz = static_cast<float>(level_sum_down_[2] / n);
    const float g = sqrtf(dx * dx + dy * dy + dz * dz);
    const float pitch = -asinf(dx / g);
    const float roll = atan2f(dy, dz);

    float p[SENSOR_NUM];
    float mean = 0.0f;
    for (int i = 0; i < SENSOR_NUM; ++i)
    {
        p[i] = static_cast<float>(level_sum_p_[i] / n);
        mean += p[i];
    }
    mean /= SENSOR_NUM;

    // 偏置 = 实测各路相对平均值的偏差 - 按加速度计姿态应有的偏差（与 ESKF CorrectPressure 同一公式）
    float offset[SENSOR_NUM];
    for (int i = 0; i < SENSOR_NUM; ++i)
    {
        const float expected = (-kLevelX[i] * sinf(pitch) + kLevelY[i] * cosf(pitch) * sinf(roll)) * kMbarPerMeter;
        offset[i] = (p[i] - mean) - expected;
        if (!isfinite(offset[i]) || fabsf(offset[i]) > kLevelMaxAbsMbar)
        {
            Event_PostText(EV_WARN_LEVEL_FAIL, 5, "RANGE");
            return;
        }
    }
    CenterOffsets(offset);
    for (int i = 0; i < SENSOR_NUM; ++i)
    {
        level_offset_mbar[i] = offset[i];
        Event_PostText(EV_INFO_LEVEL_RESULT, static_cast<int32_t>(lroundf(offset[i] * 1000.0f)), kChannelText[i]);
    }
    if (!LevelSave(true))
    {
        Event_PostText(EV_WARN_LEVEL_FAIL, 6, "FLASH"); // 本次运行仍然生效，只是断电后丢失
        return;
    }
    Event_PostText(EV_INFO_LEVEL_DONE, 1, "SAVE");
}
