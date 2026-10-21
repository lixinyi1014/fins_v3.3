//
// Created by LEGION on 2021/10/4.
//
//This file contains the main function and the interrupt service routines (ISRs).
//The usage of each module follows three steps: initialization, data acquisition, and execution.
//Initialization and execution are performed within the timer interrupt routine.
//Serial (UART) data reception is handled within the UART interrupt routine.
//
//此文件为主函数和中断服务函数所在地
//每个元件的使用包含三个步骤，初始化、获取数据、执行
//初始化和执行在定时器中断函数进行
//获取串口数据在串口中断函数进行
#include "Usermain.h"
#include "IMU.h"
#include "Propeller.h"
#include "Servo.h"
#include "Sensor.h"
#include "Legacy.h"
#include "Watchdog.h"
#include "LED.h"
#include "Buzzer.h"
#include "gpio.h"
#include "UART_Base.h"
#include "ControllerTask.h"
#include "SensorTask.h"
#include "EventLog.h"
#include "FreeRTOS.h"
#include "task.h"


//Adjust according to the vehicle version: V30, V31, V32, V33, V40.
//根据潜器版本调整，V30，V31，V32，V33，V40
VERSION_E Robot_Version = V33;


// Statically instantiated objects.
// 静态实例化对象
// IMU imu;
static Servo servo;
static Servo_I2C servo_i2c;
static Propeller_I2C propeller_i2c;
static Sonar sonar;
static Watchdog watchdog;
static LED led;
static Buzzer buzzer;


// Array of device pointers for unified management and invocation of device operations.
// ------ TODO: Select the required devices.
// 设备指针数组，用于统一管理和调用设备操作
//------TODO:选用需要的设备
static Device *device[] = {
        &IMU::imu,                         //IMU 
        &PressureSensor::pressure_sensor,  //Pressure Sensor 水压计
        &propeller_i2c,                    //Propeller（PWM control via the PWM expansion board） 推进器（PWM扩展板控制）
        //&propeller,                      //Propeller（PWM control via the Robomaster_C board） 推进器（C板PWM控制）
				&servo_i2c,                        //Servo（PWM control via the PWM expansion board） 舵机（PWM扩展板控制）
				//&servo,                          //Servo（PWM control via the Robomaster_C board） 舵机（C板PWM控制）
        //&watchdog,                       //Watchdog 窗口看门狗
        &led,                              //LED
        //&buzzer                          //Buzzer 蜂鸣器
};

// Macro definition for the number of devices.
// 定义设备数量宏
#define DEVICE_NUM (sizeof(device) / sizeof(device[0]))
// 与上面 device[] 一一对应的名称（只用 ASCII，事件 218 上报"哪个模块卡住"用；改 device[] 时同步改）
static const char *const device_names[] = {"IMU", "PRES", "PROP", "SERVO", "LED"};
static_assert(sizeof(device_names) / sizeof(device_names[0]) == DEVICE_NUM, "device_names 要和 device[] 一一对应");

volatile bool system_init_flag = false;
volatile bool tick_300hz = false;
uint8_t RxBuffer[SERIAL_LENGTH_MAX] = {0};      // 各设备 Receive() 解析用（只在控制任务里写）
static uint8_t uart_rx_buffer[SERIAL_LENGTH_MAX] = {0}; // USART6 中断接收专用缓冲
volatile uint8_t key_raw_state = 1;
uint32_t key_last_stamp;
volatile int32_t time_start = 0, time_end = 0, time_interval = 0, cnt = 0;

// Timer interrupt service routine (ISR) for periodic device processing.
// 定时器中断服务函数, 用于周期性处理设备
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(!system_init_flag) return;//初始化完成前不通知
    //Tim1到期一次通知一次controllertask
    if(htim == &htim1){ //300Hz
        ControllerTickFromISR();
    }
}

// UART receive interrupt service routine (ISR) for handling device-specific UART reception tasks.
// 串口接收中断服务函数, 用于处理设备的串口接收任务
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t datasize)//HAL_UART_RxCpltCallback
{
    if(huart->Instance==USART6) {
        /*
         * 中断里不再解析命令（原来在这里直接调用各设备 Receive()，
         * 其中 ACL:ON 会在中断里进 FreeRTOS 临界区导致整机卡死）。
         * 现在只把数据放进缓冲，由控制任务在下一个周期开头统一解析。
         */
        if (datasize > 0 && !CommandQueue_PushFromISR(uart_rx_buffer, datasize))
        {
            // 缓冲满时丢掉的若只是心跳 "HB"，不用报：缓冲里已有数据，本周期照样算收到心跳（水压标定阻塞约 2 s 时必然发生）
            const bool heartbeat_only = datasize <= 4 && uart_rx_buffer[0] == 'H' && uart_rx_buffer[1] == 'B';
            if (!heartbeat_only)
                Event_PostLimited(EV_WARN_CMD_OVERFLOW, datasize, 1000);
        }
        // Re-enable UART reception.
        // 重新开启接收
        HAL_UARTEx_ReceiveToIdle_IT(&huart6, uart_rx_buffer, SERIAL_LENGTH_MAX);
    }
}

/*
 * 控制任务每周期调用：USART6 若不在接收状态（中断里重开失败等），在这里补开并上报 217。
 */
void Uart6_EnsureReceiving(void)
{
    if (huart6.RxState != HAL_UART_STATE_READY)
        return;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    HAL_StatusTypeDef status = HAL_BUSY;
    const bool idle = huart6.RxState == HAL_UART_STATE_READY;
    if (idle)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart6);
        status = HAL_UARTEx_ReceiveToIdle_IT(&huart6, uart_rx_buffer, SERIAL_LENGTH_MAX);
    }
    __set_PRIMASK(primask);
    if (idle)
        Event_PostLimited(EV_WARN_UART_RX_RESTART, static_cast<int32_t>(status), 1000);
}

/*
 * 串口错误回调：溢出(ORE)时 HAL 会中止接收，这里重新开启，
 * 否则一次溢出之后就再也收不到命令。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        Event_PostLimited(EV_WARN_UART_RX_ERROR, static_cast<int32_t>(huart->ErrorCode), 1000);
        if (huart->RxState == HAL_UART_STATE_READY)
        {
            HAL_UARTEx_ReceiveToIdle_IT(&huart6, uart_rx_buffer, SERIAL_LENGTH_MAX);
        }
    }
}

// UART transmit interrupt callback function.
// 串口发送中断回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    #define HANDLE_UART_TX(ID)                            \
        do {                                              \
            if (huart == uartHandleList[ID]) {            \
                UARTBaseLite<ID>::GetInstance().OnTxDone(); \
                return;                                   \
            }                                             \
        } while (0)

    if (huart == &huart6) {
        Telemetry_OnTxDone(); // USART6 由事件/遥测模块统一发送
        return;
    }
    HANDLE_UART_TX(1);

    #undef HANDLE_UART_TX
    // if(huart == &huart1) {
    //     UARTBaseLite<1>::GetInstance().OnTxDone();
    // }
    // else if(huart == &huart6){
    //     UARTBaseLite<6>::GetInstance().OnTxDone();
    // }
}

// Key/button interrupt (currently unused).
// 按键中断，暂时无用
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//     if(system_init_flag == 0)return;
//     if (GPIO_Pin == GPIO_PIN_0){
//         key_raw_state = HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);
//         uint32_t time_interval = HAL_GetTick() - key_last_stamp;
//         key_last_stamp = HAL_GetTick();
//         /*if (time_interval >= 50){
//             if (key_raw_state == 1){
//                 period = 5000;
//             }else{
//                 period = 500;
//             }
//         }*/
//     }
//     IMU::imu.ITHandle(GPIO_Pin);
// }

/**
  * @brief  The application entry point.
  * @retval int
  */
// Entry point of the main function.
// 主函数入口
int main(void) {
    /*
     * 复位原因（RCC->CSR 高 8 位），随上电事件一起上报，
     * 方便区分正常上电、复位键、欠压复位（推进器大电流拉低电压时常见）等。
     */
    const uint32_t reset_flags = RCC->CSR >> 24;
    RCC->CSR |= RCC_CSR_RMVF;
    Event_Post(EV_INFO_BOOT, static_cast<int32_t>(reset_flags));

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM5_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM10_Init();
    MX_TIM8_Init();
    MX_ADC1_Init();
    MX_ADC3_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_USART3_UART_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    //MX_IWDG_Init();
    MX_USB_DEVICE_Init();

    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);


    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);
    //TODO adc校准？
    //RemoteControl::init();
    bsp_flash_read(&flashData);
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

    // Initialize all devices.
    // 初始化所有设备
    for(int i = 0; i < DEVICE_NUM; ++i){
        device[i] -> Init();
    }
    // LVL 水下水平标定：推进器在转时不标定、不写 Flash
    PressureSensor::pressure_sensor.SetLevelGuard([]() -> bool { return propeller_i2c.IsActive(); });
    /*
     * 先创建设备任务。
     *
     * 这个函数会创建：
     * 1. ControllerTask
     * 2. imu_queue
     * 3. pressure_queue
     */
    StartControllerTask(device, DEVICE_NUM, device_names);

    /*
     * 再创建负责读取传感器队列的 SensorTask。
     */
    StartSensorTask();

    /*
     * 所有设备初始化完成后再开启串口接收（原来分散在推进器、舵机的 Init 里）。
     */
    // 初始化期间上位机的心跳已经在发，串口早就溢出(ORE)了；先清掉，否则一开接收就报一次 207
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, uart_rx_buffer, SERIAL_LENGTH_MAX);

    // Set the initialization-complete flag.
    // 设置初始化完成标志,允许定时器中断开始通知任务
    system_init_flag = true;
    time_start = HAL_GetTick();
    // 启动FreeRTOS调度器，正常情况下这个函数不会返回
    vTaskStartScheduler();
    // 正常情况下不会执行到这里
    Fins_Fatal(EV_FATAL_SCHEDULER_EXIT, 0, nullptr);
}
void send_float(float value, uint8_t decimalPlaces, bool endSign){
    bool isNegative = 0;
    uint8_t len = 0;
    static char buffer_float[20] = {0};
		
		// Handle negative values.
    // 处理负数
    if(value < 0){
        isNegative = 1;
        value = -value;
    }
    if(isNegative) {
        buffer_float[0] = '-';
        len = 1;
    }

		// Convert the integer part.
    // 转换整数部分
    int32_t intPart = (int)value;
    len += sprintf(buffer_float+len, "%d", intPart);
		
		// Process the fractional part.
    // 处理小数部分
    if (decimalPlaces > 0) {
        buffer_float[len++] = '.';

        float fraction = value - (float)intPart;
        for (uint8_t i = 0; i < decimalPlaces; i++) {
            fraction *= 10;
            int digit = (int)fraction;
            buffer_float[len++] = digit + '0';
            fraction -= digit;
        }
    }

		// If this is the final element, append a newline; otherwise, append a comma.
    // 若结束则换行，否则是逗号
    if(endSign)
        buffer_float[len++] = '\n'; // Null terminator of the string 字符串结束符
    else
        buffer_float[len++] = ',';

    buffer_float[len] = '\0';

    // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, len, 0x00ff);
    Telemetry_Write(buffer_float, len);
}

void send_int(int32_t value, bool endSign){
    static char buffer_int[20] = {0};
    uint8_t len = 0;
    bool isNegative = 0;

		// Handle negative values.
    // 处理负数
    if(value < 0){
        isNegative = 1;
        value = -value;
    }
    if(isNegative) {
        buffer_int[0] = '-';
        len = 1;
    }

    len += sprintf(buffer_int+len, "%d", value);

    if(endSign)
        buffer_int[len++] = '\n';
    else
        buffer_int[len++] = ',';

    buffer_int[len] = '\0';
    // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, len, 0x00ff);
    Telemetry_Write(buffer_int, len);
}