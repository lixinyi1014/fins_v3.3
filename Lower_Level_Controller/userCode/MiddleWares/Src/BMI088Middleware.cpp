#include "BMI088Middleware.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    // 原来是 int a = 10; while (a--); —— 编译器开优化后整段删掉，等于完全不延时，
    // BMI088 写寄存器后的等待全部失效。加 volatile 保留循环：168 MHz 下内层约 6 周期/次，30 次约 1 us（只会偏长）。
    for (uint16_t i = 0; i < us; i++)
    {
        volatile int a = 30;
        while (a--)
        {
        }
    }
}




void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

