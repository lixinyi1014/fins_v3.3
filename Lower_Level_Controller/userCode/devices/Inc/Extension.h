//
// Created by admin on 2023/10/29.
//

#ifndef CONTROL_FRAME_MAIN_EXTENSION_H
#define CONTROL_FRAME_MAIN_EXTENSION_H

#include "Usermain.h"



#define TCA9548A_ADDR         0x70

#define TCA9548A_CHANNEL_0          0x01
#define TCA9548A_CHANNEL_1          0x02
#define TCA9548A_CHANNEL_2          0x04
#define TCA9548A_CHANNEL_3          0x08
#define TCA9548A_CHANNEL_4          0x10
#define TCA9548A_CHANNEL_5          0x20
#define TCA9548A_CHANNEL_6          0x40
#define TCA9548A_CHANNEL_7          0x80

#define PCA9685_ADDR 0x80

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
/*
 * I2C2 单次传输超时（ms）。400 kHz 下最长的一次传输（5 字节）约 0.15 ms。
 * 原来是 10000 / 0xffff：推进器一转、总线受干扰出现仲裁丢失或总线错误时，
 * HAL 会在一个标志上死等 10 s～65 s，控制任务整个卡住（遥测、心跳检查、停机全停，推进器保持最后的 PWM）。
 */
#define EXT_I2C_TIMEOUT_MS 3U

enum class ExtensionResult :uint8_t {
    Ok,
    InvalidChannel,
    InvalidValue,
    I2cError,
    Timeout
};
/*

extern uint8_t RxBuffer_servo[];


extern void I2C_Extension_init();
extern void I2C_Extension();*/

extern ExtensionResult TCA_SetChannel(uint8_t channel);
extern ExtensionResult PCA_Setpwm(uint8_t num, uint32_t on, uint32_t off);
extern ExtensionResult PCA_Write(uint8_t adrr,uint8_t data);
extern ExtensionResult PCA_Setfreq(float freq);
extern ExtensionResult PCA_Read(uint8_t adrr, uint8_t *value);
/* I2C2 卡死（BUSY 一直为 1、或有总线错误/仲裁丢失）时复位总线；在两次传输之间调用。返回 true 表示做了复位。 */
extern bool I2C2_RecoverIfStuck(void);




#endif //CONTROL_FRAME_MAIN_EXTENSION_H
