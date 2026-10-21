//
// Created by admin on 2023/11/28.
//
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#ifndef CONTROL_FRAME_MAIN_LEGACY_H
#define CONTROL_FRAME_MAIN_LEGACY_H

#define FLASH_SECTOR_9_ADDRESS 0x080A0000

typedef struct {
    /*uint32_t robot_ID;
    uint32_t yaw_zero;
    uint32_t pitch_zero;
    uint32_t pat[125];*/
    uint32_t pressure_offset[4];
    /* 2026-10-20：水压计水下偏置（LVL 标定结果）。magic 不对或校验不对时用程序默认值 */
    uint32_t level_magic;
    float level_offset_mbar[4];
    uint32_t level_check;
}flash_data_t;

typedef struct {
    float x,y,z;
    float temp;
}ist8310_t;

extern flash_data_t flashData;

extern void bsp_ADC_vccMoni();
/*
 * 供电监测（2026-10-13）：每个控制周期调用一次。
 * 测 C 板输入电压（PF10/ADC3_IN8，板上 200k/22k 分压，C 板由电池 XT30 供电时就是电池电压）
 * 和单片机自身 3.3 V（用内部基准 VREFINT 反算）。thrusters_active=false 时更新"空闲电压"基准。
 * 跌落时上报 221。
 */
extern void Power_Sample(bool thrusters_active);
extern float Power_Vin(void);   // V，未接/读不到时约 0
extern float Power_Vdda(void);  // V
extern void usart_printf(const char *fmt,...);
extern bool bsp_flash_write(flash_data_t *_flashData); // true=写入并读回一致
extern void bsp_flash_read(flash_data_t *_flashData);


#endif //CONTROL_FRAME_MAIN_LEGACY_H
