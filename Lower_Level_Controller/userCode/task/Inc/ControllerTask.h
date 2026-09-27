//
// Created by lixinyi on 2026/9/26.
/*
 * 创建一个负责调用所有设备 Handle() 的 FreeRTOS 任务。用于替代handle.
 *
 * devices 是 Usermain.cpp 中的 device 数组。
 * count 是数组中有多少个设备。
 */
//

#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

#include <stdint.h>
#include "Usermain.h"
struct PressureSample;

void StartControllerTask(Device *devices[], uint32_t count);
//这个函数由 TIM1 定时器中断调用。
//它只负责通知任务，不负责执行设备处理。
void ControllerTickFromISR(void);
bool WaitPressureSample(PressureSample *sample, uint32_t timeout_ms);
#endif
