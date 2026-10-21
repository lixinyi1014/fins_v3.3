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
struct ImuSample;
void StartControllerTask(Device *devices[], uint32_t count, const char *const *names = nullptr); // names：与 devices 一一对应的 ASCII 名称，可省略
//这个函数由 TIM1 定时器中断调用。
//它只负责通知任务，不负责执行设备处理。
void ControllerTickFromISR(void);
// USART6 不在接收状态时补开（实现在 Usermain.cpp，控制任务每周期调用）
void Uart6_EnsureReceiving(void);
// 模块在 Handle() 里要做已知的长操作（水压标定约 2.4 s）前调用，本周期不报 218"控制任务被卡住"
void Controller_ExpectLongStep(void);
bool WaitPressureSample(PressureSample *sample, uint32_t timeout_ms);
bool WaitImuSample(ImuSample *sample, uint32_t timeout_ms);
/*
 * 串口命令接收：中断里只把收到的数据放进缓冲（不解析），
 * 控制任务每个周期开头统一取出、按行拆分、交给各设备 Receive()。
 * 这样命令解析与 Handle() 在同一个任务里顺序执行，不会互相打断。
 */
bool CommandQueue_PushFromISR(const uint8_t *data, uint16_t length);
/*
 * 距离上一次收到上位机任何数据已经过去多少个控制周期（150 Hz）。
 * 推进器模块用它做心跳检查。
 */
uint32_t CommandSilentCycles(void);
#endif
