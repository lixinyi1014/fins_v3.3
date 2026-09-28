//
// Created by lixin on 2026/9/28.
//

#ifndef RM_FRAME_C_SENSORTASK_H
#define RM_FRAME_C_SENSORTASK_H
/*
 * 创建负责接收、整理传感器数据的 FreeRTOS 任务。
 *
 * 必须先调用 StartControllerTask() 创建输入队列，
 * 再调用本函数，最后启动 FreeRTOS 调度器。
 *
 * 本函数只在系统启动时调用一次。
 */
void StartSensorTask();
#endif //RM_FRAME_C_SENSORTASK_H
