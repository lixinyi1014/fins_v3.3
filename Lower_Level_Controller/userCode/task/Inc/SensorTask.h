//
// Created by lixin on 2026/9/28.
//

#ifndef RM_FRAME_C_SENSORTASK_H
#define RM_FRAME_C_SENSORTASK_H
struct FusionState;
/*
 * 创建负责接收、整理传感器数据的 FreeRTOS 任务。
 *
 * 必须先调用 StartControllerTask() 创建输入队列，
 * 再调用本函数，最后启动 FreeRTOS 调度器。
 *
 * 本函数只在系统启动时调用一次。
 */
void StartSensorTask();
/*
 * 复制 SensorTask 当前保存的最新融合状态。
 *
 * 返回 true 表示 ESKF 已经完成初始化，
 * 返回 false 表示目前还没有可用的完整融合状态。
 *
 * state 必须指向调用者准备好的 FusionState 对象。
 */
bool GetFusionState(FusionState *state);
#endif //RM_FRAME_C_SENSORTASK_H
