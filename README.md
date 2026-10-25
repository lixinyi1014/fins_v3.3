可以先看一下readme
整体是根据公布出来的架构改的
## 下位机部分
使用了FreeRTOS改写，pid调参的时候看propeller.cpp

深度pid的kd不起作用，理由是水压计 20 ms 才出一个新深度，而控制每 6.7 ms 算一次，调大之后抖的很厉害。
所以写了一个新逻辑（但明显这么写很丑先将就一下），在propeller30行左右，调整depth的kdepthkd来进行调参
> constexpr float kDepthKd = 2.0f;            // us / (mbar/s)
> 
> constexpr float kDepthRateTau_s = 0.15f;    // 平滑时间常数 s
> 
> constexpr float kDepthDMax = 50.0f;         // D 项限幅 us
具体这个kd的逻辑在UpdateDepthRate()里，可以看一下

以及加了前馈
> struct MotionFeedforward { float depth; float pitch; float roll; };
> 
> constexpr MotionFeedforward kFfFront = {-28.0f,  25.0f,  0.0f};  // 前进
> 
> constexpr MotionFeedforward kFfBack  = {-30.0f, -20.0f,  0.0f};  // 后退
> 
> constexpr MotionFeedforward kFfLeft  = {-25.0f,  10.0f,  0.0f};  // 左移
> 
> constexpr MotionFeedforward kFfRight = {-27.0f,   0.0f, -2.0f};  // 右移
> 
> constexpr float kMotionFfRate_us_s = 75.0f; // 斜坡速度：每秒最多变化 75 us

其他控制器逻辑没什么不一样，水压计我还没改成3，2路也可以接受并进行一些计算（因为还没碰到过四个以下无法接收的情况）


## 上位机部分
打开host_console文件夹，里面有readme
运行后会跳出来两个网页，一个读数据一个连机子。
一般上位机连机子需要勾选一下eskf以及空气中水压计零偏校准再下水


## 论文实现部分
给到codex检查，目前这些问题还在，我预期下周解决
| PDF 中的工作 | 当前代码情况 | 判断 |
|---|---|---|
| 四元数、陀螺仪零偏、六维误差状态、协方差预测及校正 | 有对应实现，并接入传感器任务 | **基础部分已实现** |
| 四、三、二个水压计可用时，重构观测并降级融合 | 只接受四路全部有效 | **未实现** |
| 水压差专用观测模型及相关噪声协方差传播 | 将压差转成“重力方向”，复用加速度计更新 | **与 PDF 不一致** |
| 水压计相邻采样突变门控、失效后的深度处理 | 有通信和数值有效性检查，但缺少论文规定的突变判断及姿态补偿深度估计 | **部分实现** |
| 磁力计观测与异步序贯融合 | 有融合流程，但磁力计使用简化航向观测，时间轴也未完全按理论落实 | **部分实现** |
| 六自由度位置单环、姿态双环 PID | 有定深和姿态闭环，水平前后、横移仍是开环指令 | **未完成六自由度闭环** |

主要的问题在345这里，对于3，我直接使用了固定的方差；对于4，突变门控确实可以再加一下，然后深度处理没有做好，应该先把各传感器的深度换算到机器人参考点，再组合。对于5，首先yaw还没测过，磁力计还没加到这个eskf的状态估计里面去。同时注意下读磁力计我的测量时间与处理时间没有严格区分。新 IMU 帧不一定意味着其中磁力计也是新数据。
