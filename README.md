可以先看一下readme
整体是根据公布出来的架构改的
## 下位机部分
使用了FreeRTOS改写，pid调参的时候看propeller.cpp

深度pid的kd不起作用，理由是水压计 20 ms 才出一个新深度，而控制每 6.7 ms 算一次，调大之后抖的很厉害。
所以写了一个新逻辑（但明显这么写很丑先将就一下），在propeller30行左右，调整depth的kdepthkd来进行调参
   constexpr float kDepthKd = 2.0f;            // us / (mbar/s)
   constexpr float kDepthRateTau_s = 0.15f;    // 平滑时间常数 s
   constexpr float kDepthDMax = 50.0f;         // D 项限幅 us
具体这个kd的逻辑在UpdateDepthRate()里，可以看一下

以及加了前馈
struct MotionFeedforward { float depth; float pitch; float roll; };
constexpr MotionFeedforward kFfFront = {-28.0f,  25.0f,  0.0f};  // 前进
constexpr MotionFeedforward kFfBack  = {-30.0f, -20.0f,  0.0f};  // 后退
constexpr MotionFeedforward kFfLeft  = {-25.0f,  10.0f,  0.0f};  // 左移
constexpr MotionFeedforward kFfRight = {-27.0f,   0.0f, -2.0f};  // 右移
constexpr float kMotionFfRate_us_s = 75.0f; // 斜坡速度：每秒最多变化 75 us

其他控制器逻辑没什么不一样，水压计我还没改成3，2路也可以接受的（因为还没碰到过四个以下无法接收的情况）

## 上位机部分
打开host_console文件夹，里面有readme
运行后会跳出来两个网页，一个读数据一个连机子。
一般上位机连机子需要勾选一下eskf以及空气中水压计零偏校准再下水
