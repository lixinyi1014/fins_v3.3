"""FinsROV 下位机事件编号表。

必须与下位机 userCode/devices/Inc/EventLog.h 完全一致（test_protocol.py 会自动核对）。
编号规则：
    1xx 停机   推进器全部回中位并锁存，需要重新 ON / TES
    2xx 警告   不停机，只记录
    3xx 信息   上电、解锁、模式切换
    9xx 致命   单片机已停在死循环，必须复位 / 重新上电
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

SENSOR_NAMES = {0: "0号 左前", 1: "1号 左后", 2: "2号 右后", 3: "3号 右前"}
MOTION_NAMES = {0: "停止", 1: "悬停", 2: "前进", 3: "后退", 4: "左移", 5: "右移",
                6: "顺时针", 7: "逆时针"}
MODE_NAMES = {0: "待机", 1: "深度闭环", 2: "深度+偏航闭环", 3: "手动输出(TES)"}


def _reset_cause(arg: int) -> str:
    parts = []
    if arg & 0x08:
        parts.append("上电/掉电（单片机 3.3 V 断过电；没动电源开关却出现就是供电被拉垮）")
    elif arg & 0x02:
        parts.append("欠压复位（电压被拉低，常见于推进器大电流）")
    if arg & 0x04 and not arg & 0x08:
        parts.append("复位键/烧录器复位")
    if arg & 0x10:
        parts.append("软件复位")
    if arg & 0x20:
        parts.append("独立看门狗复位")
    if arg & 0x40:
        parts.append("窗口看门狗复位")
    if arg & 0x80:
        parts.append("低功耗复位")
    return "复位原因：" + ("、".join(parts) if parts else f"未知(0x{arg:02X})")


def _uart_error(arg: int) -> str:
    bits = []
    if arg & 0x01:
        bits.append("校验错误")
    if arg & 0x02:
        bits.append("噪声")
    if arg & 0x04:
        bits.append("帧错误(波特率/格式不符)")
    if arg & 0x08:
        bits.append("溢出(数据来不及收)")
    if arg & 0x10:
        bits.append("DMA")
    return "、".join(bits) if bits else f"ErrorCode=0x{arg:X}"


@dataclass(frozen=True)
class CodeInfo:
    code: int
    name: str
    detail: Callable[[int, str], str] | None = None

    @property
    def level(self) -> str:
        return {1: "stop", 2: "warn", 3: "info", 9: "fatal"}.get(self.code // 100, "warn")


_MODULE_NAMES = {"IMU": "IMU（陀螺/加计/磁力计）", "PRES": "水压计", "PROP": "推进器", "SERVO": "舵机", "LED": "LED"}


def _i2c_status(arg: int) -> str:
    arg &= 0xFFFFFFFF
    count, sr1, sr2 = arg >> 20, (arg >> 8) & 0xFFF, arg & 0xFF
    if count == 0:          # 旧固件格式：SR1 | SR2 << 16
        count, sr1, sr2 = 1, arg & 0xFFFF, (arg >> 16) & 0xFFFF
    bits = []
    if sr2 & 0x02:
        bits.append("总线一直忙(BUSY)")
    if sr2 & 0x01:
        bits.append("C 板还处在主机发送中、停止位没能发出去（线上有东西拉住 SDA/SCL）")
    if sr1 & 0x0100:
        bits.append("总线错误(BERR)")
    if sr1 & 0x0200:
        bits.append("仲裁丢失(ARLO)")
    return (f"近 1 秒复位 {count} 次；{'、'.join(bits) or '状态异常'}（SR1=0x{sr1:03X} SR2=0x{sr2:02X}），"
            "多半是推进器电流干扰")


def _bmi088_error(arg: int, text: str) -> str:
    parts = []
    if arg & 0x80:
        parts.append("加计自检未过")
    if arg & 0x40:
        parts.append("陀螺自检未过")
    if arg & 0x3F:
        parts.append(f"寄存器配置失败码 0x{arg & 0x3F:02X}")
    detail = f"错误位 0x{arg & 0xFF:02X}（{'、'.join(parts) or '无'}）"
    if text:
        detail += f"；加计实际量程 ±{text}，已按此换算" if text != "?" else "；读不到加计量程，加速度可能不可信"
    return detail


_LEVEL_FAIL = {
    1: "推进器在转，先 OFF 再标定",
    2: "水不够深，至少 10 mbar（约 10 cm）",
    3: "水压计读数无效（有通道读不到）",
    4: "车在晃动，等静止后再标",
    5: "结果超过 ±2 mbar，不合理，可能标定时车在动或水压计有问题",
    6: "Flash 写入失败：本次偏置仍然生效，但断电后丢失",
    7: "正在做水压零点标定（CA），已取消",
}


def _info(code, name, detail=None):
    return CodeInfo(code, name, detail)


CODES: dict[int, CodeInfo] = {c.code: c for c in [
    # ---- 1xx 停机 ----
    _info(101, "收到 OFF，已停机"),
    _info(102, "心跳丢失停机",
          lambda a, t: f"连续 {a} 个控制周期（约 {a / 150:.2f} s）没收到上位机任何数据"),
    _info(103, "垂直控制量连续 100 周期为 NaN，已停机"),
    _info(104, "偏航控制量连续 100 周期为 NaN，已停机"),
    _info(105, "推进器 PCA9685 连续 100 周期写入失败，已停机",
          lambda a, t: f"最后失败通道 {a}。I2C 不通时中位也写不进去，推进器可能仍在转，请断电"),
    _info(106, "I2C 扩展器 TCA9548A 连续 100 周期选通失败，已停机",
          lambda a, t: "推进器板所在的 4 号口选不通，推进器可能仍保持旧 PWM，请断电"),
    # ---- 2xx 警告 ----
    _info(201, "水压计读取失败，沿用上一次的值",
          lambda a, t: f"{SENSOR_NAMES.get(a, a)}（1 秒内其余通道的同类警告不重复显示）"),
    _info(202, "磁力计读取失败，沿用上一次的值"),
    _info(203, "推进器 PCA9685 写入失败（连续失败的第一次）", lambda a, t: f"通道 {a}"),
    _info(204, "TCA9548A 选通失败（连续失败的第一次）"),
    _info(205, "垂直控制量出现 NaN，本周期沿用上次输出并复位 PID"),
    _info(206, "偏航控制量出现 NaN，本周期沿用上次输出并复位 PID"),
    _info(207, "串口接收错误，已自动重新开启接收", lambda a, t: _uart_error(a)),
    _info(208, "命令缓冲满，丢弃一包", lambda a, t: f"包长 {a} 字节，上位机发得太快"),
    _info(209, "无法识别的命令", lambda a, t: f"「{t}」（长度 {a}）"),
    _info(210, "下位机遥测发送缓冲满，部分输出被丢弃", lambda a, t: f"丢弃 {a} 字节"),
    _info(211, "请求补发的事件已被覆盖", lambda a, t: f"下位机最早还保留第 {a} 条"),
    _info(212, "HAL 初始化 Error_Handler 被调用（程序继续运行）",
          lambda a, t: f"调用处地址 0x{a & 0xFFFFFFFF:08X}，用编译生成的 .map 文件查是哪个函数"),
    _info(213, "BMI088 陀螺/加计自检或配置未通过（继续运行）",
          lambda a, t: _bmi088_error(a, t)),
    _info(214, "IST8310 磁力计初始化失败（继续运行）", lambda a, t: f"错误码 {a}"),
    _info(215, "IMU SPI1 初始化失败"),
    _info(216, "舵机 PCA9685 写入失败", lambda a, t: f"舵机 {a}"),
    _info(217, "串口接收意外停止，已重新开启", lambda a, t: f"重开结果 {['成功', 'HAL_ERROR', 'HAL_BUSY', 'HAL_TIMEOUT'][a] if 0 <= a <= 3 else a}"),
    _info(218, "控制任务被某个模块卡住（期间遥测、心跳检查、停机都没执行）",
          lambda a, t: f"{_MODULE_NAMES.get(t, t)} 一次用了 {a} ms（正常整周期 < 7 ms；水压标定时约 2 s 属正常）"),
    _info(219, "串口发送卡住，已强制重启发送", lambda a, t: f"丢弃 {a} 字节（事件会自动补发）"),
    _info(220, "I2C2 总线（推进器/水压计）卡死，已复位总线",
          lambda a, t: _i2c_status(a)),
    _info(221, "供电跌落", lambda a, t: (f"C 板输入电压降到 {a / 1000:.2f} V（低于空闲时的 60%）" if t == "VIN"
                                          else f"单片机 3.3 V 降到 {a / 1000:.3f} V") + "，推进器电流把电源拉低了"),
    _info(222, "水下水平标定没做成，偏置保持原样", lambda a, t: _LEVEL_FAIL.get(a, f"原因 {a} {t}")),
    # ---- 3xx 信息 ----
    _info(301, "下位机上电/复位", lambda a, t: _reset_cause(a)),
    _info(302, "ON：深度闭环开启", lambda a, t: f"目标深度 {a / 10:.1f} mbar（约 cm）"),
    _info(303, "TES：手动 PWM 生效", lambda a, t: f"解析到 {a} 路"),
    _info(304, "ESKF 姿态融合就绪", lambda a, t: f"标定重力 {a / 1000:.3f} m/s²"),
    _info(305, "偏航闭环开启 (RPY:ON)"),
    _info(306, "偏航闭环关闭", lambda a, t: "ACL:OF" if a == 1 else "RPY:OFF"),
    _info(307, "锁定当前航向 (ACL:ON)", lambda a, t: f"目标 {a / 100:.1f}°，来源 {t}"),
    _info(308, "水压计零点标定开始（约 2 s 内控制暂停）"),
    _info(309, "水压计零点", lambda a, t: f"{t} = {a / 1000:.3f} mbar"),
    _info(310, "水压计零点标定结束"),
    _info(311, "闭环反馈来源切换", lambda a, t: "四路水压差（与原版相同）" if a == 0 else "ESKF 融合"),
    _info(312, "水下水平标定开始", lambda a, t: f"约 {a / 1000:.0f} s，期间保持静止、不要碰车"),
    _info(313, "水压计水下偏置", lambda a, t: f"{SENSOR_NAMES.get(int(t[2:]) if t.startswith('CH') and t[2:].isdigit() else -1, t)} = {a / 1000:+.3f} mbar"),
    _info(314, "水下偏置已生效", lambda a, t: {"LOAD": "上电读取：", "SAVE": "标定完成并写入 Flash：", "DEF": "已恢复默认："}.get(t, "")
          + ("LVL 标定值" if a == 1 else "程序默认值（10-20 沉底数据）")),
    # ---- 9xx 致命 ----
    _info(901, "HardFault（单片机已停止）",
          lambda a, t: f"出错指令地址 0x{a & 0xFFFFFFFF:08X}（0 表示发生在中断里）；{t}"),
    _info(902, "MemManage 内存保护错误（单片机已停止）",
          lambda a, t: f"地址 0x{a & 0xFFFFFFFF:08X}；{t}"),
    _info(903, "BusFault 总线错误（单片机已停止）",
          lambda a, t: f"地址 0x{a & 0xFFFFFFFF:08X}；{t}"),
    _info(904, "UsageFault 非法指令/除零/未对齐（单片机已停止）",
          lambda a, t: f"地址 0x{a & 0xFFFFFFFF:08X}；{t}"),
    _info(905, "进入未处理的中断（单片机已停止）", lambda a, t: f"IRQ 编号 {a}"),
    _info(906, "FreeRTOS 断言失败（单片机已停止）", lambda a, t: f"{t} 第 {a} 行"),
    _info(907, "任务栈溢出（单片机已停止）", lambda a, t: f"任务 {t}"),
    _info(908, "FreeRTOS 调度器启动失败（单片机已停止）"),
]}


def describe(code: int, arg: int, text: str) -> tuple[str, str, str]:
    """返回 (级别, 标题, 细节)。未知编号也能显示，不会抛异常。"""
    info = CODES.get(code)
    if info is None:
        level = {1: "stop", 2: "warn", 3: "info", 9: "fatal"}.get(code // 100, "warn")
        return level, f"未知事件 {code}", f"参数 {arg} {text}".strip()
    detail = ""
    if info.detail is not None:
        try:
            detail = info.detail(arg, text)
        except Exception:  # 细节格式化失败也要显示原始值
            detail = f"参数 {arg} {text}".strip()
    return info.level, info.name, detail
