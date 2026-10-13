"""手柄输入（pygame）。没装 pygame 或没插手柄时界面照常可用，只用键盘/按钮。

按键映射沿用原 FinsROV 上位机（Xbox 布局）：
    Start(7)  ON / OFF 切换        Back(6)  急停（连发 OFF）
    A(0)      上浮 UP              B(1)     下潜 DN
    X(2)      逆时针 Q             Y(3)     顺时针 E
    LB(4)     偏航锁定开/关         十字键   前后左右 W/S/A/D
    左摇杆按下(8) 左侧舵机回中      右摇杆按下(9) 右侧舵机回中
    左右摇杆  四路舵机（与原程序同样的速率和范围）

可以用 --mapping my_pad.json 覆盖编号，格式与 fins_v3 的 pad_probe.py 生成的一致。
启动时静止读数明显不为 0 的轴（一般是扳机）会被自动屏蔽，避免舵机一开机就跑到底。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

try:
    import os
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame
except ImportError:
    pygame = None

DEFAULT_MAPPING = {
    "buttons": {"ascend": 0, "descend": 1, "yaw_ccw": 2, "yaw_cw": 3, "yaw_hold_toggle": 4,
                "estop": 6, "arm_toggle": 7, "servo_left_center": 8, "servo_right_center": 9},
    "axes": {"left_horizontal": 0, "left_vertical": 1, "right_horizontal": 2, "right_vertical": 3},
    "hat": 0,
    "deadzone": 0.5,
}


def load_mapping(path: str | None) -> dict:
    mapping = json.loads(json.dumps(DEFAULT_MAPPING))
    if not path:
        beside = Path(__file__).with_name("my_pad.json")
        if not beside.is_file():
            return mapping
        path = str(beside)
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    for section in ("buttons", "axes"):
        if section in data:
            mapping[section].update({k: v for k, v in data[section].items() if k in mapping[section]})
    if "hat" in data:
        mapping["hat"] = data["hat"]
    return mapping


@dataclass
class PadState:
    connected: bool = False
    name: str = "未连接"
    hat: tuple[int, int] = (0, 0)
    held: set[str] = field(default_factory=set)      # 按住的逻辑按键
    pressed: set[str] = field(default_factory=set)   # 本帧刚按下
    axes: dict[str, float] = field(default_factory=dict)
    disabled_axes: list[int] = field(default_factory=list)


class Gamepad:
    def __init__(self, mapping: dict, log=print) -> None:
        self.mapping = mapping
        self._log = log
        self._joystick = None
        self._previous_buttons: set[str] = set()
        self._disabled_axes: set[int] = set()
        self.available = pygame is not None
        if self.available:
            try:
                pygame.display.init()
                pygame.joystick.init()
            except Exception as exc:  # noqa: BLE001
                self._log(f"手柄初始化失败：{exc}", "warn")
                self.available = False
        self._next_scan = 0.0

    def _acquire(self, now: float) -> None:
        if self._joystick is not None or now < self._next_scan:
            return
        self._next_scan = now + 1.0
        try:
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                return
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        except Exception:  # noqa: BLE001
            return
        self._joystick = joystick
        pygame.event.pump()
        self._disabled_axes = set()
        for name, index in self.mapping["axes"].items():
            try:
                value = joystick.get_axis(index)
            except Exception:  # noqa: BLE001
                self._disabled_axes.add(index)
                continue
            if abs(value) > 0.5:
                self._disabled_axes.add(index)
                self._log(f"手柄轴 {index}（{name}）静止读数 {value:+.2f}，不是摇杆，已屏蔽。"
                          f"请用 my_pad.json 指定正确的轴号", "warn")
        self._log(f"手柄已连接：{joystick.get_name()}", "host")

    def poll(self, now: float) -> PadState:
        state = PadState()
        if not self.available:
            state.name = "未安装 pygame"
            return state
        try:
            for event in pygame.event.get():
                if event.type == getattr(pygame, "JOYDEVICEREMOVED", -1):
                    self._joystick = None
                    self._log("手柄已断开", "warn")
        except Exception:  # noqa: BLE001
            pass
        self._acquire(now)
        joystick = self._joystick
        if joystick is None:
            return state
        try:
            state.connected = True
            state.name = joystick.get_name()
            buttons = self.mapping["buttons"]
            for name, index in buttons.items():
                if index < joystick.get_numbuttons() and joystick.get_button(index):
                    state.held.add(name)
            state.pressed = state.held - self._previous_buttons
            self._previous_buttons = set(state.held)
            if joystick.get_numhats() > self.mapping["hat"]:
                state.hat = tuple(joystick.get_hat(self.mapping["hat"]))
            dead = self.mapping.get("deadzone", 0.5)
            for name, index in self.mapping["axes"].items():
                value = 0.0
                if index not in self._disabled_axes and index < joystick.get_numaxes():
                    value = joystick.get_axis(index)
                state.axes[name] = value if abs(value) > dead else 0.0
            state.disabled_axes = sorted(self._disabled_axes)
        except Exception:  # noqa: BLE001  手柄中途拔掉
            self._joystick = None
            self._previous_buttons = set()
            return PadState()
        return state
