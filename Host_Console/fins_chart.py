"""实时曲线（纯 tkinter Canvas，不依赖 matplotlib）。

* 每种单位一个小图（姿态角 °、角速度 °/s、深度 mbar、控制量 us、推进器 PWM us、水压计 mbar、加速度 m/s²），绝不把两种单位画在同一根 y 轴上。
* 每个轴向固定一个颜色（横滚蓝、俯仰橙、偏航青、深度黄），实际值实线、目标值同色虚线，跨图一致。
* "曲线"菜单勾选要显示的量，也有常用组合；鼠标移到图上显示十字线和该时刻的数值。
* 时间轴用下位机板上时间，丢行不会把曲线拉歪；下位机重启自动清空。
"""

from __future__ import annotations

import math
import tkinter as tk
from collections import deque
from tkinter import ttk

# 参考调色板（dataviz skill references/palette.md，浅色模式，已验证色盲区分度）
BLUE, ORANGE, AQUA, YELLOW, MAGENTA, GREEN, VIOLET, RED = (
    "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948")
TEXT, MUTED, GRID, AXIS, BORDER, CROSS = "#1F2328", "#6B7280", "#EEF0F2", "#C7CCD1", "#E3E6EA", "#9CA3AF"

GROUPS = [("angle", "姿态角", "°"), ("rate", "角速度", "°/s"), ("depth", "深度", "mbar"),
          ("control", "控制量", "us"), ("pwm", "推进器 PWM", "us"),
          ("pressure", "水压计", "mbar"), ("accel", "加速度计", "m/s²"),
          ("vin", "输入电压", "V"), ("vdda", "单片机电源", "V")]

# (键, 组, 名称, 颜色, 虚线=目标值, 小数位)
SERIES = [
    ("roll", "angle", "横滚", BLUE, False, 1),
    ("pitch", "angle", "俯仰", ORANGE, False, 1),
    ("yaw", "angle", "偏航", AQUA, False, 1),
    ("target_yaw", "angle", "目标偏航", AQUA, True, 1),
    ("roll_rate", "rate", "横滚", BLUE, False, 1),
    ("target_roll_rate", "rate", "目标横滚", BLUE, True, 1),
    ("pitch_rate", "rate", "俯仰", ORANGE, False, 1),
    ("target_pitch_rate", "rate", "目标俯仰", ORANGE, True, 1),
    ("yaw_rate", "rate", "偏航", AQUA, False, 1),
    ("target_yaw_rate", "rate", "目标偏航", AQUA, True, 1),
    ("depth", "depth", "深度", YELLOW, False, 1),
    ("target_depth", "depth", "目标深度", YELLOW, True, 1),
    ("u_depth", "control", "深度", YELLOW, False, 0),
    ("u_roll", "control", "横滚", BLUE, False, 0),
    ("u_pitch", "control", "俯仰", ORANGE, False, 0),
    ("u_yaw", "control", "偏航", AQUA, False, 0),
] + [(f"pwm{i}", "pwm", f"{i}号", c, False, 0)
     for i, c in enumerate((BLUE, ORANGE, AQUA, YELLOW, MAGENTA, GREEN, VIOLET, RED))] + [
    ("pres0", "pressure", "0 左前", BLUE, False, 1),
    ("pres1", "pressure", "1 左后", ORANGE, False, 1),
    ("pres2", "pressure", "2 右后", GREEN, False, 1),
    ("pres3", "pressure", "3 右前", VIOLET, False, 1),
    ("accel_x", "accel", "X 前", BLUE, False, 2),
    ("accel_y", "accel", "Y 右", ORANGE, False, 2),
    ("accel_z", "accel", "Z 下", AQUA, False, 2),
    ("accel_norm", "accel", "模长 |a|", RED, False, 2),
    ("vin", "vin", "输入电压", RED, False, 2),
    ("vdda", "vdda", "3.3 V", VIOLET, False, 3),
]
SERIES_BY_KEY = {s[0]: s for s in SERIES}

PRESETS = [
    ("姿态调参：横滚/俯仰", ["roll", "pitch", "roll_rate", "target_roll_rate", "pitch_rate", "target_pitch_rate",
                      "u_roll", "u_pitch"]),
    ("偏航调参", ["yaw", "target_yaw", "yaw_rate", "target_yaw_rate", "u_yaw"]),
    ("深度调参", ["depth", "target_depth", "u_depth"]),
    ("推进器 PWM", [f"pwm{i}" for i in range(8)]),
    ("传感器：水压计 + 加速度计", ["pres0", "pres1", "pres2", "pres3", "accel_x", "accel_y", "accel_z", "accel_norm"]),
    ("供电排查：电压 + 控制量", ["vin", "vdda", "u_depth", "u_roll", "u_pitch", "u_yaw"]),
]
DEFAULT_SELECTION = ["roll", "pitch", "roll_rate", "target_roll_rate", "pitch_rate", "target_pitch_rate",
                     "depth", "target_depth"]

LEFT, RIGHT, LEGEND_H, BOTTOM = 58, 14, 22, 22


def nice_step(span: float, count: int = 4) -> float:
    raw = span / max(count, 1)
    if raw <= 0 or not math.isfinite(raw):
        return 1.0
    power = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * power:
            return m * power
    return 10 * power


class StripChart(ttk.Frame):
    def __init__(self, master, fonts: dict) -> None:
        super().__init__(master)
        self.fonts = fonts
        self.data = {key: deque(maxlen=30 * 130) for key, *_ in SERIES}
        self.selected = {key: tk.BooleanVar(value=key in DEFAULT_SELECTION) for key, *_ in SERIES}
        self.window_s = tk.StringVar(value="30 s")
        self.paused = tk.BooleanVar(value=False)
        self._last_t = None
        self._pause_t = None
        self._mouse = None

        bar = ttk.Frame(self)
        bar.pack(fill="x")
        ttk.Label(bar, text="实时曲线", font=fonts["bold"]).pack(side="left")
        menu_button = ttk.Menubutton(bar, text="曲线 ▾")
        menu = tk.Menu(menu_button, tearoff=False)
        for name, keys in PRESETS:
            menu.add_command(label="常用：" + name, command=lambda k=keys: self.select_only(k))
        menu.add_command(label="全部取消", command=lambda: self.select_only([]))
        for group, gname, unit in GROUPS:
            menu.add_separator()
            menu.add_command(label=f"{gname}（{unit}）", state="disabled")
            for key, g, label, *_ in SERIES:
                if g == group:
                    menu.add_checkbutton(label="   " + label, variable=self.selected[key], command=self.render)
        menu_button["menu"] = menu
        menu_button.pack(side="left", padx=(12, 4))
        ttk.Label(bar, text="时长").pack(side="left", padx=(8, 2))
        window_box = ttk.Combobox(bar, textvariable=self.window_s, values=["10 s", "30 s", "60 s", "120 s"],
                                  width=6, state="readonly")
        window_box.pack(side="left")
        window_box.bind("<<ComboboxSelected>>", lambda e: self.render())
        ttk.Checkbutton(bar, text="暂停", variable=self.paused, command=self._toggle_pause).pack(side="left", padx=8)
        ttk.Button(bar, text="清空", command=self.clear).pack(side="left")
        ttk.Label(bar, text="实线=实际  虚线=目标  方向：抬头/右倾/顺时针为正",
                  foreground=MUTED, font=fonts["small"]).pack(side="right")

        self.canvas = tk.Canvas(self, bg="#FFFFFF", highlightthickness=1, highlightbackground=BORDER)
        self.canvas.pack(fill="both", expand=True, pady=(4, 0))
        self.canvas.bind("<Configure>", lambda e: self.render())
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<Leave>", self._on_leave)

    # ---- 数据 -----------------------------------------------------------------
    def _check_time(self, t: float) -> None:
        if self._last_t is not None and t < self._last_t - 1.0:
            self.clear()                    # 板上时间倒退：下位机重启
        self._last_t = t if self._last_t is None else max(self._last_t, t)

    def push(self, t_ms: int, values: dict) -> None:
        t = t_ms / 1000.0
        self._check_time(t)
        for key, value in values.items():
            if key in self.data:
                self.data[key].append((t, value))

    def push_pwm(self, t_ms: int, pwm: list[int]) -> None:
        self.push(t_ms, {f"pwm{i}": float(v) for i, v in enumerate(pwm[:8])})

    def clear(self) -> None:
        for series in self.data.values():
            series.clear()
        self._last_t = None
        self._pause_t = None
        self.render()

    def select_only(self, keys) -> None:
        for key, var in self.selected.items():
            var.set(key in keys)
        self.render()

    def _toggle_pause(self) -> None:
        self._pause_t = self._last_t if self.paused.get() else None
        self.render()

    def _on_motion(self, event) -> None:
        self._mouse = (event.x, event.y)
        self.render()

    def _on_leave(self, _event) -> None:
        self._mouse = None
        self.render()

    # ---- 绘制 -----------------------------------------------------------------
    def render(self) -> None:
        c = self.canvas
        c.delete("all")
        width, height = c.winfo_width(), c.winfo_height()
        if width < 120 or height < 80:
            return
        groups = [g for g in GROUPS if any(self.selected[k].get() for k, grp, *_ in SERIES if grp == g[0])]
        if not groups:
            c.create_text(width / 2, height / 2, text="在左上角「曲线」菜单里选择要显示的量",
                          fill=MUTED, font=self.fonts["normal"])
            return
        window = float(self.window_s.get().split()[0])
        t_end = self._pause_t if self.paused.get() and self._pause_t is not None else self._last_t
        if t_end is None:
            c.create_text(width / 2, height / 2, text="等待下位机曲线数据（$PL）…", fill=MUTED,
                          font=self.fonts["normal"])
            return
        t0 = t_end - window
        x0, x1 = LEFT, width - RIGHT
        panel_h = (height - BOTTOM) / len(groups)
        hover_t = None
        if self._mouse and x0 <= self._mouse[0] <= x1:
            hover_t = t0 + (self._mouse[0] - x0) / (x1 - x0) * window
        tick = nice_step(window, 6)
        first_tick = math.ceil(t0 / tick) * tick

        def tx(t):
            return x0 + (t - t0) / window * (x1 - x0)

        tooltip = None
        for index, (group, gname, unit) in enumerate(groups):
            top = index * panel_h
            y0, y1 = top + LEGEND_H, top + panel_h - 6
            keys = [k for k, grp, *_ in SERIES if grp == group and self.selected[k].get()]
            visible = {k: [(t, v) for t, v in self.data[k] if t0 <= t <= t_end] for k in keys}
            values = [v for pts in visible.values() for _, v in pts if math.isfinite(v)]
            if values:
                lo, hi = min(values), max(values)
            else:
                lo, hi = -1.0, 1.0
            if group in ("rate", "control", "angle"):
                lo, hi = min(lo, 0.0), max(hi, 0.0)   # 这几类看正负，带上 0
            if hi - lo < 1e-6:
                lo, hi = lo - 1, hi + 1
            pad = (hi - lo) * 0.08
            lo, hi = lo - pad, hi + pad
            step = nice_step(hi - lo, 4)

            def ty(v, lo=lo, hi=hi, y0=y0, y1=y1):
                return y1 - (v - lo) / (hi - lo) * (y1 - y0)

            c.create_rectangle(x0, y0, x1, y1, outline=BORDER)
            v = math.ceil(lo / step) * step
            while v <= hi:
                y = ty(v)
                c.create_line(x0, y, x1, y, fill=AXIS if abs(v) < step * 1e-6 else GRID)
                c.create_text(x0 - 6, y, text=f"{v:g}", anchor="e", fill=MUTED, font=self.fonts["small"])
                v += step
            t = first_tick
            while t <= t_end + 1e-9:
                x = tx(t)
                c.create_line(x, y0, x, y1, fill=GRID)
                if index == len(groups) - 1:
                    c.create_text(x, y1 + 12, text=f"{t - t_end:.0f} s", fill=MUTED, font=self.fonts["small"])
                t += tick

            # 标题 + 图例（含当前值；文字用正文色，颜色只给线段）
            lx = x0
            title = c.create_text(lx, top + 11, text=f"{gname}（{unit}）", anchor="w", fill=TEXT,
                                  font=self.fonts["bold"])
            lx = c.bbox(title)[2] + 14
            for key in keys:
                _, _, label, color, dashed, digits = SERIES_BY_KEY[key]
                pts = visible[key]
                current = pts[-1][1] if pts else float("nan")
                c.create_line(lx, top + 11, lx + 18, top + 11, fill=color, width=2, dash=(5, 3) if dashed else ())
                text = f"{label} {current:.{digits}f}" if math.isfinite(current) else f"{label} --"
                item = c.create_text(lx + 22, top + 11, text=text, anchor="w", fill=TEXT, font=self.fonts["small"])
                lx = c.bbox(item)[2] + 12

            # 曲线：超过像素数就抽稀；NaN 处断开
            for key in keys:
                _, _, _, color, dashed, _ = SERIES_BY_KEY[key]
                pts = visible[key]
                stride = max(1, len(pts) // max(1, int(x1 - x0)))
                segment = []
                for t, value in pts[::stride] + ([pts[-1]] if pts and stride > 1 else []):
                    if not math.isfinite(value):
                        if len(segment) >= 4:
                            c.create_line(*segment, fill=color, width=2, dash=(6, 4) if dashed else ())
                        segment = []
                        continue
                    segment += [tx(t), ty(value)]
                if len(segment) >= 4:
                    c.create_line(*segment, fill=color, width=2, dash=(6, 4) if dashed else ())

            # 悬停：十字线 + 各曲线在该时刻的点
            if hover_t is not None:
                hx = tx(hover_t)
                c.create_line(hx, y0, hx, y1, fill=CROSS)
                rows = []
                for key in keys:
                    _, _, label, color, dashed, digits = SERIES_BY_KEY[key]
                    pts = visible[key]
                    if not pts:
                        continue
                    nearest = min(pts, key=lambda p: abs(p[0] - hover_t))
                    if abs(nearest[0] - hover_t) > max(0.5, window / 200) or not math.isfinite(nearest[1]):
                        continue
                    y = ty(nearest[1])
                    c.create_oval(hx - 4, y - 4, hx + 4, y + 4, fill=color, outline="#FFFFFF", width=2)
                    rows.append((label, color, dashed, f"{nearest[1]:.{digits}f} {unit}"))
                if self._mouse and y0 <= self._mouse[1] <= y1 and rows:
                    tooltip = (hover_t - t_end, rows)

        if tooltip is not None:
            self._draw_tooltip(*tooltip, width)

    def _draw_tooltip(self, dt: float, rows, width: int) -> None:
        c = self.canvas
        mx, my = self._mouse
        lines = [f"{dt:+.2f} s"] + [f"{label}  {value}" for label, _, _, value in rows]
        line_h = 17
        w = max(len(s) for s in lines) * 7 + 40
        h = line_h * len(lines) + 10
        x = mx + 16 if mx + 16 + w < width else mx - 16 - w
        y = max(4, my - h / 2)
        c.create_rectangle(x, y, x + w, y + h, fill="#FFFFFF", outline=BORDER)
        c.create_text(x + 10, y + 5 + line_h / 2, text=lines[0], anchor="w", fill=MUTED, font=self.fonts["small"])
        for i, (label, color, dashed, value) in enumerate(rows, start=1):
            yy = y + 5 + line_h * i + line_h / 2
            c.create_line(x + 10, yy, x + 24, yy, fill=color, width=2, dash=(5, 3) if dashed else ())
            c.create_text(x + 30, yy, text=f"{label}  {value}", anchor="w", fill=TEXT, font=self.fonts["small"])
