"""FinsROV 上位机控制台（白底）。

    python finsrov_console.py                 # 在界面里选串口
    python finsrov_console.py --port COM11    # 启动即连接
    python finsrov_console.py --mapping my_pad.json

右侧上方是实时曲线（姿态、角速度及目标、深度、控制量、推进器 PWM，可选），
下方"事件记录"显示下位机本次上电以来的全部事件（停机、警告、信息、致命），中间的分隔条可以上下拖动。
事件
按下位机序号排列；串口丢行会自动请求补发；下位机重启会另起一段。
上位机自己的操作（发 ON/OFF、断线……）以蓝色气泡显示在右侧，时间按下位机时间估算插入。
所有记录同时写入 logs/ 目录。
"""

from __future__ import annotations

import argparse
import datetime as dt
import time
import tkinter as tk
import tkinter.font as tkfont
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import fins_codes
import fins_protocol
from fins_chart import StripChart
from fins_gamepad import Gamepad, load_mapping
from fins_link import SerialLink, list_ports

TICK_MS = 33                      # 界面 / 手柄刷新约 30 Hz（与原上位机一致）
TELEMETRY_LOST_S = 2.0            # 推进器在动但 2 s 没收到状态行 → 上位机主动发 OFF
NEUTRAL_PWM = 1550

COLORS = {
    "bg": "#FFFFFF", "panel": "#F6F7F9", "line": "#E3E6EA", "text": "#1F2328", "muted": "#6B7280",
    "stop": "#C62828", "stop_bg": "#FDECEC", "fatal": "#FFFFFF", "fatal_bg": "#B71C1C",
    "warn": "#B45309", "warn_bg": "#FFF6E5", "info": "#374151", "info_bg": "#F3F4F6",
    "host": "#1D4ED8", "host_bg": "#EAF1FF", "ok": "#15803D", "accent": "#2563EB",
}
LEVEL_ICON = {"fatal": "■", "stop": "●", "warn": "▲", "info": "·", "host": ""}


def board_time(ms: int | None) -> str:
    if ms is None or ms < 0:
        return "--:--.---"
    minutes, rest = divmod(ms, 60000)
    return f"{minutes:02d}:{rest / 1000:06.3f}"


class Console:
    def __init__(self, root: tk.Tk, args) -> None:
        self.root = root
        self.args = args
        # 先准备日志和事件记录（手柄初始化时就可能写记录）
        self.filters = {lv: tk.BooleanVar(value=True) for lv in ("info", "warn", "stop", "host")}
        self._render_all_pending = True
        self._last_rendered_key = None
        logs = Path(__file__).with_name("logs")
        logs.mkdir(exist_ok=True)
        self.log_path = logs / f"finsrov_{dt.datetime.now():%Y%m%d_%H%M%S}.log"
        self.log_file = self.log_path.open("a", encoding="utf-8")
        self.link = SerialLink()
        self.tracker = fins_protocol.EventTracker(self._on_tracker_change)
        self.gamepad = Gamepad(load_mapping(args.mapping), log=self.note)

        self.servo = [1500, 1500, 1500, 1500]
        self.servo_dirty = False
        self.last_motion = ""
        self.last_motion_time = 0.0
        self.depth_hold_since = {}
        self.keys_held: set[str] = set()
        self._key_release_jobs: dict[str, str] = {}
        self.local_arm_intent = False
        self.telemetry_lost_reported = False
        self.resend_requests = 0
        self._tick_count = 0

        self._pad = None

        self._build_ui()
        self._bind_keys()
        # 初始把右侧上下按约 6:4 分（之后可以拖动分隔条）
        self.root.after(300, lambda: self.right_pane.sashpos(0, int(self.right_pane.winfo_height() * 0.58)))
        self.note("上位机已启动，日志：logs/" + self.log_path.name, "host")
        if args.port:
            self.port_var.set(args.port)
            self.toggle_connect()
        self.root.after(TICK_MS, self.tick)

    # ======================================================================
    # 界面
    # ======================================================================
    def _build_ui(self) -> None:
        root = self.root
        root.title("FinsROV 上位机")
        root.configure(bg=COLORS["bg"])
        root.geometry("1400x860")
        root.minsize(1100, 680)
        families = set(tkfont.families())
        family = next((f for f in ("Microsoft YaHei UI", "Microsoft YaHei", "PingFang SC",
                                   "Noto Sans CJK SC", "WenQuanYi Micro Hei") if f in families), "TkDefaultFont")
        mono = next((f for f in ("Consolas", "Cascadia Mono", "DejaVu Sans Mono") if f in families), "TkFixedFont")
        self.f_normal = tkfont.Font(family=family, size=10)
        self.f_small = tkfont.Font(family=family, size=9)
        self.f_bold = tkfont.Font(family=family, size=10, weight="bold")
        self.f_big = tkfont.Font(family=family, size=16, weight="bold")
        self.f_mono = tkfont.Font(family=mono, size=10)

        style = ttk.Style(root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=COLORS["bg"], font=self.f_normal)
        style.configure("TFrame", background=COLORS["bg"])
        style.configure("Panel.TFrame", background=COLORS["panel"])
        style.configure("TLabel", background=COLORS["bg"], foreground=COLORS["text"])
        style.configure("Panel.TLabel", background=COLORS["panel"])
        style.configure("Muted.TLabel", background=COLORS["bg"], foreground=COLORS["muted"], font=self.f_small)
        style.configure("TCheckbutton", background=COLORS["bg"])
        style.configure("TButton", padding=(10, 4))
        style.configure("Danger.TButton", foreground="#FFFFFF", background=COLORS["stop"])
        style.map("Danger.TButton", background=[("active", "#A61B1B")])
        style.configure("Accent.TButton", foreground="#FFFFFF", background=COLORS["accent"])
        style.map("Accent.TButton", background=[("active", "#1E40AF")])

        # ---- 顶栏 ----
        top = ttk.Frame(root, padding=(12, 8))
        top.pack(fill="x")
        ttk.Label(top, text="串口").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=28)
        self.port_box.pack(side="left", padx=6)
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(top, text="连接", style="Accent.TButton", command=self.toggle_connect)
        self.connect_button.pack(side="left", padx=6)
        self.link_label = tk.Label(top, text="● 未连接", fg=COLORS["muted"], bg=COLORS["bg"], font=self.f_bold)
        self.link_label.pack(side="left", padx=12)
        self.telemetry_label = ttk.Label(top, text="", style="Muted.TLabel")
        self.telemetry_label.pack(side="left", padx=6)
        self.refresh_ports()

        ttk.Separator(root).pack(fill="x")

        body = ttk.Frame(root, padding=(12, 10))
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        self._build_status(body)
        # 右侧：上曲线、下事件记录，中间分隔条可上下拖动
        self.right_pane = ttk.PanedWindow(body, orient="vertical")
        self.right_pane.grid(row=0, column=1, sticky="nsew")
        self.chart = StripChart(self.right_pane, {"normal": self.f_normal, "small": self.f_small,
                                                  "bold": self.f_bold})
        self.right_pane.add(self.chart, weight=3)
        chat_holder = ttk.Frame(self.right_pane)
        self.right_pane.add(chat_holder, weight=2)
        self._build_chat(chat_holder)
        style = ttk.Style(self.root)
        style.configure("Sash", sashthickness=8, gripcount=12)

    def _kv(self, parent, row, label, var_name, font=None):
        ttk.Label(parent, text=label, style="Panel.TLabel", foreground=COLORS["muted"]).grid(
            row=row, column=0, sticky="w", pady=2)
        value = tk.Label(parent, text="--", bg=COLORS["panel"], fg=COLORS["text"],
                         font=font or self.f_normal, anchor="w", justify="left")
        value.grid(row=row, column=1, sticky="w", padx=(10, 0), pady=2)
        setattr(self, var_name, value)

    def _build_status(self, body) -> None:
        left = ttk.Frame(body, style="Panel.TFrame", padding=12)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        self.mode_label = tk.Label(left, text="未连接", bg=COLORS["panel"], fg=COLORS["muted"],
                                   font=self.f_big, anchor="w")
        self.mode_label.grid(row=0, column=0, columnspan=2, sticky="w")
        self.stop_label = tk.Label(left, text="", bg=COLORS["panel"], fg=COLORS["stop"], font=self.f_small,
                                   anchor="w", justify="left", wraplength=290)
        self.stop_label.grid(row=1, column=0, columnspan=2, sticky="w", pady=(2, 8))
        rows = [("心跳", "v_heartbeat"), ("深度", "v_depth"), ("姿态", "v_attitude"), ("运动", "v_motion"),
                ("水压计", "v_pressure"), ("推进器", "v_pwm"), ("舵机", "v_servo"), ("手柄", "v_pad")]
        for i, (label, name) in enumerate(rows, start=2):
            self._kv(left, i, label, name, self.f_mono if name in ("v_pwm",) else None)

        ttk.Separator(left).grid(row=20, column=0, columnspan=2, sticky="ew", pady=10)
        buttons = ttk.Frame(left, style="Panel.TFrame")
        buttons.grid(row=21, column=0, columnspan=2, sticky="ew")
        for c in range(3):
            buttons.columnconfigure(c, weight=1)
        ttk.Button(buttons, text="解锁 ON", style="Accent.TButton", command=self.cmd_on).grid(
            row=0, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(buttons, text="停止 OFF", command=self.cmd_off).grid(row=0, column=1, sticky="ew", padx=2, pady=2)
        ttk.Button(buttons, text="急停", style="Danger.TButton", command=self.cmd_estop).grid(
            row=0, column=2, sticky="ew", padx=2, pady=2)
        ttk.Button(buttons, text="上浮 UP", command=lambda: self.send("UP")).grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(buttons, text="下潜 DN", command=lambda: self.send("DN")).grid(row=1, column=1, sticky="ew", padx=2, pady=2)
        ttk.Button(buttons, text="舵机回中", command=self.servo_center_all).grid(row=1, column=2, sticky="ew", padx=2, pady=2)
        ttk.Button(buttons, text="锁定航向", command=self.cmd_yaw_on).grid(row=2, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(buttons, text="关闭航向", command=lambda: self.send("RPY:OFF")).grid(row=2, column=1, sticky="ew", padx=2, pady=2)
        ttk.Button(buttons, text="水压标定", command=self.cmd_calibrate).grid(row=2, column=2, sticky="ew", padx=2, pady=2)

        entry_row = ttk.Frame(left, style="Panel.TFrame")
        entry_row.grid(row=22, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(entry_row, textvariable=self.command_var, width=22)
        self.command_entry.pack(side="left", fill="x", expand=True)
        self.command_entry.bind("<Return>", lambda e: self.send_free_command())
        ttk.Button(entry_row, text="发送", command=self.send_free_command).pack(side="left", padx=(4, 0))
        tk.Label(left, text="键盘：W/A/S/D 平移  Q/E 旋转  R 上浮  F 下潜\n"
                            "Enter 解锁/停止  空格 急停  L 锁定航向\n"
                            "命令框可发任意命令，例如 H:300  TES:1550,…",
                 bg=COLORS["panel"], fg=COLORS["muted"], font=self.f_small, justify="left").grid(
            row=23, column=0, columnspan=2, sticky="w", pady=(8, 0))

    def _build_chat(self, right) -> None:
        header = ttk.Frame(right)
        header.pack(fill="x", pady=(8, 0))
        ttk.Label(header, text="事件记录", font=self.f_bold).pack(side="left")
        self.chat_sub = ttk.Label(header, text="下位机本次上电起的全部事件", style="Muted.TLabel")
        self.chat_sub.pack(side="left", padx=8)
        ttk.Button(header, text="导出", command=self.export_log).pack(side="right")
        for level, text in (("host", "上位机"), ("info", "信息"), ("warn", "警告"), ("stop", "停机")):
            ttk.Checkbutton(header, text=text, variable=self.filters[level],
                            command=self.request_full_render).pack(side="right", padx=2)

        self.fatal_banner = tk.Label(right, text="", bg=COLORS["fatal_bg"], fg=COLORS["fatal"],
                                     font=self.f_bold, anchor="w", justify="left", padx=10, pady=6)

        frame = ttk.Frame(right)
        frame.pack(fill="both", expand=True, pady=(6, 0))
        self.chat = tk.Text(frame, wrap="word", bg=COLORS["bg"], fg=COLORS["text"], font=self.f_normal,
                            relief="flat", padx=8, pady=6, highlightthickness=1,
                            highlightbackground=COLORS["line"], cursor="arrow")
        scroll = ttk.Scrollbar(frame, command=self.chat.yview)
        self.chat.configure(yscrollcommand=scroll.set, state="disabled")
        self.chat.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        t = self.chat
        t.tag_configure("meta", foreground=COLORS["muted"], font=self.f_small)
        t.tag_configure("detail", foreground=COLORS["muted"], font=self.f_small)
        for level in ("info", "warn", "stop"):
            t.tag_configure(level, foreground=COLORS[level], background=COLORS[level + "_bg"],
                            lmargin1=6, lmargin2=6, rmargin=120, spacing1=6, spacing3=2)
        t.tag_configure("fatal", foreground=COLORS["fatal"], background=COLORS["fatal_bg"],
                        lmargin1=6, lmargin2=6, rmargin=120, spacing1=6, spacing3=2)
        t.tag_configure("host", foreground=COLORS["host"], background=COLORS["host_bg"], justify="right",
                        lmargin1=160, lmargin2=160, rmargin=6, spacing1=6, spacing3=2)
        t.tag_configure("title", font=self.f_bold)
        t.tag_configure("separator", foreground=COLORS["muted"], justify="center", spacing1=12, spacing3=8,
                        font=self.f_small)
        self.status_bar = ttk.Label(right, text="", style="Muted.TLabel")
        self.status_bar.pack(anchor="w", pady=(4, 0))

    # ======================================================================
    # 键盘
    # ======================================================================
    def _bind_keys(self) -> None:
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _typing(self) -> bool:
        return isinstance(self.root.focus_get(), (tk.Entry, ttk.Entry, ttk.Combobox))

    def _on_key_press(self, event) -> None:
        if self._typing():
            return
        key = event.keysym.lower()
        job = self._key_release_jobs.pop(key, None)
        if job:
            self.root.after_cancel(job)        # Linux 自动连发：松开后马上又按下，视为一直按住
        first = key not in self.keys_held
        self.keys_held.add(key)
        if not first:
            return
        if key == "return":
            self.toggle_arm()
        elif key == "space":
            self.cmd_estop()
        elif key == "r":
            self.send("UP")
        elif key == "f":
            self.send("DN")
        elif key == "l":
            self.toggle_yaw()

    def _on_key_release(self, event) -> None:
        key = event.keysym.lower()

        def release():
            self._key_release_jobs.pop(key, None)
            self.keys_held.discard(key)
        self._key_release_jobs[key] = self.root.after(40, release)

    # ======================================================================
    # 命令
    # ======================================================================
    def send(self, command: str, note: bool = True) -> bool:
        ok = self.link.send(command)
        if note and command not in ("HB",) and not command.startswith(("MOT:", "EVQ:")):
            if command in ("W", "A", "S", "D", "Q", "E", "Z", "UP", "DN"):
                pass   # 运动命令太频繁，不进聊天框
            else:
                self.note(f"发送 {command}" + ("" if ok else "（失败：串口未连接）"), "host" if ok else "warn")
        return ok

    def cmd_on(self) -> None:
        self.local_arm_intent = True
        self.send("ON")

    def cmd_off(self) -> None:
        self.local_arm_intent = False
        self.send("OFF")

    def cmd_estop(self) -> None:
        self.local_arm_intent = False
        ok = False
        for _ in range(3):
            ok = self.link.send("OFF") or ok
        self.note("急停：连发 3 次 OFF" + ("" if ok else "（串口未连接！请直接断电）"), "host" if ok else "warn")

    def board_mode(self) -> int | None:
        session = self.tracker.current
        status = session.last_status if session else None
        if status is None or time.time() - status.pc_time > TELEMETRY_LOST_S:
            return None
        return status.mode

    def toggle_arm(self) -> None:
        mode = self.board_mode()
        active = mode in (1, 2, 3) if mode is not None else self.local_arm_intent
        self.cmd_off() if active else self.cmd_on()

    def cmd_yaw_on(self) -> None:
        # 先 ACL:ON 锁定当前航向为目标，再 RPY:ON 打开偏航闭环（均需深度闭环已开启）
        self.send("ACL:ON")
        self.send("RPY:ON")

    def toggle_yaw(self) -> None:
        if self.board_mode() == 2:
            self.send("RPY:OFF")
        else:
            self.cmd_yaw_on()

    def cmd_calibrate(self) -> None:
        if self.board_mode() not in (0, None):
            messagebox.showwarning("水压标定", "推进器在工作时不能标定，请先 OFF。")
            return
        if messagebox.askokcancel("水压标定", "标定会把当前读数当作水面零点。\n"
                                              "请确认四个水压计口都在空气中、潜器静止。继续？"):
            self.send("CA")

    def send_free_command(self) -> None:
        command = self.command_var.get().strip()
        if command:
            self.send(command)
            self.command_var.set("")

    def servo_center_all(self) -> None:
        self.servo = [1500, 1500, 1500, 1500]
        self.servo_dirty = True

    # ======================================================================
    # 连接
    # ======================================================================
    def refresh_ports(self) -> None:
        ports = list_ports()
        self.port_box["values"] = [f"{device}  {desc}" for device, desc in ports]
        if ports and not self.port_var.get():
            self.port_var.set(ports[0][0])

    def toggle_connect(self) -> None:
        if self.link._want_open:   # 已在连接/重连中 → 断开
            self.link.stop(send_off=True)
            self.connect_button.configure(text="连接")
            self.note(f"已断开 {self.link.port_name}（断开前发送了 OFF）", "host")
            return
        port = self.port_var.get().split()[0] if self.port_var.get().strip() else ""
        if not port:
            messagebox.showinfo("串口", "请先选择串口")
            return
        self.link.start(port)
        self.connect_button.configure(text="断开")

    # ======================================================================
    # 事件记录
    # ======================================================================
    def note(self, text: str, level: str = "host") -> None:
        self.tracker.add_note(text, level)

    def _on_tracker_change(self, kind, session, item) -> None:
        if kind == "status":
            return
        if kind == "session":
            self._write_log(f"==== 下位机会话 {session.index} ====")
            self.request_full_render()
            return
        if kind == "fatal":
            self._write_log(f"[FATAL] 板上 {board_time(item.t_ms)} 代码 {item.code}")
            return
        if kind == "event":
            self._write_log(f"#{item.seq} 板上 {board_time(item.t_ms)} {item.code} {item.title}"
                            + (f" — {item.detail}" if item.detail else ""))
            key = (session.index, item.t_ms, 0, item.seq)
        else:  # note
            self._write_log(f"[上位机 {dt.datetime.fromtimestamp(item.pc_time):%H:%M:%S}] {item.text}")
            estimate = item.board_ms_estimate if item.board_ms_estimate is not None else -1
            key = (session.index, estimate, 1, len(session.notes) - 1)
        if (not self._render_all_pending and self._last_rendered_key is not None
                and session is self.tracker.current and key >= self._last_rendered_key):
            self._append_item(item)          # 正常情况：追加在末尾
            self._last_rendered_key = key
        else:
            self.request_full_render()       # 补发的旧事件：整体重排

    def _write_log(self, line: str) -> None:
        try:
            self.log_file.write(f"{dt.datetime.now():%H:%M:%S.%f}"[:-3] + "  " + line + "\n")
            self.log_file.flush()
        except OSError:
            pass

    def request_full_render(self) -> None:
        self._render_all_pending = True

    def _visible(self, item) -> bool:
        level = item.level if isinstance(item, fins_protocol.Event) else "host"
        if level == "fatal":
            return True
        if level not in ("info", "warn", "stop"):
            level = "host" if level == "host" else "warn"
        return self.filters[level].get()

    def _insert_item(self, item) -> None:
        t = self.chat
        if isinstance(item, fins_protocol.Event):
            level = item.level
            icon = LEVEL_ICON.get(level, "")
            t.insert("end", f"#{item.seq}  板上 {board_time(item.t_ms)}   "
                            f"{dt.datetime.fromtimestamp(item.pc_time):%H:%M:%S}\n", (level, "meta"))
            t.insert("end", f"{icon} {item.code}  {item.title}\n", (level, "title"))
            if item.detail:
                t.insert("end", item.detail + "\n", (level, "detail"))
        else:
            level = "host" if item.level == "host" else "warn"
            when = f"{dt.datetime.fromtimestamp(item.pc_time):%H:%M:%S}"
            if item.board_ms_estimate is not None:
                when += f"  ≈板上 {board_time(item.board_ms_estimate)}"
            if level == "host":
                t.insert("end", f"上位机 · {when}\n", ("host", "meta"))
                t.insert("end", item.text + "\n", ("host",))
            else:
                t.insert("end", f"上位机 · {when}\n", ("warn", "meta"))
                t.insert("end", "▲ " + item.text + "\n", ("warn", "title"))
        t.insert("end", "\n", ("meta",))

    def _append_item(self, item) -> None:
        if not self._visible(item):
            return
        at_bottom = self.chat.yview()[1] > 0.98
        self.chat.configure(state="normal")
        self._insert_item(item)
        self.chat.configure(state="disabled")
        if at_bottom:
            self.chat.see("end")

    def _render_all(self) -> None:
        self._render_all_pending = False
        at_bottom = self.chat.yview()[1] > 0.98
        self.chat.configure(state="normal")
        self.chat.delete("1.0", "end")
        last_key = None
        for session in self.tracker.sessions:
            boot = session.events.get(1)
            cause = ""
            if boot is not None and boot.code == 301:
                cause = " · " + boot.detail
            self.chat.insert("end", f"──── 下位机第 {session.index} 段运行（本上位机启动后）{cause} ────\n",
                             ("separator",))
            missing = session.missing()
            note_index = {id(n): i for i, n in enumerate(session.notes)}
            for item in self.tracker.timeline(session):
                if self._visible(item):
                    self._insert_item(item)
                if isinstance(item, fins_protocol.Event):
                    last_key = (session.index, item.t_ms, 0, item.seq)
                else:
                    est = item.board_ms_estimate if item.board_ms_estimate is not None else -1
                    last_key = (session.index, est, 1, note_index[id(item)])
            if missing:
                lost = [s for s in missing if s < session.lost_before]
                text = f"缺少第 {missing[0]}" + (f"~{missing[-1]}" if len(missing) > 1 else "") + " 条"
                text += "（已被覆盖，无法补回）" if lost and len(lost) == len(missing) else "，正在请求补发…"
                self.chat.insert("end", text + "\n", ("separator",))
        self.chat.configure(state="disabled")
        self._last_rendered_key = last_key
        if at_bottom:
            self.chat.see("end")

    def export_log(self) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".txt",
                                            initialfile=f"finsrov_events_{dt.datetime.now():%Y%m%d_%H%M%S}.txt",
                                            filetypes=[("文本", "*.txt"), ("全部", "*.*")])
        if not path:
            return
        lines = []
        for session in self.tracker.sessions:
            lines.append(f"==== 下位机第 {session.index} 段运行 ====")
            for item in self.tracker.timeline(session):
                if isinstance(item, fins_protocol.Event):
                    lines.append(f"#{item.seq}\t板上 {board_time(item.t_ms)}\t{item.code}\t{item.title}\t{item.detail}")
                else:
                    lines.append(f"上位机\t{dt.datetime.fromtimestamp(item.pc_time):%H:%M:%S}\t"
                                 f"≈板上 {board_time(item.board_ms_estimate)}\t{item.text}")
        Path(path).write_text("\n".join(lines), encoding="utf-8")
        self.note(f"已导出 {path}")

    # ======================================================================
    # 主循环
    # ======================================================================
    def tick(self) -> None:
        now = time.time()
        try:
            self._process_link(now)
            self._process_inputs(now)
            self._update_status(now)
            self._tick_count += 1
            if self._tick_count % 3 == 0 and not self.chart._mouse:
                self.chart.render()          # 约 10 Hz 重画曲线（鼠标悬停时由移动事件重画）
            if self._render_all_pending:
                self._render_all()
        finally:
            self.root.after(TICK_MS, self.tick)

    def _process_link(self, now: float) -> None:
        for _ in range(2000):
            try:
                kind, pc_time, text = self.link.inbox.get_nowait()
            except Exception:  # queue.Empty
                break
            if kind == "line":
                item = self.tracker.handle_line(text, pc_time)
                if isinstance(item, fins_protocol.PlotSample):
                    self.chart.push(item.t_ms, item.values)
                elif isinstance(item, fins_protocol.Status):
                    self.chart.push_pwm(item.t_ms, item.pwm)
            elif kind == "link_up":
                self.note(text, "host")
                self.telemetry_lost_reported = False
                self.link.send("EVQ:1")      # 连上就请求从头补发本次上电的所有事件
            elif kind in ("link_down", "link_fail"):
                if kind == "link_down" or not getattr(self, "_last_fail_text", "") == text:
                    self.note(text, "warn")
                self._last_fail_text = text
        seq = self.tracker.resend_request(now)
        if seq is not None and self.link.connected:
            self.link.send(f"EVQ:{seq}")
            self.resend_requests += 1

        # 上位机侧保护：推进器在动但 2 s 收不到状态行
        session = self.tracker.current
        status = session.last_status if session else None
        if status is not None and status.active and session.fatal is None:
            if now - status.pc_time > TELEMETRY_LOST_S and not self.telemetry_lost_reported:
                self.telemetry_lost_reported = True
                if self.link.connected:
                    for _ in range(3):
                        self.link.send("OFF")
                self.note(f"遥测中断超过 {TELEMETRY_LOST_S:.0f} s，推进器最后状态是在工作，上位机已发送 OFF"
                          + ("" if self.link.connected else "（串口已断开，发不出去！）"), "warn")
        if status is not None and now - status.pc_time < 0.5:
            self.telemetry_lost_reported = False

    def _process_inputs(self, now: float) -> None:
        pad = self.gamepad.poll(now)
        self._pad = pad
        p = pad.pressed
        if "arm_toggle" in p:
            self.toggle_arm()
        if "estop" in p:
            self.cmd_estop()
        if "yaw_hold_toggle" in p:
            self.toggle_yaw()
        if "servo_left_center" in p:
            self.servo[0] = self.servo[1] = 1500
            self.servo_dirty = True
        if "servo_right_center" in p:
            self.servo[2] = self.servo[3] = 1500
            self.servo_dirty = True

        # 上浮/下潜：按下发一次，按住 0.5 s 后每 0.2 s 再发
        for name, key, command in (("ascend", "r", "UP"), ("descend", "f", "DN")):
            held = name in pad.held or key in self.keys_held
            if held:
                since = self.depth_hold_since.get(command)
                if since is None:
                    self.depth_hold_since[command] = now
                    if name in p:
                        self.send(command)
                elif now - since > 0.5:
                    self.send(command)
                    self.depth_hold_since[command] = now - 0.3
            else:
                self.depth_hold_since.pop(command, None)

        # 平移/旋转：与原上位机相同的命令，按住期间每 0.2 s 重发，松开发一次 Z
        hat = pad.hat
        keys = self.keys_held
        motion = "Z"
        if hat == (0, 1) or "w" in keys:
            motion = "W"
        elif hat == (0, -1) or "s" in keys:
            motion = "S"
        elif hat == (-1, 0) or "a" in keys:
            motion = "A"
        elif hat == (1, 0) or "d" in keys:
            motion = "D"
        elif "yaw_ccw" in pad.held or "q" in keys:
            motion = "Q"
        elif "yaw_cw" in pad.held or "e" in keys:
            motion = "E"
        if motion != self.last_motion or (motion != "Z" and now - self.last_motion_time > 0.2):
            if self.link.connected and self.link.send(motion):
                self.last_motion = motion
                self.last_motion_time = now

        # 舵机：与原上位机相同的速率（30 Hz × 20 us × 摇杆量）和范围
        axes = pad.axes
        lv, lh = axes.get("left_vertical", 0.0), axes.get("left_horizontal", 0.0)
        rv, rh = axes.get("right_vertical", 0.0), axes.get("right_horizontal", 0.0)
        if lv:
            self.servo[0] = max(min(int(self.servo[0] + lv * 20), 2000), 1000)
            self.servo_dirty = True
        if lh:
            self.servo[1] = max(min(int(self.servo[1] - lh * 20), 2500), 500)
            self.servo_dirty = True
        if rv:
            self.servo[2] = max(min(int(self.servo[2] - rv * 20), 2000), 1000)
            self.servo_dirty = True
        if rh:
            self.servo[3] = max(min(int(self.servo[3] - rh * 20), 2500), 500)
            self.servo_dirty = True
        if self.servo_dirty and self.link.connected:
            if self.link.send("MOT:{},{},{},{}".format(*self.servo)):
                self.servo_dirty = False

    def _update_status(self, now: float) -> None:
        connected = self.link.connected
        session = self.tracker.current
        status = session.last_status if session else None
        fresh = status is not None and now - status.pc_time < TELEMETRY_LOST_S
        fatal = session.fatal if session else None

        if connected:
            self.link_label.configure(text=f"● 已连接 {self.link.port_name}", fg=COLORS["ok"])
        elif self.link._want_open:
            self.link_label.configure(text=f"● 正在连接 {self.link.port_name}…", fg=COLORS["warn"])
        else:
            self.link_label.configure(text="● 未连接", fg=COLORS["muted"])
        if status is not None:
            age = now - status.pc_time
            self.telemetry_label.configure(
                text=f"板上运行 {board_time(status.t_ms + int(age * 1000) if fresh else status.t_ms)}   "
                     f"状态行 {age * 1000:.0f} ms 前   事件 #{session.max_seq_seen}")
        else:
            self.telemetry_label.configure(text="尚未收到下位机状态行" if connected else "")

        if fatal is not None:
            info = fins_codes.CODES.get(fatal.code)
            name = info.name if info else str(fatal.code)
            self.fatal_banner.configure(text=f"■ 致命错误 {fatal.code}：{name}\n"
                                             f"单片机已停止运行（板上 {board_time(fatal.t_ms)}），推进器已尽力写中位。"
                                             f"请断电或按复位键重启。")
            if not self.fatal_banner.winfo_ismapped():
                self.fatal_banner.pack(fill="x", before=self.chat.master, pady=(6, 0))
        elif self.fatal_banner.winfo_ismapped():
            self.fatal_banner.pack_forget()

        if fatal is not None:
            self.mode_label.configure(text="单片机已停止", fg=COLORS["stop"])
        elif not fresh:
            self.mode_label.configure(text="无遥测" if connected else "未连接", fg=COLORS["muted"])
        else:
            mode = fins_codes.MODE_NAMES.get(status.mode, str(status.mode))
            self.mode_label.configure(text=mode, fg=COLORS["ok"] if status.mode else COLORS["text"])
        if status is not None and status.stop_code:
            _, title, _ = fins_codes.describe(status.stop_code, 0, "")
            self.stop_label.configure(text=f"最近一次停机：{status.stop_code} {title}")
        else:
            self.stop_label.configure(text="")

        if status is not None:
            silent_s = status.silent_cycles / 150
            self.v_heartbeat.configure(
                text=f"下位机 {silent_s:.2f} s 未收到数据" + ("（超过 0.67 s 会停机）" if silent_s > 0.3 else ""),
                fg=COLORS["warn"] if silent_s > 0.3 else COLORS["text"])
            self.v_depth.configure(text=f"当前 {status.depth_x10 / 10:.1f}   目标 {status.target_x10 / 10:.1f}  mbar")
            self.v_attitude.configure(
                text=f"横滚 {status.roll_cd / 100:+.1f}°  俯仰 {status.pitch_cd / 100:+.1f}°  "
                     f"偏航 {status.yaw_cd / 100:+.1f}°\n来源 {'ESKF 融合' if status.attitude_from_eskf else 'IMU（ESKF 未就绪）'}"
                     "  · 抬头/右倾/顺时针为正")
            self.v_motion.configure(text=fins_codes.MOTION_NAMES.get(status.motion, str(status.motion)))
            dots = "  ".join(("●" if status.pressure_mask & (1 << i) else "○") + fins_codes.SENSOR_NAMES[i][3:]
                             for i in range(4))
            self.v_pressure.configure(text=dots, fg=COLORS["text"] if status.pressure_mask == 15 else COLORS["warn"])
            pwm = status.pwm
            self.v_pwm.configure(text="\n".join(
                "  ".join(f"{i}:{pwm[i]:4d}" + ("*" if pwm[i] != NEUTRAL_PWM else " ") for i in range(r, r + 4))
                for r in (0, 4)) + "\n(* 非中位 1550)")
        self.v_servo.configure(text="  ".join(str(v) for v in self.servo))
        pad = getattr(self, "_pad", None)
        if pad is not None:
            text = pad.name
            if pad.disabled_axes:
                text += f"\n已屏蔽轴 {pad.disabled_axes}"
            self.v_pad.configure(text=text, fg=COLORS["text"] if pad.connected else COLORS["muted"])
        missing = len(session.missing()) if session else 0
        self.status_bar.configure(
            text=f"收到 {self.tracker.lines_ok} 行 · 校验错误 {self.tracker.checksum_errors} · "
                 f"补发请求 {self.resend_requests} · 当前缺号 {missing} · 日志 {self.log_path.name}")

    def on_close(self) -> None:
        if self.link.connected:
            self.note("关闭上位机：发送 OFF", "host")
        self.link.stop(send_off=True)
        try:
            self.log_file.close()
        except OSError:
            pass
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="FinsROV 上位机控制台")
    parser.add_argument("--port", help="串口，例如 COM11")
    parser.add_argument("--mapping", help="手柄映射 json（pad_probe.py 生成的格式）")
    args = parser.parse_args()
    root = tk.Tk()
    try:
        root.tk.call("tk", "scaling", 1.25)
    except tk.TclError:
        pass
    Console(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
