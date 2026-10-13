"""FinsROV 下位机串口协议：解析、事件排序去重、缺号补发、重启识别。

这一层不碰串口和界面，方便单独测试（test_protocol.py）。

下位机输出（见 EventLog.h）：
    $EV,<seq>,<t_ms>,<code>,<arg>,<text>*HH     事件，seq 上电从 1 递增
    $ST,<t_ms>,<last_seq>,<19 个状态字段>*HH      状态，约 10 Hz
    $FT,<t_ms>,<seq>,<code>*HH                   致命，单片机已停住，约每 0.5 s 重复
    $PL,<t_ms>,<16 个曲线字段>*HH                  画图用，约 30 Hz
上位机命令以换行结尾：
    HB          心跳（100 ms 一次）
    EVQ:<seq>   请求从 seq 开始补发事件

时序原则：
* 事件一律按下位机序号排序，到达顺序不重要（补发的旧事件会插回原位）。
* 每条事件显示下位机上电后的毫秒时间，这是唯一可信的先后依据。
* 上位机自己的动作（发 ON、断线……）按"估算的下位机时间"插入同一时间线，
  估算 = 最近一条状态行的板上时间 + 之后经过的电脑时间。
* 重启识别：状态行时间倒退 > 2 s，或同一序号出现内容不同的事件。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import fins_codes

BAUDRATE = 115200
BYTESIZE = 8
PARITY = "E"      # 下位机 USART6 是 8 数据位 + 偶校验
STOPBITS = 1

HEARTBEAT_PERIOD_S = 0.1
RESEND_INTERVAL_S = 0.5


def _checksum(body: str) -> int:
    value = 0
    for ch in body.encode("ascii", errors="replace"):
        value ^= ch
    return value


def make_line(body: str) -> str:
    """给 $ 之后的内容加校验（测试和模拟用）。"""
    return f"${body}*{_checksum(body):02X}\r\n"


def split_checked(line: str) -> list[str] | None:
    """校验并拆分一行。不是本协议的行或校验错误返回 None。"""
    line = line.strip()
    start = line.rfind("$")
    if start < 0:
        return None
    line = line[start:]
    star = line.rfind("*")
    if star < 0 or star + 3 != len(line):
        return None
    body = line[1:star]
    try:
        expected = int(line[star + 1:], 16)
    except ValueError:
        return None
    if _checksum(body) != expected:
        return None
    return body.split(",")


@dataclass
class Event:
    seq: int
    t_ms: int
    code: int
    arg: int
    text: str
    pc_time: float
    level: str = ""
    title: str = ""
    detail: str = ""

    def __post_init__(self) -> None:
        self.level, self.title, self.detail = fins_codes.describe(self.code, self.arg, self.text)

    def same_record(self, other: "Event") -> bool:
        return (self.t_ms, self.code, self.arg, self.text) == (other.t_ms, other.code, other.arg, other.text)


@dataclass
class Status:
    t_ms: int
    last_seq: int
    mode: int
    stop_code: int
    silent_cycles: int
    target_x10: int
    depth_x10: int
    roll_cd: int
    pitch_cd: int
    yaw_cd: int
    attitude_from_eskf: bool
    motion: int
    pwm: list[int]
    pressure_mask: int
    pc_time: float

    @property
    def active(self) -> bool:
        return self.mode != 0


# $PL 字段顺序（与下位机 Propeller_I2C::FormatPlotFields 一致）和换算系数
PLOT_FIELDS = [
    ("roll", 0.01), ("pitch", 0.01), ("yaw", 0.01), ("target_yaw", 0.01),          # 度
    ("roll_rate", 0.01), ("pitch_rate", 0.01), ("yaw_rate", 0.01),                 # 度/秒
    ("target_roll_rate", 0.01), ("target_pitch_rate", 0.01), ("target_yaw_rate", 0.01),
    ("depth", 0.1), ("target_depth", 0.1),                                         # mbar
    ("u_depth", 0.1), ("u_roll", 0.1), ("u_pitch", 0.1), ("u_yaw", 0.1),           # us
]


@dataclass
class PlotSample:
    t_ms: int
    values: dict[str, float]
    pc_time: float


@dataclass
class Fatal:
    t_ms: int
    seq: int
    code: int
    pc_time: float


def parse_line(line: str, pc_time: float | None = None):
    """返回 Event / Status / Fatal / PlotSample，无法识别返回 None。"""
    fields = split_checked(line)
    if not fields:
        return None
    now = time.time() if pc_time is None else pc_time
    kind = fields[0]
    try:
        if kind == "EV" and len(fields) >= 6:
            return Event(int(fields[1]), int(fields[2]), int(fields[3]), int(fields[4]),
                         ",".join(fields[5:]), now)
        if kind == "ST" and len(fields) >= 3:
            values = [int(x) for x in fields[1:]]
            if len(values) < 2 + 19:
                # 只有时间和序号（例如推进器模块尚未初始化）
                values += [0] * (2 + 19 - len(values))
            t_ms, last_seq, mode, stop, silent, tgt, depth, roll, pitch, yaw, src, motion = values[:12]
            pwm = values[12:20]
            mask = values[20]
            return Status(t_ms, last_seq, mode, stop, silent, tgt, depth, roll, pitch, yaw,
                          bool(src), motion, pwm, mask, now)
        if kind == "PL" and len(fields) >= 2 + len(PLOT_FIELDS):
            raw = [int(x) for x in fields[2:2 + len(PLOT_FIELDS)]]
            values = {name: (v * scale if v != -999999 else float("nan"))
                      for (name, scale), v in zip(PLOT_FIELDS, raw)}
            return PlotSample(int(fields[1]), values, now)
        if kind == "FT" and len(fields) >= 4:
            return Fatal(int(fields[1]), int(fields[2]), int(fields[3]), now)
    except ValueError:
        return None
    return None


class LineSplitter:
    """串口字节流 → 文本行。"""

    def __init__(self, limit: int = 512) -> None:
        self._buffer = bytearray()
        self._limit = limit

    def feed(self, data: bytes) -> list[str]:
        self._buffer.extend(data)
        lines = []
        while True:
            index = self._buffer.find(b"\n")
            if index < 0:
                break
            raw = bytes(self._buffer[:index])
            del self._buffer[:index + 1]
            lines.append(raw.decode("ascii", errors="replace"))
        if len(self._buffer) > self._limit:   # 没有换行的垃圾数据
            del self._buffer[:-self._limit]
        return lines


@dataclass
class HostNote:
    """上位机自己产生的记录（发了什么命令、串口断开……）。"""
    text: str
    level: str
    pc_time: float
    board_ms_estimate: int | None


@dataclass
class Session:
    """下位机的一次上电。"""
    index: int
    started_pc: float
    events: dict[int, Event] = field(default_factory=dict)
    notes: list[HostNote] = field(default_factory=list)
    max_seq_seen: int = 0          # 事件或状态行里出现过的最大序号
    last_t_ms: int = -1
    lost_before: int = 0           # 小于此序号且缺失的事件已被下位机覆盖，补不回来了
    fatal: Fatal | None = None
    last_status: Status | None = None

    def missing(self) -> list[int]:
        return [s for s in range(max(1, self.lost_before), self.max_seq_seen + 1) if s not in self.events]

    def first_missing(self) -> int | None:
        for s in range(max(1, self.lost_before), self.max_seq_seen + 1):
            if s not in self.events:
                return s
        return None


class EventTracker:
    """维护所有会话的事件，决定何时补发。

    回调 on_change(kind, session, item)：
        kind = "session" 新会话；"event" 新事件（可能插在中间）；"note"；"status"；"fatal"
    """

    def __init__(self, on_change=None) -> None:
        self.sessions: list[Session] = []
        self._on_change = on_change or (lambda *a: None)
        self._last_request_time = 0.0
        self._last_request_seq = 0
        self.checksum_errors = 0
        self.lines_ok = 0

    # ---- 会话 -----------------------------------------------------------------
    @property
    def current(self) -> Session | None:
        return self.sessions[-1] if self.sessions else None

    def _new_session(self, pc_time: float) -> Session:
        session = Session(len(self.sessions) + 1, pc_time)
        self.sessions.append(session)
        self._last_request_seq = 0
        self._on_change("session", session, None)
        return session

    def _session_for_time(self, t_ms: int, pc_time: float) -> Session:
        session = self.current
        if session is None:
            return self._new_session(pc_time)
        if session.last_t_ms >= 0 and t_ms + 2000 < session.last_t_ms and session.last_status is not None \
                and t_ms + 2000 < session.last_status.t_ms:
            return self._new_session(pc_time)   # 板上时间倒退：下位机重启了
        return session

    # ---- 输入 -----------------------------------------------------------------
    def handle_line(self, line: str, pc_time: float | None = None):
        item = parse_line(line, pc_time)
        if item is None:
            if "$" in line:
                self.checksum_errors += 1
            return None
        self.lines_ok += 1
        if isinstance(item, Event):
            self.handle_event(item)
        elif isinstance(item, Status):
            self.handle_status(item)
        elif isinstance(item, Fatal):
            self.handle_fatal(item)
        return item

    def handle_event(self, event: Event) -> None:
        session = self.current
        if session is None:
            session = self._new_session(event.pc_time)
        existing = session.events.get(event.seq)
        if existing is not None:
            if existing.same_record(event):
                return                      # 补发的重复事件
            session = self._new_session(event.pc_time)   # 同序号不同内容：重启了
        session.events[event.seq] = event
        session.max_seq_seen = max(session.max_seq_seen, event.seq)
        session.last_t_ms = max(session.last_t_ms, event.t_ms)
        if event.code == 211:               # 下位机告知更早的已被覆盖
            session.lost_before = max(session.lost_before, event.arg)
        self._on_change("event", session, event)

    def handle_status(self, status: Status) -> None:
        session = self._session_for_time(status.t_ms, status.pc_time)
        session.last_status = status
        session.last_t_ms = max(session.last_t_ms, status.t_ms)
        session.max_seq_seen = max(session.max_seq_seen, status.last_seq)
        self._on_change("status", session, status)

    def handle_fatal(self, fatal: Fatal) -> None:
        session = self._session_for_time(fatal.t_ms, fatal.pc_time)
        session.max_seq_seen = max(session.max_seq_seen, fatal.seq)
        session.last_t_ms = max(session.last_t_ms, fatal.t_ms)
        first = session.fatal is None
        session.fatal = fatal
        if first:
            self._on_change("fatal", session, fatal)

    def add_note(self, text: str, level: str = "host", pc_time: float | None = None) -> HostNote:
        now = time.time() if pc_time is None else pc_time
        session = self.current
        estimate = None
        if session is not None and session.last_status is not None and session.fatal is None:
            estimate = session.last_status.t_ms + int((now - session.last_status.pc_time) * 1000)
        if session is None:
            session = self._new_session(now)
        note = HostNote(text, level, now, estimate)
        session.notes.append(note)
        self._on_change("note", session, note)
        return note

    # ---- 补发 -----------------------------------------------------------------
    def resend_request(self, pc_time: float | None = None) -> int | None:
        """需要补发时返回起始序号（调用方发送 EVQ:<seq>），否则 None。"""
        now = time.time() if pc_time is None else pc_time
        session = self.current
        if session is None or session.fatal is not None:
            return None   # 致命状态下下位机自己会循环重发最近的事件
        missing = session.first_missing()
        if missing is None:
            return None
        # 刚收到的最新序号之前的缺口，给正常发送一点时间（下位机每周期限量发送）
        if session.max_seq_seen - missing < 8 and now - self._last_request_time < 0.3:
            return None
        if now - self._last_request_time < RESEND_INTERVAL_S:
            return None
        self._last_request_time = now
        self._last_request_seq = missing
        return missing

    # ---- 显示 -----------------------------------------------------------------
    @staticmethod
    def timeline(session: Session) -> list:
        """按下位机时间排好的事件 + 上位机记录。"""
        items: list[tuple] = []
        for event in session.events.values():
            items.append((event.t_ms, 0, event.seq, event))
        for i, note in enumerate(session.notes):
            key = note.board_ms_estimate if note.board_ms_estimate is not None else -1
            items.append((key, 1, i, note))
        items.sort(key=lambda x: (x[0], x[1], x[2]))
        return [x[3] for x in items]
