"""曲线窗口（仿 Scope 的网页界面）的本地服务。

上位机主程序把解析好的曲线行 / 状态行 / 事件交给 ScopeServer，
浏览器打开 http://127.0.0.1:<端口>/ 显示（web/scope.html，纯本地文件，不需要联网）。

只提供只读数据：浏览器这边不能给下位机发命令，避免误操作；遥控仍在主窗口。
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import fins_codes

WEB_DIR = Path(__file__).with_name("web")
RETENTION_FRAMES = 30 * 60 * 15          # 约 15 分钟的 30 Hz 曲线行（状态行另算）
MAX_FRAMES_PER_REPLY = 6000

# (键, 分组, 名称, 单位, 颜色, 小数位, 默认显示)
# 颜色按"角色"分：角度/深度 蓝(横滚、深度) 橙(俯仰) 紫(偏航)，实际角速度 绿，目标值 黄，控制量 红；
# 这样任意一个调参组合里四条线颜色都不同。颜色可在曲线窗口里改（浏览器会记住）
CHANNELS = [
    ("roll", "姿态", "横滚", "°", "#58a6ff", 2, True),
    ("pitch", "姿态", "俯仰", "°", "#f0883e", 2, True),
    ("yaw", "姿态", "偏航", "°", "#bc8cff", 2, False),
    ("target_yaw", "姿态", "目标偏航", "°", "#d29922", 2, False),
    ("roll_rate", "角速度", "横滚角速度", "°/s", "#3fb950", 2, False),
    ("target_roll_rate", "角速度", "目标横滚角速度", "°/s", "#d29922", 2, False),
    ("pitch_rate", "角速度", "俯仰角速度", "°/s", "#3fb950", 2, False),
    ("target_pitch_rate", "角速度", "目标俯仰角速度", "°/s", "#d29922", 2, False),
    ("yaw_rate", "角速度", "偏航角速度", "°/s", "#3fb950", 2, False),
    ("target_yaw_rate", "角速度", "目标偏航角速度", "°/s", "#d29922", 2, False),
    ("depth", "深度", "深度", "mbar", "#58a6ff", 2, False),
    ("target_depth", "深度", "目标深度", "mbar", "#d29922", 2, False),
    ("u_depth", "控制量", "深度控制量", "µs", "#f85149", 1, False),
    ("u_roll", "控制量", "横滚控制量", "µs", "#f85149", 1, False),
    ("u_pitch", "控制量", "俯仰控制量", "µs", "#f85149", 1, False),
    ("u_yaw", "控制量", "偏航控制量", "µs", "#f85149", 1, False),
    ("pwm0", "推进器 PWM", "0 前左水平", "µs", "#58a6ff", 0, False),
    ("pwm1", "推进器 PWM", "1 前左垂直", "µs", "#f0883e", 0, False),
    ("pwm2", "推进器 PWM", "2 后左垂直", "µs", "#3fb950", 0, False),
    ("pwm3", "推进器 PWM", "3 后左水平", "µs", "#d29922", 0, False),
    ("pwm4", "推进器 PWM", "4 后右水平", "µs", "#db61a2", 0, False),
    ("pwm5", "推进器 PWM", "5 后右垂直", "µs", "#a5d6ff", 0, False),
    ("pwm6", "推进器 PWM", "6 前右垂直", "µs", "#bc8cff", 0, False),
    ("pwm7", "推进器 PWM", "7 前右水平", "µs", "#f85149", 0, False),
    ("pres0", "水压计", "0 左前", "mbar", "#58a6ff", 2, False),
    ("pres1", "水压计", "1 左后", "mbar", "#f0883e", 2, False),
    ("pres2", "水压计", "2 右后", "mbar", "#3fb950", 2, False),
    ("pres3", "水压计", "3 右前", "mbar", "#bc8cff", 2, False),
    ("accel_x", "加速度计", "X 前", "m/s²", "#58a6ff", 3, False),
    ("accel_y", "加速度计", "Y 右", "m/s²", "#f0883e", 3, False),
    ("accel_z", "加速度计", "Z 下", "m/s²", "#3fb950", 3, False),
    ("accel_norm", "加速度计", "模长 |a|", "m/s²", "#f85149", 3, False),
    # 控制量分解（$PD）：P 蓝、I 黄、D 紫、内环角速度阻尼 绿；各项相加 = 对应的控制量
    ("pid_depth_p", "深度 PID 分解", "深度 P", "µs", "#58a6ff", 1, False),
    ("pid_depth_i", "深度 PID 分解", "深度 I", "µs", "#d29922", 1, False),
    ("pid_depth_d", "深度 PID 分解", "深度 D", "µs", "#bc8cff", 1, False),
    ("pid_roll_p", "横滚 PID 分解", "横滚 P", "µs", "#58a6ff", 1, False),
    ("pid_roll_i", "横滚 PID 分解", "横滚 I", "µs", "#d29922", 1, False),
    ("pid_roll_d", "横滚 PID 分解", "横滚 D", "µs", "#bc8cff", 1, False),
    ("pid_roll_rate", "横滚 PID 分解", "横滚 角速度阻尼", "µs", "#3fb950", 1, False),
    ("pid_pitch_p", "俯仰 PID 分解", "俯仰 P", "µs", "#58a6ff", 1, False),
    ("pid_pitch_i", "俯仰 PID 分解", "俯仰 I", "µs", "#d29922", 1, False),
    ("pid_pitch_d", "俯仰 PID 分解", "俯仰 D", "µs", "#bc8cff", 1, False),
    ("pid_pitch_rate", "俯仰 PID 分解", "俯仰 角速度阻尼", "µs", "#3fb950", 1, False),
    ("vin", "供电", "输入电压", "V", "#f85149", 2, False),
    ("vdda", "供电", "单片机 3.3 V", "V", "#bc8cff", 3, False),
]

# (名称, 左纵轴通道, 右纵轴通道)：调参组合把控制量放右轴，免得和角度挤在一起
PRESETS = [
    ("俯仰调参", ["pitch", "pitch_rate", "target_pitch_rate"], ["u_pitch"]),
    ("横滚调参", ["roll", "roll_rate", "target_roll_rate"], ["u_roll"]),
    ("偏航调参", ["yaw", "target_yaw", "yaw_rate", "target_yaw_rate"], ["u_yaw"]),
    ("深度调参", ["depth", "target_depth"], ["u_depth"]),
    ("俯仰 PID 分解", ["u_pitch", "pid_pitch_p", "pid_pitch_i", "pid_pitch_d", "pid_pitch_rate"], ["pitch"]),
    ("横滚 PID 分解", ["u_roll", "pid_roll_p", "pid_roll_i", "pid_roll_d", "pid_roll_rate"], ["roll"]),
    ("深度 PID 分解", ["u_depth", "pid_depth_p", "pid_depth_i", "pid_depth_d"], ["depth"]),
    ("垂直推进器", ["pwm1", "pwm2", "pwm5", "pwm6"], []),
    ("水压计", ["pres0", "pres1", "pres2", "pres3"], []),
    ("加速度计", ["accel_x", "accel_y", "accel_z", "accel_norm"], []),
    ("供电", ["vin"], ["vdda"]),
]

STATUS_KEYS = [f"pwm{i}" for i in range(8)]


class ScopeServer:
    def __init__(self, preferred_port: int = 8765) -> None:
        self._lock = threading.Lock()
        self._frames: deque = deque(maxlen=RETENTION_FRAMES)
        self._events: deque = deque(maxlen=400)
        self._seq = 0
        self._event_seq = 0
        self._info: dict = {"mode": None, "feedback": None, "connected": False, "last_rx": 0.0}
        self._httpd = None
        self.port = 0
        self._start(preferred_port)

    # ---- 数据入口（主线程调用） ---------------------------------------------
    def publish_plot(self, t_ms: int, values: dict, session: int) -> None:
        clean = {k: (round(v, 5) if isinstance(v, float) and v == v else None)
                 for k, v in values.items()}
        with self._lock:
            self._seq += 1
            self._frames.append((self._seq, t_ms, session, clean))
            self._info["last_rx"] = time.time()

    def publish_status(self, status, session: int) -> None:
        values = {f"pwm{i}": int(v) for i, v in enumerate(status.pwm[:8])}
        with self._lock:
            self._seq += 1
            self._frames.append((self._seq, status.t_ms, session, values))
            self._info.update(mode=status.mode, feedback=status.feedback_eskf,
                              stop_code=status.stop_code, last_rx=time.time())

    def publish_event(self, event, session: int) -> None:
        level, title, detail = fins_codes.describe(event.code, event.arg, event.text)
        with self._lock:
            self._event_seq += 1
            self._events.append({"id": self._event_seq, "session": session, "seq": event.seq, "t": event.t_ms,
                                 "code": event.code, "level": level, "title": title, "detail": detail})

    def set_connected(self, connected: bool) -> None:
        with self._lock:
            self._info["connected"] = connected

    # ---- HTTP ---------------------------------------------------------------
    def _snapshot(self, since: int, since_event: int) -> dict:
        with self._lock:
            frames = [f for f in self._frames if f[0] > since] if since else list(self._frames)
            truncated = len(frames) > MAX_FRAMES_PER_REPLY
            if truncated:
                frames = frames[:MAX_FRAMES_PER_REPLY]
            events = [e for e in self._events if e["id"] > since_event]
            info = dict(self._info)
            info["age"] = time.time() - info["last_rx"] if info["last_rx"] else None
            latest = self._seq
        return {"latest": latest, "more": truncated, "frames": frames, "events": events, "info": info,
                "modes": fins_codes.MODE_NAMES}

    def _start(self, preferred_port: int) -> None:
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args) -> None:   # 不在控制台刷访问日志
                pass

            def _send(self, code: int, body: bytes, ctype: str) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802
                url = urlparse(self.path)
                try:
                    if url.path in ("/", "/index.html"):
                        self._send(200, (WEB_DIR / "scope.html").read_bytes(), "text/html; charset=utf-8")
                    elif url.path == "/api/meta":
                        meta = {"channels": [dict(key=k, group=g, label=l, unit=u, color=c, digits=d, visible=v)
                                             for k, g, l, u, c, d, v in CHANNELS],
                                "presets": [dict(name=n, keys=k + r, right=r) for n, k, r in PRESETS]}
                        self._send(200, json.dumps(meta, ensure_ascii=False).encode(), "application/json")
                    elif url.path == "/api/data":
                        q = parse_qs(url.query)
                        since = int(q.get("since", ["0"])[0])
                        since_event = int(q.get("ev", ["0"])[0])
                        body = json.dumps(server._snapshot(since, since_event), ensure_ascii=False,
                                          separators=(",", ":")).encode()
                        self._send(200, body, "application/json")
                    else:
                        self._send(404, b"not found", "text/plain")
                except (BrokenPipeError, ConnectionResetError):
                    pass

        for port in range(preferred_port, preferred_port + 20):
            try:
                self._httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
                self.port = port
                break
            except OSError:
                continue
        if self._httpd is not None:
            self._httpd.daemon_threads = True
            threading.Thread(target=self._httpd.serve_forever, name="fins-web", daemon=True).start()

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.port}/" if self.port else ""

    def close(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
