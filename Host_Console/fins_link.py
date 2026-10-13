"""串口链路：接收线程 + 100 ms 心跳线程 + 断线自动重连。

所有收到的行放进队列，由界面线程取出处理（tkinter 只能在主线程里操作）。
"""

from __future__ import annotations

import queue
import threading
import time

import fins_protocol

try:
    import serial
    import serial.tools.list_ports
except ImportError:          # 允许在没装 pyserial 的机器上打开界面看看
    serial = None


def list_ports() -> list[tuple[str, str]]:
    if serial is None:
        return []
    return [(p.device, p.description) for p in serial.tools.list_ports.comports()]


class SerialLink:
    def __init__(self) -> None:
        self.port_name = ""
        self._serial = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._want_open = False
        self.inbox: "queue.Queue[tuple[str, float, str]]" = queue.Queue()  # (类型, 电脑时间, 内容)
        self.heartbeat_enabled = True
        self.bytes_in = 0
        self.lines_in = 0
        self.send_failures = 0
        self.last_rx_time = 0.0
        self._threads: list[threading.Thread] = []

    # ---- 状态 -----------------------------------------------------------------
    @property
    def connected(self) -> bool:
        with self._lock:
            return self._serial is not None and self._serial.is_open

    def _post(self, kind: str, text: str) -> None:
        self.inbox.put((kind, time.time(), text))

    # ---- 打开 / 关闭 ----------------------------------------------------------
    def start(self, port: str) -> None:
        self.stop()
        self.port_name = port
        self._want_open = True
        self._stop.clear()
        self._threads = [threading.Thread(target=self._rx_loop, name="fins-rx", daemon=True),
                         threading.Thread(target=self._hb_loop, name="fins-hb", daemon=True)]
        for t in self._threads:
            t.start()

    def stop(self, send_off: bool = False) -> None:
        if send_off and self.connected:
            for _ in range(3):
                self.send("OFF")
                time.sleep(0.03)
        self._want_open = False
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self._threads = []
        self._close_port()

    def _open_port(self) -> bool:
        if serial is None:
            self._post("link", "未安装 pyserial：pip install pyserial")
            return False
        try:
            handle = serial.Serial(port=self.port_name, baudrate=fins_protocol.BAUDRATE,
                                   bytesize=fins_protocol.BYTESIZE, parity=fins_protocol.PARITY,
                                   stopbits=fins_protocol.STOPBITS, timeout=0.05, write_timeout=0.2)
        except Exception as exc:  # noqa: BLE001  任何打开失败都只提示、稍后重试
            self._post("link_fail", f"打开 {self.port_name} 失败：{exc}")
            return False
        with self._lock:
            self._serial = handle
        self._post("link_up", f"已连接 {self.port_name}（115200 8E1）")
        return True

    def _close_port(self) -> None:
        with self._lock:
            handle, self._serial = self._serial, None
        if handle is not None:
            try:
                handle.close()
            except Exception:  # noqa: BLE001
                pass

    # ---- 发送 -----------------------------------------------------------------
    def send(self, command: str) -> bool:
        """发送一条命令（自动加换行）。失败返回 False 并计数，不抛异常。"""
        data = (command.strip() + "\n").encode("ascii", errors="replace")
        with self._lock:
            handle = self._serial
            if handle is None:
                self.send_failures += 1
                return False
            try:
                handle.write(data)
                return True
            except Exception as exc:  # noqa: BLE001
                self.send_failures += 1
                self._post("link_down", f"发送失败，串口可能已断开：{exc}")
        self._close_port()
        return False

    # ---- 线程 -----------------------------------------------------------------
    def _rx_loop(self) -> None:
        splitter = fins_protocol.LineSplitter()
        while not self._stop.is_set():
            if not self.connected:
                if not self._want_open or not self._open_port():
                    self._stop.wait(1.0)   # 每秒重试一次
                    continue
            with self._lock:
                handle = self._serial
            try:
                data = handle.read(512) if handle is not None else b""
            except Exception as exc:  # noqa: BLE001
                self._post("link_down", f"串口读取失败（线缆被拔？）：{exc}")
                self._close_port()
                continue
            if not data:
                continue
            now = time.time()
            self.bytes_in += len(data)
            self.last_rx_time = now
            for line in splitter.feed(data):
                self.lines_in += 1
                self.inbox.put(("line", now, line))

    def _hb_loop(self) -> None:
        # 心跳独立成线程：界面卡顿不影响心跳
        next_time = time.monotonic()
        while not self._stop.is_set():
            next_time += fins_protocol.HEARTBEAT_PERIOD_S
            if self.heartbeat_enabled and self.connected:
                self.send("HB")
            delay = next_time - time.monotonic()
            if delay < -1.0:
                next_time = time.monotonic()
            self._stop.wait(max(0.0, delay))
