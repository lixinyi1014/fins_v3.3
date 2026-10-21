"""把下位机的曲线行（$PL，约 30 Hz）和状态行（$ST，约 10 Hz）存成 CSV，方便事后画图排查。

每次启动上位机新建一个文件：log_csv/finsrov_日期_时间.csv。
每一行是一条 $PL 曲线数据，后面附上当时最近一条状态行的内容（模式、目标深度、8 路 PWM、反馈来源……）
和最近一条 $PD（深度/横滚/俯仰控制量的 P、I、D 分解）。
下位机重启时 session 列加一，board_ms 从 0 重新开始。
"""

from __future__ import annotations

import csv
import datetime as dt
import math
import time
from pathlib import Path

import fins_protocol

STATUS_COLUMNS = (["mode", "stop_code", "silent_cycles", "st_target_depth", "st_depth", "motion"]
                  + [f"pwm{i}" for i in range(8)] + ["pressure_mask", "attitude_from_eskf", "feedback_eskf"])


class TelemetryCsv:
    def __init__(self, folder: Path) -> None:
        folder.mkdir(exist_ok=True)
        self.path = folder / f"finsrov_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
        self._file = self.path.open("w", encoding="utf-8-sig", newline="")   # 带 BOM，Excel 直接打开不乱码
        self._writer = csv.writer(self._file)
        self._plot_keys = [name for name, _ in fins_protocol.PLOT_FIELDS] + ["accel_norm"]
        self._writer.writerow(["pc_time", "session", "board_ms"] + self._plot_keys + STATUS_COLUMNS
                              + fins_protocol.PID_FIELDS)
        self._status: fins_protocol.Status | None = None
        self._pid: dict = {}
        self._last_flush = time.monotonic()
        self.rows = 0

    def on_status(self, status: fins_protocol.Status) -> None:
        self._status = status

    def on_pid(self, sample: fins_protocol.PidSample) -> None:
        self._pid = sample.values      # 和 $PL 错开约 13 ms，CSV 里附在下一行曲线数据后面

    def on_plot(self, sample: fins_protocol.PlotSample, session: int) -> None:
        def fmt(value) -> str:
            if value is None or (isinstance(value, float) and not math.isfinite(value)):
                return ""
            return f"{value:.4g}" if isinstance(value, float) else str(value)

        row = [dt.datetime.fromtimestamp(sample.pc_time).strftime("%H:%M:%S.%f")[:-3], session, sample.t_ms]
        row += [fmt(sample.values.get(key)) for key in self._plot_keys]
        s = self._status
        if s is not None:
            row += [s.mode, s.stop_code, s.silent_cycles, s.target_x10 / 10, s.depth_x10 / 10, s.motion,
                    *s.pwm[:8], s.pressure_mask, int(s.attitude_from_eskf),
                    "" if s.feedback_eskf < 0 else s.feedback_eskf]
        else:
            row += [""] * len(STATUS_COLUMNS)
        row += [fmt(self._pid.get(key)) for key in fins_protocol.PID_FIELDS]
        try:
            self._writer.writerow(row)
            self.rows += 1
            now = time.monotonic()
            if now - self._last_flush > 1.0:      # 每秒落盘一次，断电/崩溃最多丢 1 s
                self._file.flush()
                self._last_flush = now
        except OSError:
            pass

    def close(self) -> None:
        try:
            self._file.close()
        except OSError:
            pass
