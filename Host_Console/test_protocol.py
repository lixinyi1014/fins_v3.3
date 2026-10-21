"""上位机协议测试。

    python test_protocol.py

1. 编号表与下位机 EventLog.h 一致（找得到下位机源码时）
2. 解析、校验、排序去重、缺号补发、重启识别
3. 若设置了环境变量 FINS_HARNESS（指向用真实固件 EventLog.cpp 编译的 PC 版模拟程序），
   再跑一遍端到端：丢字节、补发、致命循环输出。
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import time
import unittest
from pathlib import Path

import fins_codes
import fins_protocol
from fins_protocol import EventTracker, make_line

EVENTLOG_H = Path(__file__).resolve().parent.parent / "Lower_Level_Controller" / "userCode" / "devices" / "Inc" / "EventLog.h"


class CodeTableTest(unittest.TestCase):
    @unittest.skipUnless(EVENTLOG_H.is_file(), "找不到下位机 EventLog.h")
    def test_matches_firmware(self):
        text = EVENTLOG_H.read_text(encoding="utf-8")
        firmware = {int(v) for v in re.findall(r"EV_\w+\s*=\s*(\d+)", text)}
        self.assertTrue(firmware)
        self.assertEqual(firmware, set(fins_codes.CODES), "上位机编号表与 EventLog.h 不一致")

    def test_every_code_describes(self):
        for code in fins_codes.CODES:
            level, title, detail = fins_codes.describe(code, 3, "X")
            self.assertTrue(title)
        self.assertEqual(fins_codes.describe(555, 1, "")[1], "未知事件 555")


class ParserTest(unittest.TestCase):
    def test_checksum(self):
        line = make_line("EV,3,1200,102,100,")
        item = fins_protocol.parse_line(line, 1.0)
        self.assertIsInstance(item, fins_protocol.Event)
        self.assertEqual((item.seq, item.t_ms, item.code, item.arg), (3, 1200, 102, 100))
        self.assertEqual(item.level, "stop")
        broken = line.replace("1200", "1201")
        self.assertIsNone(fins_protocol.parse_line(broken, 1.0))

    def test_plot_line(self):
        line = make_line("PL,35,150,-250,4500,4500,-965,20,-30,-900,10,0,1230,1250,400,-80,35,-999999")
        item = fins_protocol.parse_line(line, 1.0)
        self.assertIsInstance(item, fins_protocol.PlotSample)
        self.assertAlmostEqual(item.values["roll"], 1.5)
        self.assertAlmostEqual(item.values["target_roll_rate"], -9.0)
        self.assertAlmostEqual(item.values["target_depth"], 125.0)
        self.assertNotEqual(item.values["u_yaw"], item.values["u_yaw"])   # NaN
        self.assertNotIn("pres0", item.values)                            # 旧固件 16 项照样能用

    def test_plot_line_with_sensors(self):
        base = "PL,35,150,-250,4500,4500,-965,20,-30,-900,10,0,1230,1250,400,-80,35,0"
        item = fins_protocol.parse_line(make_line(base + ",164,-5,30,12,300,-400,-981"), 1.0)
        self.assertAlmostEqual(item.values["pres0"], 16.4)
        self.assertAlmostEqual(item.values["pres3"], 1.2)
        self.assertAlmostEqual(item.values["accel_z"], -9.81)
        self.assertAlmostEqual(item.values["accel_norm"], (3 ** 2 + 4 ** 2 + 9.81 ** 2) ** 0.5)

    def test_status_feedback_field(self):
        base = "ST,1000,5,1,0,0,300,295,10,-20,4500,1,0,1550,1600,1550,1550,1550,1550,1550,1550,15"
        old = fins_protocol.parse_line(make_line(base), 1.0)
        new = fins_protocol.parse_line(make_line(base + ",1"), 1.0)
        self.assertEqual(old.feedback_eskf, -1)
        self.assertEqual(new.feedback_eskf, 1)
        self.assertEqual(new.pressure_mask, 15)

    def test_csv_recorder(self):
        import csv, tempfile
        from pathlib import Path
        import fins_csv
        with tempfile.TemporaryDirectory() as d:
            rec = fins_csv.TelemetryCsv(Path(d) / "log_csv")
            status = fins_protocol.parse_line(make_line(
                "ST,1000,5,1,0,0,300,295,10,-20,4500,1,0,1550,1600,1550,1550,1550,1550,1550,1550,15,0"), 1.0)
            rec.on_plot(fins_protocol.parse_line(make_line("PL,990," + ",".join(["10"] * 16)), 1.0), 1)
            rec.on_status(status)
            rec.on_plot(fins_protocol.parse_line(make_line("PL,1020," + ",".join(["10"] * 25)), 1.1), 1)
            rec.close()
            rows = list(csv.DictReader(rec.path.open(encoding="utf-8-sig")))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["mode"], "")
        self.assertEqual(rows[1]["pwm1"], "1600")
        self.assertEqual(rows[1]["feedback_eskf"], "0")
        self.assertEqual(rows[1]["vin"], "0.1")
        self.assertIn("pid_pitch_i", rows[1])

    def test_pid_line(self):
        item = fins_protocol.parse_line(make_line("PD,1234,105,-20,3,40,60,-5,-30,-120,90,0,-999999"), 1.0)
        self.assertIsInstance(item, fins_protocol.PidSample)
        self.assertAlmostEqual(item.values["pid_depth_p"], 10.5)
        self.assertAlmostEqual(item.values["pid_pitch_p"], -12.0)
        self.assertNotEqual(item.values["pid_pitch_rate"], item.values["pid_pitch_rate"])   # NaN

    def test_garbage_prefix(self):
        line = "\x00\xffgarbage" + make_line("ST,10,0")
        self.assertIsInstance(fins_protocol.parse_line(line, 1.0), fins_protocol.Status)

    def test_splitter(self):
        s = fins_protocol.LineSplitter()
        self.assertEqual(s.feed(b"$A*41\r\n$B"), ["$A*41\r"])
        self.assertEqual(s.feed(b"*42\n"), ["$B*42"])


def status(t, seq, mode=1):
    return make_line(f"ST,{t},{seq},{mode},0,0,300,100,0,0,0,1,1,1550,1550,1550,1550,1550,1550,1550,1550,15")


class TrackerTest(unittest.TestCase):
    def setUp(self):
        self.tracker = EventTracker()

    def feed(self, body, t=100.0):
        return self.tracker.handle_line(make_line(body) if not body.startswith("$") else body, t)

    def test_order_and_duplicates(self):
        self.feed("EV,1,0,301,8,")
        self.feed("EV,3,500,302,300,")
        self.assertEqual(self.tracker.resend_request(200.0), 2)
        self.feed("EV,2,400,201,1,")          # 补发回来的旧事件
        self.feed("EV,2,400,201,1,")          # 重复
        session = self.tracker.current
        self.assertEqual([e.seq for e in self.tracker.timeline(session)], [1, 2, 3])
        self.assertIsNone(self.tracker.resend_request(300.0))

    def test_status_reveals_missing(self):
        self.tracker.handle_line(status(1000, 5), 10.0)
        self.assertEqual(self.tracker.resend_request(11.0), 1)

    def test_reboot_by_time(self):
        self.tracker.handle_line(status(50000, 4), 10.0)
        self.feed("EV,1,0,301,8,")
        self.tracker.handle_line(status(300, 2), 11.0)    # 板上时间倒退 → 新会话
        self.assertEqual(len(self.tracker.sessions), 2)

    def test_reboot_by_conflicting_seq(self):
        self.feed("EV,1,0,301,8,")
        self.feed("EV,2,900,302,300,")
        self.feed("EV,2,15,213,4,")                   # 同序号不同内容
        self.assertEqual(len(self.tracker.sessions), 2)

    def test_lost_events(self):
        self.feed("EV,200,9000,201,0,")
        self.feed("EV,201,9001,211,73,")              # 下位机：73 之前已覆盖
        self.assertEqual(self.tracker.current.first_missing(), 73)

    def test_note_estimates_board_time(self):
        self.tracker.handle_line(status(10000, 0), 100.0)
        note = self.tracker.add_note("发送 ON", pc_time=100.5)
        self.assertEqual(note.board_ms_estimate, 10500)


@unittest.skipUnless(os.environ.get("FINS_HARNESS"), "未设置 FINS_HARNESS")
class FirmwareHarnessTest(unittest.TestCase):
    """用真实固件 EventLog.cpp 产生的字节流做端到端测试。"""

    def run_harness(self, script: str, timeout=10.0, kill_after_ft=0) -> bytes:
        proc = subprocess.Popen([os.environ["FINS_HARNESS"]], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        if not kill_after_ft:
            out, _ = proc.communicate(script.encode(), timeout=timeout)
            return out
        proc.stdin.write(script.encode())
        proc.stdin.flush()
        data = bytearray()
        deadline = time.time() + timeout
        while data.count(b"$FT") < kill_after_ft and time.time() < deadline:
            chunk = proc.stdout.read1(4096)
            if not chunk:
                break
            data.extend(chunk)
        proc.kill()
        return bytes(data)

    def test_clean_stream(self):
        script = "POST 301 8\nPOST 213 3\nPOST 302 300\nSTEP 30\nQUIT\n"
        tracker = EventTracker()
        for line in fins_protocol.LineSplitter().feed(self.run_harness(script)):
            tracker.handle_line(line, 1.0)
        session = tracker.current
        self.assertEqual(sorted(session.events), [1, 2, 3])
        self.assertIsNotNone(session.last_status)
        self.assertEqual(tracker.checksum_errors, 0)
        plots = [fins_protocol.parse_line(l, 1.0) for l in fins_protocol.LineSplitter().feed(self.run_harness(script))]
        self.assertTrue(any(isinstance(p, fins_protocol.PlotSample) for p in plots))

    def test_lossy_link_recovers_by_resend(self):
        # 链路丢 1% 字节：靠 EVQ 补发，最终 60 条事件一条不少
        tracker = EventTracker()
        posts = "".join(f"POST {201 + i % 16} {i}\n" for i in range(60))
        script = "DROP 0.01\n" + posts + "STEP 40\n"
        out = self.run_harness(script + "QUIT\n")
        for line in fins_protocol.LineSplitter().feed(out):
            tracker.handle_line(line, 1.0)
        rounds = 0
        # 实际运行中状态行 10 Hz 持续到来，上位机迟早知道最新序号；这里每轮都补一次
        while len(tracker.current.events) < 60 and rounds < 20:
            rounds += 1
            seq = tracker.current.first_missing() or tracker.current.max_seq_seen + 1
            # 真实链路的丢字节是随机的；模拟程序每轮换一个随机种子
            out = self.run_harness(f"SEED {rounds}\n" + script + f"EVQ {seq}\nSTEP 40\nQUIT\n")
            for line in fins_protocol.LineSplitter().feed(out):
                tracker.handle_line(line, 2.0 + rounds)
        self.assertGreater(tracker.checksum_errors, 0, "测试应当真的丢了数据")
        self.assertEqual(sorted(tracker.current.events), list(range(1, 61)))

    def test_late_connect_after_log_wrapped(self):
        # 下位机已记了 300 条事件（环形缓冲只留 128 条）后上位机才请求从第 1 条补发：
        # 原来"先算最早序号再记 211"会让发送循环空转，一条事件也发不出来
        posts = "".join(f"POST 221 {i}\n" for i in range(300))
        out = self.run_harness(posts + "STEP 200\nEVQ 1\nPOST 303 8\nSTEP 200\nQUIT\n")
        tracker = EventTracker()
        for line in fins_protocol.LineSplitter().feed(out):
            tracker.handle_line(line, 1.0)
        session = tracker.current
        self.assertGreater(len(session.events), 100)
        self.assertIn(303, [e.code for e in session.events.values()])
        self.assertEqual(session.missing(), [])

    def test_fatal_loop(self):
        out = self.run_harness("POST 301 8\nPOST 302 300\nSTEP 5\nFATAL\n", kill_after_ft=2)
        tracker = EventTracker()
        for line in fins_protocol.LineSplitter().feed(out):
            tracker.handle_line(line, 1.0)
        session = tracker.current
        self.assertIsNotNone(session.fatal)
        self.assertEqual(session.fatal.code, 901)
        fatal_event = session.events[session.fatal.seq]
        self.assertEqual(fatal_event.code, 901)
        self.assertIn("CFSR=", fatal_event.text)
        self.assertEqual(fatal_event.arg, 0x08001234)

    def test_assert_reports_file_and_line(self):
        out = self.run_harness("STEP 2\nASSERT\n", kill_after_ft=1)
        tracker = EventTracker()
        for line in fins_protocol.LineSplitter().feed(out):
            tracker.handle_line(line, 1.0)
        event = tracker.current.events[tracker.current.fatal.seq]
        self.assertEqual((event.code, event.arg, event.text), (906, 123, "queue.c"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
