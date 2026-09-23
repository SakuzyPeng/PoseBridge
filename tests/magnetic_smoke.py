"""Exercise the real C ABI and CLI through a fake WIT serial device; no hardware writes.

Uses the existing release artifacts and Unix PTYs. Windows runs the portable Rust
model tests and C ABI contract checks; native USB/BLE measurements are separate.
"""
import ctypes as c
import json
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import sys
import threading
import time

if os.name == "nt":
    print("Magnetic PTY integration: skipped (Unix PTY required)")
    sys.exit(0)

import tty
from hardware_smoke import Bridge

ROOT = Path(__file__).resolve().parents[1]
LIBRARY = ROOT / "target/release" / ("libposebridge_capi.dylib" if sys.platform == "darwin" else "libposebridge_capi.so")
CLI = ROOT / "target/release/posebridge"


def until(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    raise AssertionError("timed out waiting for magnetic state")


class Sensor:
    def __init__(self, sensor_type=6):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)
        self.sensor_type = sensor_type
        self.calsw = 0
        self.ignore_stop = False
        self.commands = []
        self.reads = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        pending = bytearray()
        while not self.stop.is_set():
            if not select.select([self.master], [], [], 0.05)[0]:
                continue
            pending.extend(os.read(self.master, 1024))
            while len(pending) >= 5:
                command = bytes(pending[:5])
                del pending[:5]
                self.commands.append(command)
                address, value = command[2], int.from_bytes(command[3:5], "little")
                if address == 1 and (value or not self.ignore_stop):
                    self.calsw = value
                if address != 0x27:
                    continue
                words = [0] * 8
                if value == 0x72:
                    words[0] = self.sensor_type
                elif value == 1:
                    words[0] = self.calsw
                elif value == 0x3A:
                    self.reads += 1
                    words[:3] = [120 + self.reads, -240, 60]
                response = bytes([0x55, 0x61]) + bytes(18) + struct.pack("<BBH8h", 0x55, 0x71, value, *words)
                os.write(self.master, response[:23])
                os.write(self.master, response[23:])

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)

    def writes(self, address, value):
        return self.commands.count(bytes([0xFF, 0xAA, address]) + value.to_bytes(2, "little"))


class MagneticBridge(Bridge):
    def __init__(self, sensor):
        super().__init__(LIBRARY)
        self.lib.pb_magnetic_start.argtypes = [c.c_void_p]
        self.lib.pb_magnetic_start.restype = c.c_int
        self.lib.pb_magnetic_since_json.argtypes = [c.c_void_p, c.c_char_p, c.c_uint32, c.c_void_p, c.c_uint32, c.POINTER(c.c_uint32)]
        self.lib.pb_magnetic_since_json.restype = c.c_int
        self.configure({"source_id": "mag-test", "source": {"kind": "usb", "port": sensor.port, "baud": 115200}})

    def close(self):
        if self.ctx:
            rc = self.lib.pb_context_destroy(self.ctx)
            self.ctx = c.c_void_p()  # destroy consumes the handle even on error
            assert rc == 0, ("destroy", rc)

    def batch(self, cursor=None):
        data = json.dumps(cursor).encode() if cursor is not None else None
        required = c.c_uint32()
        fn = self.lib.pb_magnetic_since_json
        assert fn(self.ctx, data, len(data) if data else 0, None, 0, c.byref(required)) == 9
        for _ in range(8):
            buffer = c.create_string_buffer(required.value + 4096)
            rc = fn(self.ctx, data, len(data) if data else 0, buffer, len(buffer), c.byref(required))
            if rc == 0:
                return json.loads(buffer.value)
            assert rc == 9
        raise AssertionError("magnetic buffer failed to stabilize")

    def start(self):
        assert self.lib.pb_magnetic_start(self.ctx) == 0
        until(lambda: self.batch()["active"])

    def command(self, action, wait=True):
        data = json.dumps({"action": action}).encode()
        rc = self.lib.pb_device_command(self.ctx, data, len(data))
        if not wait:
            return rc
        assert rc == 0, (rc, self.batch())
        return until(lambda: (lambda op: op if op and op["outcome"] != "running" else None)(self.batch()["operation"]))


def check_c_api():
    sensor = Sensor()
    bridge = MagneticBridge(sensor)
    try:
        bridge.start()
        first = bridge.batch()
        assert first["schema"] == 1 and first["latest"]["field_ut"][1] == -2
        assert first["latest"]["register_xyz"][1] == -240
        assert bridge.snapshot()["pose"] is None and bridge.snapshot()["status"]["pose_count"] == "0"
        assert bridge.lib.pb_magnetic_start(bridge.ctx) == 3
        assert bridge.lib.pb_inspect_start(bridge.ctx) == 3
        assert bridge.command("accel_calibrate", wait=False) == 3
        assert bridge.command("mag_start", wait=False) == 0
        assert bridge.command("mag_start", wait=False) == 3
        until(lambda: bridge.batch()["phase"] == "calibrating")
        started = bridge.batch()
        assert started["statistics"]["window_id"] != first["statistics"]["window_id"]
        assert started["operation"]["register_verified"]
        assert bridge.command("save")["outcome"] == "failed"
        assert sensor.writes(0, 0) == 0
        until(lambda: int(bridge.batch()["statistics"]["sample_count"]) >= 2)
        assert bridge.command("mag_stop")["completion_observed"]
        stopped = bridge.batch()
        until(lambda: int(bridge.batch()["cursor"]["sequence"]) > int(stopped["cursor"]["sequence"]))
        assert bridge.batch()["statistics"] == stopped["statistics"]
        assert bridge.command("save")["persistence"] == "unverified"
        assert sensor.writes(0, 0) == 1
        assert bridge.lib.pb_stop(bridge.ctx) == 0
        closed = bridge.batch()
        assert closed["phase"] == "closed" and not closed["active"]
        assert all(not s["fresh"] for s in closed["samples"])
        assert bridge.batch(closed["cursor"])["samples"] == []
        before = closed["latest"]["age_ns"]
        time.sleep(0.02)
        assert int(bridge.batch()["latest"]["age_ns"]) > int(before)
        bridge.start()
        assert bridge.batch(closed["cursor"])["reset"]
        bridge.command("mag_start")
        assert bridge.lib.pb_stop(bridge.ctx) == 0
        assert bridge.batch()["cleanup"]["outcome"] == "succeeded"
        assert sensor.writes(0, 0) == 1
        bridge.start()
        bridge.command("mag_start")
        sensor.ignore_stop = True
        assert bridge.lib.pb_stop(bridge.ctx) != 0
        failed = bridge.batch()
        assert failed["phase"] == "failed" and not failed["active"]
        assert failed["device_may_be_calibrating"] and failed["cleanup"]["outcome"] == "failed"
        assert "calibrating" in failed["last_error"]
    finally:
        bridge.close()
        sensor.close()

    sensor = Sensor(sensor_type=1)
    bridge = MagneticBridge(sensor)
    try:
        bridge.start()
        assert bridge.batch()["latest"]["field_ut"] is None
        assert bridge.batch()["type_error"]
        bridge.command("mag_start")
        bridge.close()  # Destruction also performs bounded cleanup.
        assert sensor.writes(1, 0) == 1 and sensor.writes(0, 0) == 0
    finally:
        bridge.close()
        sensor.close()


def check_cli():
    for action, save in [("monitor", False), ("calibrate", False), ("calibrate", True)]:
        sensor = Sensor()
        try:
            command = [str(CLI), "magnetic", action, "--transport", "usb", "--port", sensor.port, "--duration", "0.5", "--json"]
            if save:
                command.append("--save")
            result = subprocess.run(command, capture_output=True, text=True, timeout=10)
            assert result.returncode == 0, (result.stdout, result.stderr)
            records = [json.loads(line) for line in result.stdout.splitlines()]
            assert records[-1]["phase"] == "closed" and not records[-1]["active"]
            assert any(record["samples"] for record in records)
            assert sensor.writes(1, 7) == int(action == "calibrate")
            assert sensor.writes(0, 0) == int(save)
            if action == "monitor":
                assert all(command[2] == 0x27 for command in sensor.commands)
        finally:
            sensor.close()
    sensor = Sensor()
    process = subprocess.Popen([str(CLI), "magnetic", "calibrate", "--transport", "usb", "--port", sensor.port,
                                "--duration", "60", "--save", "--json"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        until(lambda: sensor.calsw == 7)
        process.send_signal(signal.SIGINT)
        output, error = process.communicate(timeout=6)
        assert process.returncode == 130, (output, error)
        final = json.loads(output.splitlines()[-1])
        assert final["phase"] == "closed" and final["cleanup"]["outcome"] == "succeeded"
        assert sensor.writes(1, 0) == 1 and sensor.writes(0, 0) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        sensor.close()


if __name__ == "__main__":
    if sys.platform == "darwin" and os.environ.get("POSEBRIDGE_PTY_SHIM") != "1":
        shim = ROOT / "target/magnetic-pty-test.dylib"
        subprocess.run(["cc", "-dynamiclib", "-Wall", "-Wextra", "-Werror",
                        str(ROOT / "tests/magnetic_pty_macos.c"), "-o", str(shim)], check=True)
        environment = dict(os.environ, POSEBRIDGE_PTY_SHIM="1", DYLD_INSERT_LIBRARIES=str(shim))
        sys.exit(subprocess.call([sys.executable, "-B", __file__], env=environment))
    check_c_api()
    check_cli()
    print("Magnetic C ABI + CLI through fake serial: PASS")
