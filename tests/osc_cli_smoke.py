"""Exercise the shipped CLI and decode OSC independently of the Rust OSC library."""
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import sys
import threading
import time

root = Path(__file__).resolve().parents[1]
binary = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "target/release/posebridge"
if sys.platform == "win32" and binary.suffix != ".exe":
    binary = binary.with_suffix(".exe")


def decode(data):
    def string(offset):
        end = data.index(0, offset)
        return data[offset:end].decode(), (end + 4) & ~3
    address, pos = string(0)
    tags, pos = string(pos)
    count = len(tags) - 1
    assert tags in (",fff", ",ffff")
    assert len(data) == pos + 4 * count
    return address, struct.unpack_from(">" + "f" * count, data, pos)


for fmt, expected in [
    ("euler", (30, 20, 10)),
    ("quaternion", (0.189307857, 0.239298338, 0.038134576, 0.951548525)),
]:
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("127.0.0.1", 0))
    udp.settimeout(0.05)
    packets = []
    stop = threading.Event()

    def receive():
        while True:
            try:
                packets.append(udp.recv(1024))
            except socket.timeout:
                if stop.is_set():
                    return

    reader = threading.Thread(target=receive)
    reader.start()
    try:
        started = time.perf_counter()
        run = subprocess.run([
            str(binary), "simulate", "--yaw", "30", "--pitch", "20", "--roll", "10",
            "--format", fmt, "--sample-rate-hz", "100", "--osc-rate-hz", "50",
            "--osc-target", f"127.0.0.1:{udp.getsockname()[1]}", "--duration", "1", "--json",
        ], capture_output=True, text=True, timeout=10)
        elapsed = time.perf_counter() - started
    finally:
        stop.set()
        reader.join()
        udp.close()
    assert run.returncode == 0, (run.stdout, run.stderr)
    # CLI duration is checked by its control thread. A loaded CI runner can
    # schedule that thread late while acquisition continues at the correct rate.
    # Bound packets by elapsed monotonic time, including the initial send slot.
    assert 25 <= len(packets) <= math.ceil(elapsed * 50) + 2, (len(packets), elapsed)
    rows = [json.loads(line) for line in run.stdout.splitlines()]
    assert any(row["pose"] and row["pose"]["fresh"] for row in rows)
    for packet in packets:
        address, values = decode(packet)
        assert address == f"/posebridge/v1/{fmt}"
        assert all(abs(a-b) < 1e-5 for a, b in zip(values, expected)), values
    print(f"CLI -> OSC {fmt}: PASS ({len(packets)} complete packets in {elapsed:.3f}s)")
