"""Measure a Release CLI against an independent loopback OSC receiver; no device writes.

Example: python scripts/measure_osc.py -- simulate --sample-rate-hz 200
         --osc-rate-hz 200 --duration 15
Device rate changes, if needed, must be performed and restored explicitly by the operator.
"""

import argparse
import json
import math
import os
import socket
import statistics
import struct
import subprocess
import threading
import time
from pathlib import Path


def decode(data):
    def string(offset):
        end = data.index(0, offset)
        return data[offset:end].decode("ascii"), (end + 4) & ~3

    address, offset = string(0)
    tags, offset = string(offset)
    expected = {
        "/posebridge/v1/quaternion": ",ffff",
        "/posebridge/v1/euler": ",fff",
    }
    if address not in expected or tags != expected[address]:
        raise ValueError("unexpected OSC address/types")
    values = struct.unpack(">" + "f" * (len(tags) - 1), data[offset:])
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite pose")
    if len(values) == 4 and abs(sum(value * value for value in values) - 1) > 1e-5:
        raise ValueError("non-unit quaternion")
    return values


def measure(exe, arguments, warmup, timeout):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        sock.settimeout(0.1)
        command = [str(exe)] + arguments + [
            "--json", "--osc-target", "127.0.0.1:" + str(sock.getsockname()[1])
        ]
        packets, errors = [], []
        stopped = threading.Event()

        def receive():
            while not stopped.is_set():
                try:
                    data = sock.recv(512)
                except socket.timeout:
                    continue
                now = time.perf_counter()
                try:
                    decode(data)
                    packets.append(now)
                except (ValueError, UnicodeError, struct.error) as error:
                    errors.append(str(error))

        thread = threading.Thread(target=receive)
        thread.start()
        try:
            process = subprocess.run(command, capture_output=True, timeout=timeout)
        finally:
            stopped.set()
            thread.join()

    rows = [json.loads(line) for line in process.stdout.decode().splitlines() if line.startswith("{")]
    poses = [row for row in rows if row.get("pose") and row["pose"]["fresh"]]
    result = {
        "exit": process.returncode,
        "stderr": process.stderr.decode(errors="replace").strip(),
        "invalid_packets": errors,
        "osc_packets": len(packets),
        "warmup_seconds": warmup,
        "last_status": rows[-1]["status"] if rows else None,
    }
    if poses:
        session = poses[-1]["pose"]["session_id"]
        same_session = [row for row in poses if row["pose"]["session_id"] == session]
        first = same_session[0]["pose"]["received_ns"]
        settled = [row["pose"] for row in same_session if row["pose"]["received_ns"] - first >= warmup * 1e9]
        if len(settled) >= 2 and settled[-1]["received_ns"] > settled[0]["received_ns"]:
            span = (settled[-1]["received_ns"] - settled[0]["received_ns"]) / 1e9
            result["input_hz"] = (settled[-1]["sequence"] - settled[0]["sequence"]) / span
    if packets:
        stable = [stamp for stamp in packets if stamp >= packets[0] + warmup]
        if len(stable) >= 2:
            gaps = sorted((b - a) * 1000 for a, b in zip(stable, stable[1:]))
            result["osc_hz"] = (len(stable) - 1) / (stable[-1] - stable[0])
            result["osc_gaps"] = {
                "p50_ms": statistics.median(gaps),
                "p95_ms": gaps[min(len(gaps) - 1, int(len(gaps) * 0.95))],
                "max_ms": gaps[-1],
            }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    binary = "posebridge.exe" if os.name == "nt" else "posebridge"
    parser.add_argument("--exe", type=Path, default=Path(__file__).resolve().parents[1] / "target/release" / binary)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--warmup", type=float, default=3)
    parser.add_argument("--timeout", type=float, default=45)
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    options = parser.parse_args()
    arguments = options.arguments
    if arguments[:1] == ["--"]:
        arguments = arguments[1:]
    if not arguments or arguments[0] not in ("simulate", "bridge"):
        parser.error("supply -- simulate/bridge with a finite --duration")
    if "--duration" not in arguments:
        parser.error("--duration is required")
    try:
        duration = float(arguments[arguments.index("--duration") + 1])
    except (ValueError, IndexError):
        parser.error("--duration needs a positive number")
    if not math.isfinite(duration) or duration <= 0:
        parser.error("--duration must be finite and positive")
    if not math.isfinite(options.warmup) or not 0 <= options.warmup < duration:
        parser.error("--warmup must be finite, nonnegative and shorter than --duration")
    if not math.isfinite(options.timeout) or options.timeout <= duration:
        parser.error("--timeout must be finite and longer than --duration")
    result = measure(options.exe, arguments, options.warmup, options.timeout)
    text = json.dumps(result, indent=2)
    if options.output:
        options.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["exit"] == 0 and not result["invalid_packets"] and result.get("osc_hz", 0) > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
