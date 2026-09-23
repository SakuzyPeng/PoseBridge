"""Explicit read-only magnetic hardware measurement through the release CLI.

Never calibrates or saves. Keep only compact aggregate output, without local
device IDs. This validates reception and unit conversion, not calibration accuracy.
"""
import argparse
import json
import math
from pathlib import Path
import platform
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--cli", required=True)
parser.add_argument("--transport", choices=["usb", "ble"], required=True)
parser.add_argument("--port")
parser.add_argument("--device")
parser.add_argument("--seconds", type=float, default=30)
parser.add_argument("--output", type=Path)
args = parser.parse_args()
command = [args.cli, "magnetic", "monitor", "--transport", args.transport, "--duration", str(args.seconds), "--json"]
command += ["--port", args.port] if args.transport == "usb" else ["--device", args.device]
assert command[-1], "provide --port or --device"
result = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", timeout=args.seconds + 45)
assert result.returncode == 0, result.stderr
records = [json.loads(line) for line in result.stdout.splitlines()]
final = records[-1]
samples = [sample for record in records for sample in record["samples"]]
assert len(samples) >= 2 and final["phase"] == "closed" and not final["active"]
assert final["last_error"] is None and not final["latest"]["fresh"]
assert all(record["operation"] is None and record["cleanup"] is None and record["history_overrun"] == "0" for record in records)
for index, sample in enumerate(samples, 1):
    assert int(sample["cursor"]["sequence"]) == index
    if final["scale_ut_per_count"] is not None:
        assert all(math.isclose(value, raw * final["scale_ut_per_count"], abs_tol=1e-12)
                   for raw, value in zip(sample["register_xyz"], sample["field_ut"]))
    else:
        assert sample["field_ut"] is None
report = {
    "platform": platform.system(), "transport": args.transport, "duration_seconds": args.seconds,
    "sensor_type": final["sensor_type"], "scale_ut_per_count": final["scale_ut_per_count"],
    "calsw_before": records[0]["calsw"], "calsw_after": final["calsw"],
    "samples_delivered": len(samples), "statistics": final["statistics"],
    "stopped_fresh": final["latest"]["fresh"], "history_overrun": 0,
    "type_error": final["type_error"], "calibration_exercised": False, "save_sent": False,
    "result": "pass",
}
report["statistics"].pop("window_id")
text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
if args.output:
    args.output.write_text(text, encoding="utf-8")
print(text, end="")
