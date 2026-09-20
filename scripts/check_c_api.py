"""Compile and exercise C ABI callers against a relocated shared library."""
import json
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
artifacts = root / "target/release"
windows = sys.platform == "win32"
library = "posebridge_capi.dll" if windows else "libposebridge_capi.dylib" if sys.platform == "darwin" else "libposebridge_capi.so"
with tempfile.TemporaryDirectory(prefix="c-abi-relocation-", dir=root / "target") as location:
    stage = Path(location)
    shutil.copy2(artifacts / library, stage / library)

    def compile_c(source, name, link_library=True):
        binary = stage / (name + (".exe" if windows else ""))
        if windows:
            command = ["cl", "/nologo", "/std:c11", "/W4", "/WX", f"/I{root / 'include'}", str(source),
                       f"/Fe{binary}", f"/Fo{stage / (name + '.obj')}"]
            if link_library:
                command += ["/link", str(artifacts / "posebridge_capi.dll.lib")]
        else:
            runtime_path = "@loader_path" if sys.platform == "darwin" else "$ORIGIN"
            command = ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", f"-I{root / 'include'}",
                       str(source), "-o", str(binary)]
            if link_library:
                command += [f"-L{stage}", "-lposebridge_capi", f"-Wl,-rpath,{runtime_path}"]
        subprocess.run(command, cwd=stage, check=True)
        if link_library and sys.platform == "darwin":
            load_commands = subprocess.check_output(["otool", "-L", str(binary)], text=True)
            assert "@rpath/libposebridge_capi.dylib" in load_commands, load_commands
            assert str(artifacts) not in load_commands, load_commands
        return binary

    smoke = compile_c(root / "tests/c_api_smoke.c", "smoke")
    subprocess.run([str(smoke)], cwd=stage, check=True, timeout=10)
    policy = compile_c(root / "tests/consumer_policy_test.c", "consumer_policy_test", False)
    subprocess.run([str(policy)], cwd=stage, check=True, timeout=5)
    consumer = compile_c(root / "examples/pose_consumer.c", "pose_consumer")

    def final_snapshot(output):
        value = [json.loads(line) for line in output.splitlines() if line.startswith("{")][-1]
        assert value["schema"] == 4 and value["status"]["state"] == "stopped", value
        assert value["pose"] is None or not value["pose"]["fresh"], value
        return value

    def run_consumer(arguments, expected=0):
        result = subprocess.run([str(consumer), *arguments], cwd=stage, capture_output=True,
                                text=True, encoding="utf-8", timeout=12)
        assert result.returncode == expected, (result.returncode, result.stdout, result.stderr)
        final = final_snapshot(result.stdout)
        return result, final

    default, final = run_consumer([])
    assert final["descriptor"]["transport"] == "simulate"
    assert final["descriptor"]["application_config"]["osc"] is None
    assert "ACTIVE" in default.stdout and "REFERENCE" in default.stdout
    assert int(final["status"]["pose_count"]) > 0

    config_file = stage / "consumer-config.json"
    config_file.write_text(json.dumps({"source": {"kind": "simulate", "rate_hz": 1}}), encoding="utf-8")
    slow, final = run_consumer(["--config", str(config_file), "--duration", "2.3", "--max-age-ms", "100"])
    assert slow.stdout.count("ACTIVE") >= 2 and slow.stdout.count("FROZEN") >= 2, slow.stdout
    assert slow.stdout.count("REFERENCE") == 1
    assert int(final["pose"]["age_ns"]) > 0

    config_file.write_text('{"source":{"kind":"simulate","rate_hz":0}}', encoding="utf-8")
    invalid, final = run_consumer(["--config", str(config_file)], expected=1)
    assert "configure failed" in invalid.stderr and final["pose"] is None

    # Valid initial angles; the first animated sample exceeds the Euler input bound.
    config_file.write_text(json.dumps({"source": {"kind": "simulate", "pattern": "yaw",
                                                  "euler_deg": [1000000, 0, 0]}}), encoding="utf-8")
    failed, final = run_consumer(["--config", str(config_file)], expected=1)
    failures = [json.loads(line) for line in failed.stderr.splitlines() if line.startswith("{")]
    assert failures and failures[-1]["status"]["state"] == "failed", failed.stderr

    if not windows:
        process = subprocess.Popen([str(consumer), "--duration", "30"], cwd=stage,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, encoding="utf-8")
        try:
            assert select.select([process.stdout], [], [], 5)[0], "consumer did not start"
            assert process.stdout.readline().strip() == "READY"
            process.send_signal(signal.SIGINT)
            output, error = process.communicate(timeout=5)
            assert process.returncode == 130, (output, error)
            final_snapshot(output)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
    # Keep only the usable example next to the existing release library.
    shutil.copy2(consumer, artifacts / consumer.name)
print("Relocated C ABI and C11 consumer (default/slow/error/cleanup): PASS")
