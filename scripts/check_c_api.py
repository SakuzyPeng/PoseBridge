"""Compile a real C caller against a relocated copy of the shared library."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
artifacts = root / "target/release"
windows = sys.platform == "win32"
library = "posebridge.dll" if windows else "libposebridge.dylib" if sys.platform == "darwin" else "libposebridge.so"
with tempfile.TemporaryDirectory(prefix="c-abi-relocation-", dir=root / "target") as location:
    stage = Path(location)
    shutil.copy2(artifacts / library, stage / library)
    binary = stage / ("smoke.exe" if windows else "smoke")
    source = root / "tests/c_api_smoke.c"
    if windows:
        command = ["cl", "/nologo", "/W4", f"/I{root / 'include'}", str(source),
                   f"/Fe{binary}", f"/Fo{stage / 'smoke.obj'}", "/link", str(artifacts / "posebridge.dll.lib")]
    else:
        runtime_path = "@loader_path" if sys.platform == "darwin" else "$ORIGIN"
        command = ["cc", "-std=c11", f"-I{root / 'include'}", str(source), f"-L{stage}",
                   "-lposebridge", f"-Wl,-rpath,{runtime_path}", "-o", str(binary)]
    subprocess.run(command, cwd=stage, check=True)
    if sys.platform == "darwin":
        load_commands = subprocess.check_output(["otool", "-L", str(binary)], text=True)
        assert "@rpath/libposebridge.dylib" in load_commands, load_commands
        assert str(artifacts) not in load_commands, load_commands
    subprocess.run([str(binary)], cwd=stage, check=True)
print("Relocated C ABI consumer: PASS")
