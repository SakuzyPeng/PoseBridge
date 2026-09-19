"""Copy the cbindgen artifact from this workspace's single target directory."""
from pathlib import Path
import argparse
import shutil

parser = argparse.ArgumentParser()
parser.add_argument("--profile", choices=["debug", "release"], default="release")
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
headers = list((root / "target" / args.profile / "build").glob("posebridge-capi-*/out/posebridge.h"))
if not headers:
    raise SystemExit("Build posebridge-capi before exporting its header")
header = max(headers, key=lambda p: p.stat().st_mtime_ns)
target = root / "include" / "posebridge.h"
target.parent.mkdir(exist_ok=True)
generated = header.read_bytes()
# Preserve an equivalent CRLF checkout on Windows; only API changes should dirty the tree.
if not target.exists() or target.read_bytes().replace(b"\r\n", b"\n") != generated.replace(b"\r\n", b"\n"):
    shutil.copyfile(header, target)
print(target)
