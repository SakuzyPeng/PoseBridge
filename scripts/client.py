#!/usr/bin/env python3
"""Build/package the native client using the caller's existing Cargo caches."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import plistlib
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    return subprocess.run([str(x) for x in command], check=True, **kwargs)


def target_directory(value):
    target = Path(value).expanduser().resolve()
    if sys.platform == 'darwin' and target.parts[:2] == ('/', 'Volumes'):
        volume = Path(*target.parts[:3])
        if not volume.is_mount() or not target.is_dir():
            raise SystemExit(f'Build volume is not mounted, or target is missing: {target}')
    return target


def notices(cargo, destination, platform):
    metadata = json.loads(subprocess.check_output(
        [cargo, 'metadata', '--locked', '--format-version', '1', '--filter-platform', platform], cwd=ROOT))
    packages = {p['id']: p for p in metadata['packages']}
    nodes = {p['id']: p for p in metadata['resolve']['nodes']}
    start = next(k for k, p in packages.items() if p['name'] == 'posebridge-client')
    seen, pending = set(), [start]
    while pending:
        key = pending.pop()
        if key in seen:
            continue
        seen.add(key)
        pending.extend(d['pkg'] for d in nodes[key]['deps']
                       if any(k['kind'] != 'dev' for k in d['dep_kinds']))
    entries = []
    for key in sorted(seen, key=lambda k: (packages[k]['name'], packages[k]['version'])):
        p = packages[key]
        folder = Path(p['manifest_path']).parent
        files = sorted({f for pattern in ['LICENSE*', 'LICENCE*', 'COPYING*', 'NOTICE*']
                        for f in folder.glob(pattern) if f.is_file()})
        if p.get('license_file'):
            files.append(folder / p['license_file'])
        text = '\n'.join(f.read_text(encoding='utf-8', errors='replace') for f in files)
        entries.append(f"{p['name']} {p['version']}\nLicense: {p.get('license') or 'see license file'}\n{p.get('repository') or ''}\n{text}")
    (destination / 'ThirdPartyNotices.txt').write_text('\n\n'.join(entries), encoding='utf-8')
    shutil.copy2(ROOT / 'LICENSE', destination / 'LICENSE.txt')
    shutil.copy2(ROOT / 'crates/posebridge-client/assets/OFL.txt', destination / 'FONT-LICENSE.txt')
    shutil.copy2(ROOT / 'crates/posebridge-client/assets/PLAYER-LICENSE.txt', destination / 'PLAYER-LICENSE.txt')
    shutil.copy2(ROOT / 'docs/client.md', destination / '使用说明.md')


def verify(binary, directory, smoke):
    with tempfile.TemporaryDirectory(prefix='client-check-', dir=directory) as data:
        data = Path(data)
        for flag, name in [('--check-install', 'install-check.json')] + ([('--smoke-test', 'smoke-report.json')] if smoke else []):
            run([binary, flag, '--data-dir', data], timeout=45)
            report = json.loads((data / name).read_text(encoding='utf-8'))
            if report['hardware_accessed']:
                raise SystemExit('A startup check accessed hardware')
            if flag == '--smoke-test' and (report['frames'] < 6 or report['tabs'] != 6):
                raise SystemExit('Incomplete window smoke check')
    if sys.platform == 'darwin':
        architecture = subprocess.check_output(['lipo', '-archs', binary], text=True).strip()
        if architecture != 'arm64':
            raise SystemExit(f'Expected an Apple Silicon executable, got {architecture}')
        links = subprocess.check_output(['otool', '-L', binary], text=True)
        for line in links.splitlines()[1:]:
            dep = line.strip().split(' (', 1)[0]
            if not dep.startswith(('/usr/lib/', '/System/Library/')):
                raise SystemExit(f'Non-system runtime dependency: {dep}')
    else:
        dumpbin = shutil.which('dumpbin') or shutil.which('dumpbin.exe')
        if not dumpbin:
            raise SystemExit('Packaging verification requires the MSVC dumpbin tool on PATH')
        headers = subprocess.check_output([dumpbin, '/headers', binary]).decode(errors='replace')
        if not re.search(r'8664 machine', headers, re.IGNORECASE):
            raise SystemExit('Expected a Windows x64 executable')
        links = subprocess.check_output([dumpbin, '/dependents', binary]).decode(errors='replace')
        system = set('kernel32 user32 gdi32 gdi32full dwmapi advapi32 ole32 oleaut32 shell32 shlwapi comctl32 comdlg32 secur32 ws2_32 ntdll winmm rpcrt4 setupapi bthprops runtimeobject powrprof uxtheme d3d11 d3d12 dxgi d3dcompiler_47 bcrypt imm32 version msvcrt usp10 winhttp wsock32 crypt32 propsys hid windowsapp cfgmgr32 uiautomationcore opengl32 bcryptprimitives'.split())
        for name in re.findall(r'^\s*([A-Za-z0-9_.-]+)\.dll\s*$', links, re.MULTILINE | re.IGNORECASE):
            if name.lower() not in system and not name.lower().startswith(('api-ms-', 'ext-ms-')):
                raise SystemExit(f'Non-system runtime dependency: {name}.dll')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['build', 'package'])
    parser.add_argument('--target-dir', default=os.environ.get('CARGO_TARGET_DIR', ROOT / 'target'))
    parser.add_argument('--output', type=Path, default=ROOT / 'dist')
    parser.add_argument('--cargo', default='cargo')
    parser.add_argument('--skip-build', action='store_true')
    parser.add_argument('--smoke', action='store_true', help='Also open and close all six pages without hardware access')
    args = parser.parse_args()
    if sys.platform not in ('darwin', 'win32'):
        raise SystemExit('Desktop packages currently support macOS and Windows')
    target = target_directory(args.target_dir)
    env = dict(os.environ, CARGO_TARGET_DIR=str(target))
    env.setdefault('MACOSX_DEPLOYMENT_TARGET', '14.0')
    if not args.skip_build:
        run([args.cargo, 'build', '-p', 'posebridge-client', '--release', '--locked'], cwd=ROOT, env=env)
    if args.action == 'build':
        return
    version = json.loads(subprocess.check_output([args.cargo, 'metadata', '--no-deps', '--format-version', '1'], cwd=ROOT))['packages']
    version = next(p['version'] for p in version if p['name'] == 'posebridge-client')
    platform = 'aarch64-apple-darwin' if sys.platform == 'darwin' else 'x86_64-pc-windows-msvc'
    args.output.mkdir(parents=True, exist_ok=True)
    package = args.output / ('PoseBridge.app' if sys.platform == 'darwin' else 'PoseBridge-windows-x64')
    if package.exists():
        raise SystemExit(f'Choose a fresh output directory; preserving {package}')
    source = target / 'release' / ('posebridge-client.exe' if sys.platform == 'win32' else 'posebridge-client')
    if sys.platform == 'darwin':
        binary = package / 'Contents/MacOS/posebridge-client'
        resources = package / 'Contents/Resources'
        binary.parent.mkdir(parents=True)
        resources.mkdir()
        plist = dict(CFBundleIdentifier='org.posebridge.client', CFBundleName='PoseBridge', CFBundleDisplayName='PoseBridge',
                     CFBundleExecutable='posebridge-client', CFBundlePackageType='APPL', CFBundleVersion=version,
                     CFBundleShortVersionString=version, LSMinimumSystemVersion='14.0', NSHighResolutionCapable=True,
                     NSBluetoothAlwaysUsageDescription='PoseBridge reads orientation and magnetic data from the Bluetooth sensor you select.')
        (package / 'Contents/Info.plist').write_bytes(plistlib.dumps(plist))
    else:
        package.mkdir()
        binary = package / source.name
        resources = package
    shutil.copy2(source, binary)
    notices(args.cargo, resources, platform)
    if sys.platform == 'darwin':
        run(['codesign', '--force', '--sign', '-', package])
        run(['codesign', '--verify', '--deep', '--strict', package])
    verify(binary.resolve(), args.output, args.smoke)
    manifest = dict(application='PoseBridge', version=version, target=platform,
                    sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), hardware_accessed=False,
                    window_smoke=args.smoke,
                    source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                    source_dirty=bool(subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=normal'], cwd=ROOT)))
    (args.output / f'PoseBridge-{platform}.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    archive = args.output / f'PoseBridge-{version}-{platform}.zip'
    if archive.exists():
        raise SystemExit(f'Preserving existing archive: {archive}')
    if sys.platform == 'darwin':
        run(['ditto', '-c', '-k', '--sequesterRsrc', '--keepParent', package, archive])
    else:
        with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as z:
            for f in package.rglob('*'):
                if f.is_file():
                    z.write(f, f.relative_to(args.output))
    print(archive)


if __name__ == '__main__':
    main()
