"""Dual-output C ABI and CLI interoperability, using independent ephemeral UDP receivers."""
import ctypes as c
import json
import math
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import sys
import time

from hardware_smoke import Bridge, decode

ROOT = Path(__file__).resolve().parents[1]
TARGET = Path(os.environ.get('CARGO_TARGET_DIR', ROOT / 'target')) / 'release'
LIBRARY = TARGET / ('posebridge_capi.dll' if sys.platform == 'win32' else 'libposebridge_capi.dylib' if sys.platform == 'darwin' else 'libposebridge_capi.so')
CLI = TARGET / ('posebridge.exe' if sys.platform == 'win32' else 'posebridge')


def receiver():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    s.setblocking(False)
    return s


def target(s):
    return '127.0.0.1:' + str(s.getsockname()[1])


def dar_decode(data):
    assert len(data) == 28
    assert data[:16] == b'/ypr\0\0\0\0,fff\0\0\0\0'
    values = struct.unpack('>fff', data[16:])
    assert all(math.isfinite(v) for v in values)
    return values


def collect(pb, dar, seconds):
    poses, dolby, metadata = [], [], []
    finish = time.monotonic() + seconds
    while time.monotonic() < finish:
        ready, _, _ = select.select([s for s in [pb, dar] if s is not None], [], [], .01)
        for s in ready:
            while True:
                try:
                    data = s.recv(8193)
                except BlockingIOError:
                    break
                if s is dar:
                    dolby.append(dar_decode(data))
                else:
                    addr, value = decode(data)
                    if addr.endswith('quaternion'):
                        poses.append(value)
                    else:
                        metadata.append(value)
                        if 'descriptor' in value:
                            assert 'dolby_osc' not in value['descriptor']['application_config']
                        if 'status' in value:
                            assert 'dolby_osc' not in value['status']
    return poses, dolby, metadata


def c_api():
    pb, dar = receiver(), receiver()
    bridge = Bridge(LIBRARY)
    bridge.lib.pb_dolby_recenter.argtypes = [c.c_void_p]
    bridge.lib.pb_dolby_recenter.restype = c.c_int
    try:
        assert bridge.lib.pb_dolby_recenter(None) == 1
        assert bridge.lib.pb_dolby_recenter(bridge.ctx) == 4
        config = dict(source_id='dual-test', source=dict(kind='simulate', euler_deg=[35., 25., -10.], rate_hz=100),
                      osc=dict(target=target(pb), max_rate_hz=20, format='quaternion'),
                      dolby_osc=dict(target=target(dar), max_rate_hz=100))
        bridge.configure(config)
        assert bridge.lib.pb_start(bridge.ctx) == 0
        poses, dolby, meta = collect(pb, dar, .7)
        assert len(poses) >= 6 and len(dolby) >= len(poses) * 2 and meta
        before = bridge.snapshot()
        assert before['status']['dolby_osc']['active'] and not before['status']['dolby_osc']['reference_set']
        assert int(before['status']['dolby_osc']['send_errors']) == 0
        assert bridge.lib.pb_dolby_recenter(bridge.ctx) == 0
        poses2, dolby2, _ = collect(pb, dar, .4)
        assert poses2 and dolby2 and all(abs(v) < .001 for v in dolby2[-1])
        after = bridge.snapshot()
        assert after['status']['dolby_osc']['reference_set']
        for key in ['reference_epoch', 'metadata_revision']:
            assert before['pose'][key] == after['pose'][key]
        assert before['pose']['quaternion_xyzw'] == after['pose']['quaternion_xyzw']
        assert all(p['quaternion'] == poses[0]['quaternion'] for p in poses2)
        # A receiver disappearing never interrupts acquisition or the other output.
        dar.close()
        continued, _, _ = collect(pb, None, .2)
        assert continued and bridge.snapshot()['status']['state'] == 'active'
        bridge.stop()
        collect(pb, None, .05)
        assert not collect(pb, None, .12)[0]
        assert bridge.lib.pb_dolby_recenter(bridge.ctx) == 4
        # Dolby-only mode has no PoseBridge telemetry or legacy OSC counts.
        dar = receiver()
        config.pop('osc')
        config['dolby_osc']['target'] = target(dar)
        bridge.configure(config)
        assert bridge.lib.pb_start(bridge.ctx) == 0
        _, solo, _ = collect(None, dar, .2)
        snapshot = bridge.snapshot()
        assert solo and int(snapshot['status']['osc_sent']) == 0 and int(snapshot['status']['telemetry_sent']) == 0
        assert not snapshot['status']['dolby_osc']['reference_set']
        bridge.stop()
        collect(None, dar, .05)
        assert not collect(None, dar, .12)[1]
        print(f'C ABI dual OSC / independent forward / stop / receiver loss / Dolby-only: PASS ({len(poses)} PB, {len(dolby)} Dolby)')
    finally:
        bridge.close()
        pb.close()
        dar.close()


def cli():
    pb, dar = receiver(), receiver()
    process = subprocess.Popen([str(CLI), 'simulate', '--duration', '1', '--sample-rate-hz', '100',
                                '--osc-target', target(pb), '--dolby-target', target(dar), '--dolby-rate-hz', '50', '--json'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        poses, dolby, meta = collect(pb, dar, 1.3)
        _, stderr = process.communicate(timeout=5)
        assert process.returncode == 0, stderr
        assert len(poses) > 20 and len(dolby) > 10 and meta
        print(f'CLI -> independent PoseBridge 3 and /ypr receivers: PASS ({len(poses)} PB, {len(dolby)} Dolby)')
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        pb.close()
        dar.close()


if __name__ == '__main__':
    c_api()
    cli()
