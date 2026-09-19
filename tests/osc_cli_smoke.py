"""Independent decoder for the single current PoseBridge OSC contract."""
import itertools
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
binary = Path(sys.argv[1]) if len(sys.argv) > 1 else root / 'target/release/posebridge'
if sys.platform == 'win32' and binary.suffix != '.exe':
    binary = binary.with_suffix('.exe')

def decode(data):
    assert len(data) <= 8192
    def string(offset):
        end = data.index(0, offset)
        next_offset = (end + 4) & ~3
        assert next_offset <= len(data) and not any(data[end:next_offset])
        return data[offset:end].decode('utf-8'), next_offset
    address, offset = string(0)
    tags, offset = string(offset)
    assert tags in (',s', ',ishhhhhhhhihhfff', ',ishhhhhhhhihhffff')
    values = []
    for tag in tags[1:]:
        if tag == 's':
            value, offset = string(offset)
        else:
            fmt = {'h': '>q', 'i': '>i', 'f': '>f'}[tag]
            value = struct.unpack_from(fmt, data, offset)[0]
            offset += struct.calcsize(fmt)
        values.append(value)
    assert offset == len(data)
    return address, values

instances = set()
for (fmt, expected), synthetic in itertools.product([
    ('euler', (30, 20, 10)),
    ('quaternion', (0.189307857, 0.239298338, 0.038134576, 0.951548525)),
], [False, True]):
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 0)); udp.settimeout(.05)
    packets = []; stop = threading.Event()
    def receive():
        while True:
            try: packets.append(udp.recv(8193))
            except socket.timeout:
                if stop.is_set(): return
    reader = threading.Thread(target=receive); reader.start()
    try:
        command = [str(binary), 'simulate', '--source-id', 'integration', '--yaw', '30', '--pitch', '20', '--roll', '10',
            '--format', fmt, '--sample-rate-hz', '100', '--osc-rate-hz', '50', '--duration', '1', '--json',
            '--osc-target', f'127.0.0.1:{udp.getsockname()[1]}'] + (['--sample-clock'] if synthetic else [])
        started = time.perf_counter()
        run = subprocess.run(command, capture_output=True, text=True, timeout=12)
        elapsed = time.perf_counter() - started
    finally:
        stop.set(); reader.join(); udp.close()
    assert run.returncode == 0, (run.stdout, run.stderr)
    snapshots = [json.loads(line) for line in run.stdout.splitlines()]
    assert all(s['schema'] == 3 and s['descriptor']['source_id'] == 'integration' for s in snapshots)
    assert snapshots[-1]['status']['state'] == 'stopped'
    assert any(s['pose'] and s['pose']['fresh'] for s in snapshots)
    pose_count = 0; telemetry = []; previous = None
    for data in packets:
        address, values = decode(data)
        if address in ('/posebridge/info', '/posebridge/status'):
            assert len(values) == 1
            meta = json.loads(values[0]); telemetry.append(meta)
            assert meta['schema'] == 3 and meta['source_id'] == 'integration'
            for key in ('instance_id', 'session_id', 'reference_epoch', 'metadata_revision', 'message_seq'):
                assert isinstance(meta[key], str) and int(meta[key]) >= 0
            continue
        assert address == f'/posebridge/{fmt}'
        version, source, instance, session, sequence, tx, reference, revision, received, age, kind, sample, epoch = values[:13]
        assert version == 3 and source == 'integration' and min(instance, session, sequence, tx, reference, revision) > 0
        assert 0 <= age < 500000000
        if previous:
            assert instance == previous[0] and session == previous[1] and sequence > previous[2] and tx == previous[3] + 1
        if synthetic: assert kind == 2 and epoch == 1 and sample == received // 1000000
        else: assert (kind, sample, epoch) == (0, 0, 0)
        previous = instance, session, sequence, tx
        assert len(values[13:]) == len(expected)
        assert all(abs(a-b) < 1e-5 for a,b in zip(values[13:], expected))
        pose_count += 1
    assert 25 <= pose_count <= math.ceil(elapsed * 50) + 2, (pose_count, elapsed)
    assert {'info', 'status'} <= {v['kind'] for v in telemetry}
    assert any(m['kind'] == 'status' and m['status']['state'] == 'stopped' for m in telemetry)
    assert previous[0] not in instances; instances.add(previous[0])
    print(f'CLI -> OSC {fmt} synthetic={synthetic}: PASS ({pose_count} poses, {len(telemetry)} metadata packets)')

rejected = subprocess.run([str(binary), 'simulate', '--osc-version', 'v2', '--duration', '.1'], capture_output=True, text=True)
assert rejected.returncode != 0 and 'osc-version' in rejected.stderr
