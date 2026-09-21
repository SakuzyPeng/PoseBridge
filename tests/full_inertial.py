"""Explicit 20 Hz E4 experiment; restore RAM settings on success, failure or Ctrl-C.

Only writes RRATE and RSW. Never calibrates, zeroes or saves. The read-only probe
keeps aggregate metrics in memory; this wrapper saves one small JSON report.
Run with the sensor disconnected from all other applications. On macOS BLE,
use the existing permission-enabled host as described in docs/motion.md.
"""
import argparse
import datetime
import json
from pathlib import Path
import platform
import signal
import subprocess
import sys

from hardware_smoke import Bridge

FORMATS = {0x61: 'motion', 0x81: 'timestamp_euler', 0x84: 'timestamp_quaternion',
           0xa4: 'timestamp_gyro_quaternion', 0xe4: 'experimental_full_inertial_20hz'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True, help='Existing PoseBridge Config JSON')
    p.add_argument('--library', type=Path, required=True)
    p.add_argument('--probe', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=int, default=60)
    args = p.parse_args()
    if not 60 <= args.seconds <= 600:
        p.error('capture must be 60..600 seconds after warmup')
    config = json.loads(args.config.read_text())
    if config['source']['kind'] not in ('usb', 'ble'):
        p.error('hardware usb/ble source required')
    config.pop('osc', None)
    config['pose_input'] = 'stream_quaternion'
    bridge = Bridge(args.library)
    before = None
    changed = False
    process = None
    report = {'platform': platform.platform(), 'transport': config['source']['kind'],
              'date_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'passed': False, 'restored': False}
    try:
        bridge.configure(config)
        before = bridge.operation()['descriptor']['device']
        report['before'] = before
        assert before['valid'] and before['output_register'] in FORMATS, before
        assert before['rate_hz'] in (1, 2, 5, 10, 20, 50, 100, 200), before
        assert before['calsw'] == 0, 'Finish active calibration before this experiment'
        assert before['output_register'] != 0xe4 or before['rate_hz'] == 20, before
        changed = True  # Set before any write, including uncertain transport failures.
        bridge.operation({'action': 'rate', 'hz': 20})
        bridge.operation({'action': 'output', 'format': FORMATS[0xe4]})
        current = bridge.operation()['descriptor']['device']
        report['configured'] = current
        assert current['rate_register'] == 7 and current['output_register'] == 0xe4, current
        bridge.stop()
        process = subprocess.Popen([str(args.probe.resolve()), json.dumps(config), str(args.seconds)],
                                   stdout=subprocess.PIPE, text=True)
        result, _ = process.communicate(timeout=args.seconds + 40)
        assert process.returncode == 0, f'probe exit {process.returncode}'
        report['capture'] = json.loads(result)
        report['passed'] = report['capture']['passed']
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error) or type(error).__name__
    finally:
        # Do not let a repeated Ctrl-C bypass restoration. Killing the host or
        # removing power cannot be recovered in-process; report remains failed.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
        try:
            if changed:
                bridge.stop()
                bridge.configure(config)
                # Leave E4 before restoring any non-20 Hz rate.
                bridge.operation({'action': 'output', 'format': FORMATS[before['output_register']]})
                bridge.operation({'action': 'rate', 'hz': int(before['rate_hz'])})
                after = bridge.operation()['descriptor']['device']
                report['after'] = after
                assert all(after[key] == before[key] for key in
                           ('rate_register', 'output_register', 'algorithm_register', 'calsw')), after
                report['restored'] = True
        except Exception as error:
            report['restoration_error'] = str(error)
        finally:
            try:
                bridge.close()
            except Exception as error:
                report['close_error'] = str(error)
                report['passed'] = False
        report['passed'] &= report['restored']
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
