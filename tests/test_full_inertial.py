"""Exercise restoration without touching hardware."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import full_inertial as experiment


class Restoration(unittest.TestCase):
    def exercise(self, failure):
        commands = []

        class Bridge:
            def __init__(self, _):
                self.device = dict(valid=True, output_register=0xa4, rate_register=8,
                                   rate_hz=50, algorithm_register=0, calsw=0)

            def configure(self, _): pass
            def stop(self): pass
            def close(self): pass

            def operation(self, command=None):
                if command:
                    commands.append(command)
                    if command['action'] == 'rate':
                        self.device['rate_hz'] = command['hz']
                        self.device['rate_register'] = {20: 7, 50: 8}[command['hz']]
                    else:
                        self.device['output_register'] = {v: k for k, v in experiment.FORMATS.items()}[command['format']]
                        if failure == 'uncertain_write' and self.device['output_register'] == 0xe4:
                            raise RuntimeError('device changed, but readback failed')
                return {'descriptor': {'device': dict(self.device)}}

        class Probe:
            returncode = 0
            done = False
            def __init__(self, *_, **__): pass
            def communicate(self, **_):
                if failure == 'interrupt' and not self.done:
                    raise KeyboardInterrupt()
                self.done = True
                return json.dumps({'passed': failure != 'capture_failed'}), None
            def poll(self): return 0 if self.done else None
            def terminate(self): self.done = True

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, report = root/'config.json', root/'result.json'
            config.write_text(json.dumps({'source': {'kind': 'usb', 'port': 'fake', 'baud': 115200}}))
            argv = ['full_inertial.py', '--config', str(config), '--library', 'fake',
                    '--probe', 'fake', '--output', str(report)]
            with patch.object(experiment, 'Bridge', Bridge), \
                 patch.object(experiment.subprocess, 'Popen', Probe), \
                 patch.object(experiment.signal, 'signal'), \
                 patch.object(experiment.platform, 'platform', return_value='test-host'), \
                 patch.object(experiment.sys, 'argv', argv), contextlib.redirect_stdout(io.StringIO()):
                code = experiment.main()
            value = json.loads(report.read_text())
            self.assertTrue(value['restored'])
            self.assertEqual(value['passed'], failure is None)
            self.assertEqual(code, 0 if failure is None else 1)
            self.assertEqual(commands[-2:], [{'action': 'output', 'format': 'timestamp_gyro_quaternion'},
                                           {'action': 'rate', 'hz': 50}])
            self.assertTrue(all(c['action'] in ('rate', 'output') for c in commands))

    def test_success_and_failures_always_restore_output_before_rate(self):
        for failure in [None, 'uncertain_write', 'interrupt', 'capture_failed']:
            with self.subTest(failure=failure):
                self.exercise(failure)


if __name__ == '__main__':
    unittest.main()
