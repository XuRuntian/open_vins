"""A truncated native output must not hide the signal that stopped its writer."""
from argparse import Namespace
import json
import signal

import pytest

from ov_offline import run as runner


@pytest.mark.parametrize('native_signal', [signal.SIGFPE, signal.SIGSEGV])
def test_native_signal_is_reported_before_partial_csv_export(tmp_path, monkeypatch, native_signal):
    binary = tmp_path / 'native_stub'
    binary.write_text(
        '#!/usr/bin/python3\n'
        'import os, pathlib, resource, signal, sys\n'
        'resource.setrlimit(resource.RLIMIT_CORE, (0, 0))\n'
        'output = pathlib.Path(sys.argv[sys.argv.index("--output_dir") + 1])\n'
        '(output / "states.csv").write_text("timestamp,timestamp_imu,px,py,pz,qx,qy,qz,qw\\n1,1,0,0,0,0")\n'
        f'os.kill(os.getpid(), {int(native_signal)})\n'
    )
    binary.chmod(0o755)
    monkeypatch.setattr(runner, 'load_inputs', lambda args: {
        'output_dir': str(tmp_path / 'output'), 'estimator_config_path': str(tmp_path / 'config'),
        'videos': {side: {'path': str(tmp_path / side)} for side in ('left', 'right')},
        'stereo_pairs_path': str(tmp_path / 'pairs'), 'imu_clean_path': str(tmp_path / 'imu'),
    })
    args = Namespace(binary=binary, start_s=0., duration_s=None)
    assert runner.run(args) == 128 + native_signal
    output = tmp_path / 'output/trajectory'
    report = json.loads((output / 'run_report.json').read_text())
    assert report['status'] == 'failed'
    assert report['returncode'] == -native_signal
    assert report['signal'] == native_signal.name
    assert native_signal.name in report['error']
    assert report['wall_duration_s'] > 0
    assert (output / 'states.csv').read_text().endswith('1,1,0,0,0,0')
    assert not (output / 'trajectory.npz').exists()
