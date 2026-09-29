"""Opt-in replay of the private cashier recording; no dataset is committed."""
from argparse import Namespace
import json
import os
from pathlib import Path

import numpy as np
import pytest

from ov_offline.run import run


@pytest.mark.skipif(not os.environ.get("VIO_CASHIER_MANIFEST"), reason="Private cashier fixture not supplied")
def test_cashier_short_replay_stays_in_work_area(tmp_path):
    source = json.loads(Path(os.environ["VIO_CASHIER_MANIFEST"]).read_text())
    source["output_dir"] = str(tmp_path / "output")
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps(source))
    binary = Path(os.environ.get("VIO_OFFLINE_BINARY", "/home/user/build/openvins/openvins_offline_native"))
    mode = os.environ.get("VIO_CASHIER_TIME_MODE", "estimate")
    assert run(Namespace(binary=binary, manifest=manifest, start_s=0., duration_s=10., time_offset_mode=mode)) == 0
    result = np.load(tmp_path / "output/trajectory/trajectory.npz")
    valid = result['status'] == 'valid'
    time = result["timestamp_camera"][valid]
    position = result["T_world_camera"][valid, :3, 3]
    assert time[-1] - time[0] > 7.0, "Do not pass by discarding the divergent tail"
    # This task-specific bound catches catastrophic drift, not pose accuracy.
    radius = np.linalg.norm(position - position[0], axis=1).max()
    assert radius < 10.0, f"Cashier left the generous 10 m work-area bound: {radius:.2f} m"
