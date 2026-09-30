"""Opt-in cold-start regression; the private recording is never committed."""
import json
import csv
import os
from pathlib import Path

import numpy as np
import pytest

from ov_offline.run import parse_args, run


@pytest.mark.skipif(not os.environ.get("VIO_PRIVATE_DATA_ROOT"), reason="Private cashier recording not supplied")
def test_cashier_cold_start_does_not_reproduce_runaway_path(tmp_path):
    root = Path(os.environ["VIO_PRIVATE_DATA_ROOT"])
    session = "session_20260827_113215_收银_20260903162506"
    window = root / "vio_window_audit_20260929/cold_v2" / session / "start_20_duration_8"
    videos = root / "processed_data" / session / "videos"
    args = parse_args([
        "--binary", os.environ["VIO_OFFLINE_BINARY"],
        "--config", str(window / "config/estimator_config.yaml"),
        "--left_video", str(videos / "left_rectified.mp4"),
        "--right_video", str(videos / "right_rectified.mp4"),
        "--stereo_pairs", str(window / "pairs.csv"),
        "--imu", str(window / "imu.csv"),
        "--output_dir", str(tmp_path), "--time-offset-mode", "estimate",
    ])
    assert run(args) == 0
    with np.load(tmp_path / "trajectory/trajectory.npz", allow_pickle=False) as data:
        valid = data["status"] == "valid"
        common = valid & (data["frame_index"] >= 661) & (data["frame_index"] <= 839)
        # Timing changes may move initialization by a few frames; require a
        # sustained tail rather than prescribing the old initializer's frame.
        assert common.sum() >= 165
        assert data["frame_index"][valid][-1] == 839
        assert np.all(np.diff(data["frame_index"][valid]) == 1)
        positions = data["T_world_camera"][common, :3, 3]
        path = np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()
        # Symptom guard, not a ground-truth accuracy claim (full replay: 0.824 m).
        assert path < 2.0, f"Reproduced cold-start runaway: {path:.6f} m"
        assert len(data["frame_index"]) == 1360
        assert not {"valid", "processed", "initialized"}.intersection(data.files)
        assert np.isnan(data["T_world_camera"][~valid]).all()
        assert np.isnan(data["T_world_imu"][~valid]).all()


@pytest.mark.skipif(not os.environ.get("VIO_SHELF_COLD_FIXTURE"), reason="Private shelf cold fixture not supplied")
def test_shelf_cold_start_has_sustained_estimates_without_history(tmp_path):
    fixture = Path(os.environ["VIO_SHELF_COLD_FIXTURE"])
    args = parse_args([
        "--binary", os.environ["VIO_OFFLINE_BINARY"],
        "--config", str(fixture / "config/estimator_config.yaml"),
        "--left_video", str(fixture / "left.mp4"),
        "--right_video", str(fixture / "right.mp4"),
        "--stereo_pairs", str(fixture / "pairs.csv"),
        "--imu", str(fixture / "imu.csv"),
        "--output_dir", str(tmp_path), "--time-offset-mode", "estimate",
    ])
    code = run(args)
    report = json.loads((tmp_path / "trajectory/run_report.json").read_text())
    assert code == 0, report["status"]
    with np.load(tmp_path / "trajectory/trajectory.npz", allow_pickle=False) as data:
        assert data["schema_version"].item() == "openvins.frame_aligned.v2"
        assert len(data["frame_index"]) == 120
        assert not {"valid", "processed", "initialized"}.intersection(data.files)
        valid = data["status"] == "valid"
        with (fixture / "pairs.csv").open() as handle:
            first_camera = float(next(csv.DictReader(handle))["timestamp"])
        with (fixture / "imu.csv").open() as handle:
            first_imu = float(next(csv.DictReader(handle))["timestamp"])
        covered = first_camera + report["offline"]["initial_time_offset_s"] >= first_imu
        assert data["status"][0] == ("initializing" if covered else "imu_unavailable")
        assert data["status"][-1] == "valid"
        assert valid.sum() >= 45, "Do not pass with only a transient initialized state"
        assert np.all(np.diff(data["frame_index"][valid]) == 1)
        assert data["timestamp_camera"][valid][-1] - data["timestamp_camera"][valid][0] >= 1.4
        assert np.isfinite(data["T_world_camera"][valid]).all()
        assert np.isnan(data["T_world_camera"][~valid]).all()
        assert np.isnan(data["T_world_imu"][~valid]).all()
