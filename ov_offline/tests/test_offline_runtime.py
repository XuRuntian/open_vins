"""Container integration tests; these exercise the real compiled offline runner."""
import json
import os
from pathlib import Path
import subprocess
from argparse import Namespace
import csv
import shutil

import cv2
import numpy as np
import pytest

from ov_offline.run import run, parse_args

IMU_FIELDS = ["timestamp", "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
              "linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z"]


BINARY = Path(os.environ.get("VIO_OFFLINE_BINARY", "/home/user/build/openvins/openvins_offline_native"))
pytestmark = pytest.mark.skipif(not Path("/home/user/build").is_dir() and "VIO_OFFLINE_BINARY" not in os.environ,
                                reason="Requires VIO container build mount")


def test_offline_entry_exists_and_explains_required_inputs():
    assert BINARY.is_file(), f"Missing offline adapter: {BINARY}"
    result = subprocess.run([str(BINARY), "--help"], capture_output=True, text=True)
    assert result.returncode == 0
    assert "--stereo_pairs" in result.stdout
    assert "--imu" in result.stdout


def test_offline_entry_rejects_missing_input():
    assert BINARY.is_file(), f"Missing offline adapter: {BINARY}"
    result = subprocess.run([str(BINARY), "--config", "/missing/config.yaml"], capture_output=True, text=True)
    assert result.returncode != 0
    assert "ERROR" in result.stderr


def make_replay(tmp_path, frames=20, textured=False, stationary_init=False):
    """Synthetic data/calibration; never substituted for a real recording."""
    shutil.copytree(Path(__file__).parent / "data", tmp_path / "config")
    estimator = tmp_path / "config/estimator_config.yaml"
    config = estimator.read_text().replace("try_zupt: false", f"try_zupt: {str(stationary_init).lower()}")
    config = config.replace("record_timing_information: true", "record_timing_information: false")
    estimator.write_text(config)
    rng = np.random.default_rng(42)
    texture = rng.integers(0, 256, size=(240, 320, 3), dtype=np.uint8)
    videos = {}
    for side in ("left", "right"):
        path = tmp_path / f"{side}.avi"
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30., (320, 240))
        assert writer.isOpened()
        image = texture if side == "left" else np.roll(texture, -8, axis=1)
        for _ in range(frames):
            writer.write(image if textured else np.zeros((240, 320, 3), dtype=np.uint8))
        writer.release()
        videos[side] = {"path": str(path)}
    pairs = tmp_path / "pairs.csv"
    with pairs.open("w") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp", "left_frame_index", "right_frame_index"])
        for index in range(frames):
            writer.writerow([1 + index / 30, index, index])
    imu = tmp_path / "imu.csv"
    with imu.open("w") as handle:
        writer = csv.writer(handle)
        writer.writerow(IMU_FIELDS)
        for index in range(int((2 + frames / 30) * 200)):
            writer.writerow([index / 200, 0, 0, 0, 0, 0, 9.81])
    manifest = {"output_dir": str(tmp_path / "output"), "videos": videos,
                "estimator_config_path": str(estimator),
                "stereo_pairs_path": str(pairs), "imu_clean_path": str(imu),
                "T_imu_camera0": np.eye(4).tolist()}
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest))
    return Namespace(binary=BINARY, manifest=path, start_s=0., duration_s=None)


def test_uninitialized_replay_is_failure_with_diagnostics(tmp_path):
    args = make_replay(tmp_path)
    assert run(args) == 3
    root = tmp_path / "output/trajectory"
    report = json.loads((root / "run_report.json").read_text())
    assert report["status"] == "not_initialized"
    assert report["pose_count"] == 0
    assert report["offline"]["processed_camera_frames"] == 20
    assert report["offline"]["unbracketed_camera_frames"] == 0
    assert (root / "trajectory.jsonl").read_text() == ""
    assert np.load(root / "trajectory.npz")["T_world_camera"].shape == (0, 4, 4)
    assert (root / "frame_status.csv").is_file()
    with pytest.raises(FileExistsError):
        run(args)


def test_stationary_scene_exposes_wait_for_motion_policy(tmp_path):
    args = make_replay(tmp_path, frames=100, textured=True)
    assert run(args) == 3
    log = (tmp_path / "output/openvins.log").read_text()
    assert "no accel jerk detected" in log


def test_official_zero_velocity_option_initializes_stationary_fixture(tmp_path):
    args = make_replay(tmp_path, frames=100, textured=True, stationary_init=True)
    assert run(args) == 0
    trajectory = np.load(tmp_path / "output/trajectory/trajectory.npz")
    assert len(trajectory["frame_index"]) >= 2
    assert np.isfinite(trajectory["T_world_camera"]).all()
    positions = trajectory["T_world_camera"][:, :3, 3]
    assert np.linalg.norm(positions[-1] - positions[0]) < 0.01


def test_replay_rejects_duplicate_imu_timestamps(tmp_path):
    args = make_replay(tmp_path)
    imu = tmp_path / "imu.csv"
    lines = imu.read_text().splitlines()
    lines[2] = lines[1]
    imu.write_text("\n".join(lines) + "\n")
    assert run(args) == 2
    assert "timestamps must strictly increase" in (tmp_path / "output/openvins.log").read_text()


def test_replay_honors_requested_time_range(tmp_path):
    args = make_replay(tmp_path)
    args.start_s = 0.2
    args.duration_s = 0.1
    assert run(args) == 3
    report = json.loads((tmp_path / "output/trajectory/run_report.json").read_text())
    assert report["offline"]["processed_camera_frames"] == 3


def test_standalone_cli_without_project_manifest(tmp_path):
    fixture = make_replay(tmp_path, frames=100, textured=True, stationary_init=True)
    fixture.manifest.unlink()
    args = parse_args(["--binary", str(BINARY), "--config", str(tmp_path / "config/estimator_config.yaml"),
                       "--left_video", str(tmp_path / "left.avi"), "--right_video", str(tmp_path / "right.avi"),
                       "--stereo_pairs", str(tmp_path / "pairs.csv"), "--imu", str(tmp_path / "imu.csv"),
                       "--output_dir", str(tmp_path / "output")])
    assert run(args) == 0
    assert np.load(tmp_path / "output/trajectory/trajectory.npz")["T_world_camera"].shape[0] >= 2


@pytest.mark.parametrize("stationary_init,expected", [(False, 3), (True, 0)])
def test_legacy_cli_exports_into_exact_output_directory(tmp_path, stationary_init, expected):
    make_replay(tmp_path, frames=100, textured=True, stationary_init=stationary_init)
    launcher = BINARY.with_name("roboego_openvins_offline")
    output = tmp_path / "legacy/trajectory"
    result = subprocess.run([str(launcher), "--config", str(tmp_path / "config/estimator_config.yaml"),
                             "--left_video", str(tmp_path / "left.avi"),
                             "--right_video", str(tmp_path / "right.avi"),
                             "--stereo_pairs", str(tmp_path / "pairs.csv"),
                             "--imu", str(tmp_path / "imu.csv"), "--output_dir", str(output)],
                            capture_output=True, text=True)
    assert result.returncode == expected, result.stdout + result.stderr
    report = json.loads((output / "run_report.json").read_text())
    assert report["status"] == ("complete" if expected == 0 else "not_initialized")
    assert (output / "trajectory.npz").is_file()
    assert not (output / "trajectory").exists()
