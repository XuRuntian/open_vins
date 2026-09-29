# ROS-Free Offline Replay

This module owns stereo/IMU replay, native OpenVINS invocation, trajectory
conversion and diagnostics. It does not import or require roboego-hand-vis.
The estimator and initialization defaults are unchanged. ROS support elsewhere
in this repository is retained.

## Build

From the OpenVINS repository root, with Eigen, OpenCV, Ceres and Boost installed:

```bash
cmake -S ov_msckf -B ov_msckf/build -DENABLE_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build ov_msckf/build --target roboego_openvins_offline --parallel 2
```

Python export requires Python 3.10+, NumPy and PyYAML. Tests additionally require
pytest and Python OpenCV. No ROS, pretrained models or runtime networking are
needed. Tested with the existing Ubuntu 22.04 VIO dependency image.

## Independent Execution

From the repository root (or invoke `ov_offline/run.py` by absolute path):

```bash
python3 -m ov_offline.run \
  --binary ov_msckf/build/openvins_offline_native \
  --config /data/config/estimator_config.yaml \
  --left_video /data/left.mp4 --right_video /data/right.mp4 \
  --stereo_pairs /data/stereo_pairs.csv --imu /data/imu_clean.csv \
  --output_dir /results/run01 --start_s 0 --duration_s 20
```

The configuration is an OpenVINS estimator YAML, referencing its IMU and camera
YAMLs. Camera 0 must contain `T_imu_cam` as a nested 4x4 array. This is the rigid
transform from camera to IMU, not Kalibr's inverse `T_cam_imu`. Inputs can be raw
or rectified images, but intrinsics, distortion, extrinsics and resolution must
describe the images actually supplied. Session-specific rectification and
Kalibr conversion remain upstream data preparation, not estimator logic.

- Stereo CSV requires `timestamp,left_frame_index,right_frame_index`.
- IMU CSV requires `timestamp,angular_velocity_x,angular_velocity_y,angular_velocity_z,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z`.
- Timestamps are seconds, gyro is rad/s, acceleration is m/s^2, indices are
  zero-based. Rows must strictly increase in time. Camera rows represent already
  matched stereo pairs. Additional numeric CSV columns are accepted.
- `timestamp_imu = timestamp_camera + timeshift_cam_imu`; boundary camera frames
  without IMU bracketing are counted and skipped. Do not shift timestamps twice.
- `start_s` is relative to the first stereo timestamp; `duration_s` is optional.
  The adapter sequentially decodes to the selected frame indices.

## Manifest Compatibility

Existing orchestrators can continue supplying `--manifest FILE` instead of the
individual input flags. The manifest is JSON with these required fields:

```json
{
  "output_dir": "/results/run01",
  "estimator_config_path": "/data/config/estimator_config.yaml",
  "videos": {"left": {"path": "/data/left.mp4"}, "right": {"path": "/data/right.mp4"}},
  "stereo_pairs_path": "/data/stereo_pairs.csv",
  "imu_clean_path": "/data/imu_clean.csv",
  "T_imu_camera0": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
}
```

The identity above is illustrative only; supply the real camera-to-IMU transform.
Manifest-relative paths resolve relative to the manifest directory. Extra keys
are ignored. Individual CLI paths resolve relative to the working directory.
The existing `roboego.vio.offline.v1` report schema name is retained for consumers.

## Outputs And Limits

`OUTPUT_DIR/trajectory` contains `states.csv`, `trajectory.jsonl`, `trajectory.npz`,
`frame_status.csv`, `offline_report.json` and `run_report.json`.
`OUTPUT_DIR/openvins.log` retains native output. Use a fresh output directory;
existing trajectories are not overwritten. Exported transforms are T_world_imu
and T_world_camera; quaternions use Hamilton xyzw. Covariance is not exported.

No initialized trajectory returns nonzero. Exit zero / `status: complete` means
execution produced at least two poses, **not** a quality guarantee. A known
cashier recording initializes but diverges severely; migration does not fix it.
Do not accept trajectories solely based on the exit code.

## Tests

```bash
VIO_OFFLINE_BINARY="$PWD/ov_msckf/build/openvins_offline_native" \
  python3 -m pytest -p no:cacheprovider ov_offline/tests -q
```

Fixtures are synthetic, not device calibration. Tests cover export, input
validation, time selection, uninitialized failure and stationary ZUPT startup.
The module can be built and tested with only this repository mounted.

## Original Orchestrator Compatibility

The build target `roboego_openvins_offline` produces two sibling files:

- `openvins_offline_native`: compiled C++ estimator adapter.
- `roboego_openvins_offline`: executable Python compatibility launcher, accepting
  the original `--config`, video/CSV paths, `--output_dir`, `--start_s` and optional
  `--duration_s`, without requiring `--binary` or a project manifest.

The compatibility launcher writes directly into the supplied `--output_dir`.
In contrast, `python -m ov_offline.run` treats `--output_dir` as the run root and
appends `/trajectory`. This distinction preserves the original orchestrator,
which already appends that directory. The launcher calls the sibling native
binary and exports reports, JSONL and NPZ, propagating failure exit codes.
Its generated source path points to this checkout; keep it mounted at the
build-time location or rerun CMake after moving the checkout. Python dependencies
are required for this compatibility command, but ROS is not.

No project-side Python modifications are needed. The original project's
`offline_entry.py`, `runner.py`, configuration and tests can stay unchanged.
