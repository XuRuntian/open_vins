# ROS-Free Offline Replay

This module owns stereo/IMU replay, native OpenVINS invocation, trajectory
conversion and diagnostics. It does not import or require roboego-hand-vis.
Offline synchronous stereo initialization now replenishes features when shared
stereo IDs leave the initializer starved. Acceptance thresholds are unchanged.
Python/compatibility entrypoints enable camera/IMU timing bootstrap and the
official online offset state by default; see Timing below. ROS support elsewhere
in this repository is retained.

## Build

From the OpenVINS repository root, with Eigen, OpenCV, Ceres and Boost installed:

```bash
cmake -S ov_msckf -B ov_msckf/build -DENABLE_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build ov_msckf/build --target roboego_openvins_offline --parallel 2
```

Python export requires Python 3.10+, NumPy, PyYAML and Python OpenCV. Tests additionally require
pytest. No ROS, pretrained models or runtime networking are
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

### Legacy CLI Compatibility

The built `roboego_openvins_offline` launcher accepts the original direct input
flags and writes `trajectory.jsonl` and `run_report.json` directly under its
`--output_dir` (no additional `trajectory/` suffix). JSONL preserves the original
timestamp/frame fields and `position_world_imu`, `velocity_world_imu`, and
`orientation_world_imu_xyzw`; additional matrix/camera fields are additive.
Velocity is the estimator's world-frame velocity, not a position difference.
The legacy orientation field preserves OpenVINS' raw JPL xyzw quaternion
(`state->_imu->quat()`, global-to-IMU convention). It must not be interpreted
as a Hamilton world-to-IMU rotation. Matrix fields remain the explicit
`T_world_imu` / `T_world_camera` contract; no trajectory conversion is changed.

The compatibility report uses the original `schema_version: "1.0"`, backend,
published stereo/IMU counts, pose count, initialized flag and JSONL artifact
path, plus current status/diagnostics. The standalone Python entry retains its
`roboego.vio.offline.v1` report schema. Counts describe actual feed calls in the
corrected replay, not the old runner's counts; initialization does not certify
trajectory quality. Existing stricter failure handling and time-window semantics
remain unchanged (the end of a requested interval is exclusive).

Rebuild the native target when updating this adapter: `states.csv` now includes
velocity and raw quaternion columns. Old binary/new exporter mixtures are not
supported. Docker delivery remains `trajectory.npz` and `report.json`; internal
JSONL and detailed reports are retained only with `--debug`.

`OUTPUT_DIR/trajectory` contains `states.csv`, `trajectory.jsonl`, `trajectory.npz`,
`frame_status.csv`, `offline_report.json` and `run_report.json`.
`OUTPUT_DIR/openvins.log` retains native output. Use a fresh output directory;
existing trajectories are not overwritten. Exported transforms are T_world_imu
and T_world_camera; camera quaternions use Hamilton xyzw, while the legacy IMU
quaternion field uses the raw OpenVINS convention described above. Covariance is
not exported.

No initialized trajectory returns nonzero. Exit zero / `status: complete` means
execution produced at least two poses, **not** a quality guarantee. A cashier
recording diverges severely with the original fixed time-offset configuration;
online time-offset estimation addresses that reproduced case, not all possible
tracking failures.
Do not accept trajectories solely based on the exit code.

### Frame-Aligned NPZ

`trajectory.npz` now uses `openvins.frame_aligned.v2`. Every frame of the left
input video has one row, including missing poses. `frame_index` is exactly
`0..N-1` for the input video's reported frame count. `pose_count` still counts
only available poses; `frame_alignment` in the report contains frame count,
valid/invalid pose counts, requested stereo pair count and per-status counts.
This changes NPZ row semantics: consumers must filter using `status == 'valid'` rather than
assuming every matrix is usable. The legacy JSONL remains valid-pose-only and
retains its field contract.

Per-frame fields: `frame_index`, `right_frame_index`, `timestamp_camera`,
`timestamp_imu`, `status`,
`T_world_imu`, `T_world_camera`. Missing matrices are entirely NaN, not identity,
zero, repeated previous poses or interpolated poses. Use `np.load(...,
allow_pickle=False)`. Scalar fields identify schema and `quality_status`.

| Status | Meaning |
| --- | --- |
| `valid` | Finite estimator pose advanced to this camera timestamp; accuracy not validated |
| `initializing` | Camera processed, initialization not complete |
| `imu_unavailable` | Aligned camera time outside available IMU bracketing |
| `state_not_advanced` | Initialized, but state did not advance to this frame; not a definitive tracking-loss diagnosis |
| `invalid_state` | Estimator produced a non-finite state; replay fails |
| `not_processed` | Requested pair not reached, e.g. after a runtime failure |
| `not_requested` | Known stereo pair outside the requested start/duration window |
| `no_stereo_pair` | Left video frame absent from input stereo CSV |

Start/duration replay still returns N rows, marking excluded known pairs
`not_requested`; original frame indices do not change. A frame without a stereo
pair has right index -1 and unknown camera/IMU timestamps NaN; no timestamps are
fabricated from FPS. Redundant `valid`, `processed`, and `initialized` arrays
are not saved in NPZ. Native initialization/state-advance flags remain in the
debug `frame_status.csv`. Existing v1 files are not rewritten; rebuild the
runtime and rerun to produce v2 files.
An all-uninitialized run still returns nonzero but exports the aligned invalid
rows. Docker retains these rows even in default non-debug failure output, when
export succeeded. Invalid input preventing export may produce only report/log.

```python
with np.load('trajectory.npz', allow_pickle=False) as data:
    valid = data['status'] == 'valid'
    usable_frames = data['frame_index'][valid]
    usable_poses = data['T_world_camera'][valid]
    unavailable = data['frame_index'][~valid]
```

## Tests

```bash
VIO_OFFLINE_BINARY="$PWD/ov_msckf/build/openvins_offline_native" \
  python3 -m pytest -p no:cacheprovider ov_offline/tests -q
```

Fixtures are synthetic, not device calibration. Tests cover export, input
validation, time selection, uninitialized failure and stationary ZUPT startup.
The module can be built and tested with only this repository mounted.

For a data-only, full-session Docker regression, use
`diagnostics/audit_runtime.py --data-root DIR --output NEW_DIR --image IMAGE`.
It pins the image ID, runs raw/rectified inputs, checks the delivery contract and
flags gross trajectory symptoms without altering estimates. `--reference DIR`
compares common frames with a prior audit. Relocated rectified manifests can
use `--manifest-mount-alias` for validated read-only original-path mounts, not
rewritten metadata. See `diagnostics/RELEASE_AUDIT_20260930.md` for the tested
matrix and limits; passing the audit is not an accuracy certificate.

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

## Timing

The offline Python entry and compatibility launcher default to
`--time-offset-mode estimate`. This enables OpenVINS' existing
`do_calib_camera_timeoffset` state. Before creating the initializer/filter, a
confidence-checked bootstrap can replace the YAML initial offset using the
requested recording, not a session-specific hard-coded correction. The native C++ entry alone defaults
to `config` to honor explicit native configurations. `--time-offset-mode config`
on either CLI honors `calib_cam_timeoffset` from the YAML. The compatibility
launcher also accepts the environment default `OPENVINS_TIME_OFFSET_MODE`.

The source calibration/config is never rewritten. Reports record whether the
offset was estimated, its initial value and its final value. Per-frame offsets
are in `frame_status.csv`; exported IMU timestamps use the updated estimate.
Online calibration requires informative observations and is not guaranteed to
repair arbitrary synchronization, calibration or low-excitation failures.

### Offline Startup Alignment

`--timing-bootstrap auto` is the native default in `estimate` mode. It reads up
to the first six seconds **inside the requested camera window**, estimates
short-baseline visual rotations with OpenCV KLT/RANSAC using the supplied camera
model, and compares them to calibrated gyro integration. It searches +/-100 ms
around the supplied calibration in 2 ms increments, using only intervals with
IMU coverage for the entire search. IMU endpoints are never extrapolated; gaps
over 20 ms disqualify an interval. No earlier IMU is obtained from another file.

Acceptance requires at least 24 intervals spanning 2.4 seconds, varying rotation,
consistent estimates in the first/second halves, an interior and distinguishable
minimum, and a materially better fit than the calibration. Constant gyro bias
is treated as a nuisance in scoring, not installed as a filter bias. These are
conservative bootstrap checks, not statistical confidence intervals or relaxed
OpenVINS initialization/update thresholds. If checks fail, the calibration is
retained and the rejection reason is reported; this does **not** certify it as
correct. The existing initializer may still fail or produce an inaccurate pose.

Accepted timing is applied to both `VioManagerOptions.calib_camimu_dt` and
`init_options.calib_camimu_dt` before constructing OpenVINS. Online offset
estimation then continues normally. Source timestamps/calibrations and spatial
extrinsics are not edited. Replay starts again from the requested beginning;
look-ahead does not advance the VIO state or fabricate initial poses. This is an
offline, noncausal preprocessing step: the first pose timestamp is **not** a
real-time startup latency measurement. A timing change may legitimately change
IMU boundary coverage and the first initialized frame.

Use `--timing-bootstrap off` (or `OPENVINS_TIMING_BOOTSTRAP=off` for the Python
launcher) to isolate the previous online-only behavior. `--time-offset-mode
config` always disables bootstrap and honors the YAML's online-calibration flag.
`offline.timing_bootstrap` in the delivery report records the decision, original
and candidate offsets, usable interval count, residual scores and split-window
disagreement. Zero scores in an early rejection mean fitting was not performed.
No new per-frame arrays or default delivery artifacts are added.

Limitations: essential-matrix rotations can be biased by dynamic objects,
translation/degenerate geometry or incorrect calibration. The checks can reject
useful data and cannot rule out every false alignment. This is a repair of the
reproduced startup failure, not a replacement sensor calibration or proof of
upstream timestamp provenance. Four local full recordings plus the two fixed
cold windows are regression evidence, not ground-truth accuracy validation.

```bash
cmake --build ov_msckf/build --target offline_timing_test
ov_msckf/build/offline_timing_test
VIO_PRIVATE_DATA_ROOT=/data/ego_data \
VIO_SHELF_COLD_FIXTURE=/data/ego_data/vio_frame_aligned_20260929/cold_fixture \
VIO_OFFLINE_BINARY="$PWD/ov_msckf/build/openvins_offline_native" \
  python3 -m pytest ov_offline/tests/test_cold_start_regression.py -q
```

The private cashier regression demonstrated 35 m drift in 10 seconds with the
fixed -47.8 ms offset. Full replay with online estimation converged to -6.8 ms,
reduced path length from about 1984 m to 8.78 m, and stayed within 0.80 m of its
initial position. No pose samples were clipped to force these numbers.

Optional real-data regression (prepare inputs first, use paths visible to the
test process):

```bash
VIO_CASHIER_MANIFEST=/results/run/run_manifest.json \
VIO_OFFLINE_BINARY="$PWD/ov_msckf/build/openvins_offline_native" \
  python3 -m pytest ov_offline/tests/test_cashier_regression.py -q
```

`VIO_CASHIER_TIME_MODE=config` reproduces the failing fixed-offset case when the
fixture YAML disables calibration. The fixture is not committed; ordinary tests
skip this case unless the environment variable is supplied. The 10 m test bound
is specific to this recording and detects catastrophic drift, not general accuracy.
`diagnostics/check_rotation_alignment.py` independently compares short-baseline
visual rotations to gyro predictions; it does not modify calibration or produce
an authoritative replacement calibration.
