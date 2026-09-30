# Cold-start diagnosis: first evidence checkpoint

This is a diagnosis, not an algorithm fix or trajectory-accuracy acceptance.
No estimator parameters, calibration, production output policy, image tag, or
input recordings were changed. No commits or pushes were made.

## Baseline and reproduction

- OpenVINS HEAD: `7e036d941810a7c9477e80160e95025d22608471`.
  Created `fix/vio-cold-start` from `feature/ros-free-offline`.
- Integration HEAD: `1d47c92ea0b0696885eb16e5028bea77584b1191`.
  Left `feat/docker-hand-vio-envs` and its files unchanged.
- Preserved existing untracked build/install/log directories.
- Pinned runtime: `sha256:4474a8157e80308ed46e72914614c5ee13e2b9fe9cbd1bb9639a347783c7ace8`.
  Its revision labels match both HEADs. This differs from the older image ID in
  `docker/vio-env/VALIDATION.md`; results here use the pinned ID, not a mutable tag.
- Runtime doctor passed. Bundled tests: 29 passed, 1 private-data test skipped.
- All artifacts: `/home/user/ego_data/vio_cold_diagnosis_20260929/`.

From `/home/user/open_vins`:

```bash
python3 ov_offline/diagnostics/replay_cold_start.py \
  --output /home/user/ego_data/vio_cold_diagnosis_20260929/baseline_01
```

Executed three times (`baseline_01` through `baseline_03`), all exit 1: shelf and cashier
symptoms detected. Individual native runs take approximately 1.5-23 seconds;
the complete six-run regression takes roughly a minute. Numerical paths differ
by less than 0.000002 m between these runs, with identical status counts.
The tool records each Docker command and checks all invalid matrices are NaN
and that removed boolean arrays remain absent. The final harness also records
hashes of input CSVs and configuration files for subsequent experiments.

Cold inputs are the original `cold_v2` CSVs/configurations with original videos,
not the preview. Camera indices remain original. Only selected pairs exist in
these diagnostic CSVs, so other video rows are `no_stereo_pair`, not
`not_requested`. This is distinct from the production full-pair-table window.
It does not alter the production frame-alignment contract.

## Controlled comparison

Each run starts a fresh estimator. `cold` uses window-only IMU; `history` changes
only the IMU CSV to the complete prepared stream; `full` additionally supplies
the complete stereo pair table. Configurations are identical across modes.
The offline loop itself starts IMU feeding about three seconds before the
first selected camera (`init_window_time + 1`), not at the recording beginning.

| Case / mode | Available poses | Path (m) | Final offset (ms) |
| --- | ---: | ---: | ---: |
| Shelf 19..23, cold | 0 | unavailable | -47.776 |
| Shelf 19..23, history | 74 | 0.371 | -38.868 |
| Shelf complete | 649 | 4.323 | -39.156 |
| Cashier 20..28, cold | 179 | 18.546 | -39.929 |
| Cashier 20..28, history | 194 | 1.446 | -7.376 |
| Cashier complete | 1290 | 8.922 | -7.229 |

Full cashier replay on the exact 179 cold-valid frame IDs: **0.824331 m**.
Shelf cold: 119 initializing, 1 IMU-unavailable among 120 selected pairs.
Cashier cold: 60 initializing, 1 IMU-unavailable among 240 selected pairs.
Neither a nonzero pose count nor exit zero implies accuracy.

## Hypotheses and isolation

Ranked before probing: (1) different first processed camera due to IMU coverage;
(2) tracker replenishment versus initializer feature requirements; (3) bad
timing/initial state followed by loss of visual updates; (4) insufficient
excitation or spatial calibration. The failures are investigated separately.

```bash
python3 ov_offline/diagnostics/replay_cold_start.py \
  --output /home/user/ego_data/vio_cold_diagnosis_20260929/boundary_01 \
  --modes history_skip_first boundary
```

`history_skip_first` removes only the first pair from the history experiment.
`boundary` adds only the last real IMU sample before the cold IMU CSV, allowing
the first camera to be bracketed. It is NOT strict cold start and is not proposed
as a fix or as synthetic input padding.

| Case / intervention | Available poses | Path (m) |
| --- | ---: | ---: |
| Shelf, history without first pair | 0 | unavailable |
| Shelf, cold plus one real boundary sample | 60 | 0.332 |
| Cashier, history without first pair | 193 | 5.839 |
| Cashier, cold plus one real boundary sample | 178 | 5.965 |

**Conclusion:** pre-window IMU versus cold is confounded by which image first
enters the tracker. Historical IMU alone does not cure shelf failure when that
first image is held consistent. Adding one boundary sample is not sufficient
to resolve the cashier anomaly.

## Shelf: initialization rejection

`tracking_01` changes logging only to ALL, preserving the reproduced results.
`VioManager.cpp` divides 50 initialization features across two cameras: target
25 per camera. In `TrackKLT.cpp::perform_detection_stereo`, replenishment occurs
only when the deficit exceeds `min(20, floor(0.5 * 25))`, i.e. 12. A camera with
16 surviving points does not request replenishment. Left-track logs settle at
16 and remain there; the initializer database decays from 36 to 19 unique IDs.

`DynamicInitializer.cpp` rejects database sizes below `0.75 * 50`, requiring at
least 38 unique IDs. Its warning incorrectly prints `0.95 * 50` (48), so the
printed threshold is not the predicate. Stereo observations share feature IDs;
per-camera observation counts cannot simply be added to obtain unique features.

This identifies a concrete replenishment/initialization budget mismatch to
test. It does NOT yet establish the appropriate general policy change. Do not
lower the initialization threshold. A synthetic stereo tracking regression
must exercise replenishment and the real unique-feature database before a fix.

This case exits before the dynamic initializer's angular-excitation test.
The window gyro RMS is 0.2829 rad/s, acceleration component-standard-deviation
norm 1.2120 m/s^2, first two seconds integrated gyro magnitude about 24.43 deg.
These do not prove full observability, but do not support calling this failure
an observed minimum-motion rejection.

## Cashier: initialization succeeds, constraints then vanish

Cold initial velocity is approximately `(0.4105, 0.3407, 0.0056)` m/s;
history-window initialization gives `(-0.0418, 0.0932, 0.0483)` m/s.
Cold initialized gyro bias is `(0.0575, 0.0374, -0.0210)` rad/s.
179 cold-valid frames contain 163 with both exported feature counts zero;
the last nonzero exported visual count is frame 765. The history run has only
20 such frames out of 194 and nonzero counts through frame 839. Counts are
diagnostics, not a substitute for inspecting individual update rejections.
Cold offset stalls near -41.25 ms, then -39.929 ms as visual constraints vanish.

`DynamicInitializer.cpp` uses `params.calib_camimu_dt` for IMU preintegration;
the later filter's online offset state does not calibrate this initialization.
`init_dyn_mle_opt_calib` is not a switch for optimizing initialization timing.
The initializer logs 101.797 degrees integrated rotation and a converged Ceres
solve, not a minimum-rotation or solver-timeout rejection. Convergence does not
establish a correct state under incorrect timing.

Independent existing visual-rotation/gyro diagnostic, extended only to select
windows, was run on original raw videos/calibration (host OpenCV 4.12.0):

```bash
python3 ov_offline/diagnostics/check_rotation_alignment.py \
  /home/user/ego_data/processed_data/session_20260827_113215_收银_20260903162506 \
  --start-seconds 20 --seconds 8
```

79 visual intervals favor -10 ms on a 10 ms grid: median rotation error 0.179
degrees versus 0.637 degrees at -50 ms. Shelf 19..23 (39 intervals) favors
-40 ms: 0.212 versus 0.236 degrees at -50 ms. These support a cashier timing
mismatch independently of the estimated trajectory, not an exact correction
or proof of its capture/calibration origin. Essential-matrix estimates can be
biased by translation and dynamic content; no production calibration was edited.
Window IMU spacing is approximately 5 ms with maxima below 5.8 ms in both cases.

## Next evidence required before an estimator fix

1. Instrument per-camera existing/new tracks and unique IDs to confirm the
   shelf replenishment dead zone, then write a synthetic stereo regression.
   Test a budget-consistent replenishment policy without reducing acceptance
   requirements. Re-run both cold cases and complete recordings.
2. For cashier, hold the camera/IMU boundaries fixed and isolate initialization
   timing from subsequent timing updates. A diagnostic timing counterfactual
   must not become a hard-coded per-session production correction.
3. Inspect triangulation and innovation rejection reasons after cashier startup,
   alongside temporal offset and biases. Determine whether failure is initiated
   by timing, track geometry, or another calibration error.
4. Validate any candidate on unchanged frame/status/NaN outputs and full clips.
   No pose clipping, history substitution, threshold relaxation, or filled poses.

The algorithm repair remains open. Current changes are diagnostic tooling and
this evidence record only. The delivery image and both output contracts remain
unchanged.
