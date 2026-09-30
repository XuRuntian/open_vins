# Cold-start repair candidate, 2026-09-30

This supersedes the repair-boundary checkpoint in `CASHIER_TIMING_20260930.md`.
It addresses two reproduced failures, not all possible VIO failures or the
unknown upstream origin of the cashier timing discrepancy. No commits/pushes.
Original recordings, calibrations, baseline image and old export remain intact.

## Changes and causal boundary

1. The existing synchronous stereo/KLT shortage-feedback fix replenishes
   features only when the cleaned unique-ID database is below the unchanged
   dynamic-initializer gate. It does not equate clear images with enough tracked
   IDs. See `COLD_START_20260930.md` for red/green tracker and full-run evidence.
2. The offline adapter now estimates startup timing before constructing
   `VioManager` when `--time-offset-mode estimate` and bootstrap is `auto`.
   OpenCV KLT forward/backward checks and essential-matrix RANSAC provide short
   visual rotations. Gyro predictions use the loaded IMU intrinsic corrections,
   camera rotation and piecewise SO(3) integration. A bounded local lag search
   removes a constant residual bias and checks the two temporal halves.
3. Accepted timing is installed in both the initializer options and filter
   options. The prior online-only policy changed the filter offset after a
   potentially bad initialization; it did not repair its original preintegration.
   Neither input timestamps nor source calibration nor spatial extrinsics change.
4. Invalid/uninformative/ambiguous fits leave calibration unchanged and report
   a reason. No VIO acceptance gate was weakened, and no estimated pose is
   removed, copied, interpolated or shortened after estimation.

Bootstrap uses up to six seconds of the requested camera window, not the full
recording when a short window was requested. It does not import historical IMU
from another file. The original images are replayed again afterward. This is
explicitly offline look-ahead; timestamps of first valid poses are not real-time
startup latency. IMU-covered boundary frames can change when timing changes.

## Red/green regression

`test_cashier_cold_start_does_not_reproduce_runaway_path` was first run against
the shelf-fixed native/library in `vio_cold_diagnosis_20260930/build/fix`:

```
AssertionError: Reproduced cold-start runaway: 18.546230 m
1 failed, 1 deselected in 4.14s
```

It now passes with the timing candidate. It requires a sustained final sequence,
all video rows, invalid NaNs, no redundant validity arrays, and a path below 2 m
on the available tail of frames 661..839. This catches catastrophic drift, not
accuracy. The availability assertion allows a few-frame initialization shift
instead of prescribing the baseline initializer's first frame. Shelf first-frame
status is checked against actual initial alignment/IMU coverage, not frozen to
the old offset's boundary decision. Existing synthetic boundary tests remain.

Native synthetic tests cover known positive/negative offsets with bias/outliers,
correct-calibration retention, out-of-range lag, inconsistent temporal halves,
unobservable constant rotation, empty inputs, endpoint extrapolation and IMU gaps.
They run in the runtime build. Private data is opt-in and not added to the image.

## Fixed-input observations

Pinned runtime used as build/replay environment:
`sha256:4474a8157e80308ed46e72914614c5ee13e2b9fe9cbd1bb9639a347783c7ace8`.
Candidate native/library: `vio_cold_diagnosis_20260930/build/timing/`.
Records (commands, input/config/probe hashes, reports, logs, NPZ):
`/home/user/ego_data/vio_cashier_cold_20260930/bootstrap_final/`.
Earlier repeated observations: `bootstrap_01`, `bootstrap_regression_01`.

| Case/mode | Available poses | Path (m) | Initial offset (ms) | Bootstrap |
| --- | ---: | ---: | ---: | --- |
| Shelf strict cold | 60 | 0.321489 | -33.776 | accepted |
| Shelf cold, first pair excluded in both trials | 59 | 0.324134 | -47.776 | retained calibration |
| Shelf with historical IMU | 70 | 0.362112 | -33.776 | accepted |
| Cashier strict cold | 176 | 0.829028 | -7.776 | accepted |
| Cashier cold, first pair excluded in both trials | 181 | 0.867072 | -5.776 | accepted |
| Cashier with historical IMU | 194 | 1.032087 | -7.776 | accepted |

The strict cashier output starts at frame 664 (three frames later than the old
frame 661); no frames are dropped from NPZ. To avoid making that availability
change look like a repair by omission, the controlled `cold_skip_first` replay
feeds the same 239 pairs as its baseline. On exactly the same **179 frames
661..839**, its path is **0.847708 m**, versus **18.546230 m** before, and
**0.824331 m** for the historical full-run reference. Full-run reference is not
ground truth. No -10 ms per-session/default constant is installed.

Cashier strict-cold bootstrap: 56 usable intervals; fitted lag -7.776 ms;
median bias-centered rotation-rate error 0.101891 -> 0.010157 rad/s; temporal
half estimates agree on the 2 ms grid. Final filter lag is -7.300 ms. Shelf
and packing do not get forced to the cashier lag; the controlled shelf and
packing full runs explicitly retain calibration when evidence is insufficient.

## Full-recording regression

Other full-run records: `vio_cashier_cold_20260930/bootstrap_other_full/`.
All four produce complete, finite available tails with full-frame invalid NaNs.

| Recording | Baseline poses/path (m) | Candidate poses/path (m) |
| --- | --- | --- |
| Target shelf | 649 / 4.323159 | 649 / 4.309932 |
| Cashier | 1290 / 8.921579 | 1287 / 8.594047 |
| Other shelf, ending 20260827190358 | 2075 / 18.704444 | 2075 / 18.683074 |
| Packing, ending 20260831144710 | 2737 / 25.665842 | 2737 / 25.665843 |

Cashier full initialization also moves by a few frames; this is recorded rather
than hidden. Regression bounds are symptom detectors, not a pose-quality score.
Two complete repetitions of the target cases differ by less than 1e-6 m in path.

## Reproduction

From `/home/user/open_vins`, with the rebuilt native and sibling library:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 ov_offline/diagnostics/replay_cold_start.py \
  --output /home/user/ego_data/vio_cashier_cold_20260930/NEW_OUTPUT \
  --native-probe /home/user/ego_data/vio_cold_diagnosis_20260930/build/timing/openvins_offline_native \
  --reference /home/user/ego_data/vio_cold_diagnosis_20260929/baseline_03 \
  --modes cold cold_skip_first history full
```

`ov_offline/README.md` describes native unit and private regression commands.
Source tests with both private cases enabled, project integration tests and
updated Docker delivery tests: **35 passed, 1 skipped**. The skip is the older
optional cashier-manifest regression, not either fixed cold-window regression.
`git diff --check` passes in both repositories. The project repository changes
are restricted to `docker/`; each repository is on `fix/vio-cold-start`.

## Self-contained runtime verification

Built `roboego-vio-runtime:cold-start`:
`sha256:636fc8e9b200267534d11c219708bc38069c35d029255ae4cb40ba0aad102482`.
The existing `roboego-vio-runtime:dev` still identifies baseline `4474a8157e80`;
the existing export under `/home/user/docker-images/vio/` was not replaced.
Both revision labels correctly carry `-dirty`; no code was committed.

The candidate's doctor passes. Its bundled tests, with only private data mounted
(no source/build mounts), give **35 passed, 1 skipped**. The native timing unit
executable also passed during the image build. A negative control using this
same image with `OPENVINS_TIMING_BOOTSTRAP=off` reproduces **18.546230 m** and
fails the cashier cold regression as intended. This separates the timing repair
from the shelf patch, packaging, output export and unrelated changes.

Additional public `run` entry smoke checks, with only session/output mounts:

| Run | Frames / available poses | Path (m) | Files |
| --- | --- | ---: | --- |
| Cashier raw full | 1360 / 1272 | 7.422915 | NPZ + report only |
| Shelf raw 19..23 s, historical IMU | 714 / 51 | 0.299358 | NPZ + report + requested debug directory |

Records: `runtime_cashier_raw_full` and `runtime_shelf_raw_window` under the
cashier diagnostic root. The shelf `run --start_s` invocation is deliberately
classified as historical-IMU replay, not strict cold start. These raw runs are
entry/output smoke tests, not direct comparisons to the rectified numbers above.
NPZ schema, exact frame index coverage, status-only validity, finite available
matrices and all-NaN unavailable matrices were programmatically checked.

## Remaining limitations

The guard thresholds are engineering checks, not calibrated probabilities.
Incorrect intrinsics/extrinsics, dynamic scenes, degenerate visual geometry,
time-varying delay or false essential-matrix rotations can still defeat them.
Rejected bootstrap leaves the original behavior available, including potential
failure/drift; a successful bootstrap does not certify an accurate trajectory.
The original/recovered video provenance is still unavailable, so acquisition
versus recovery/cutting origins of the effective lag remain unproven.

Prevention: test strict cold starts separately from windows with retained IMU;
track unique stereo IDs rather than equating two per-camera counts to the
initializer database; initialize timing before preintegration when recording
timing can differ from calibration; retain status/coverage and quality caveats.
