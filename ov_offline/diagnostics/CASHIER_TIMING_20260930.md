# Cashier cold start: controlled timing and visual rejection evidence

Historical checkpoint. The logging-only patch was archived during release
cleanup under `/home/user/ego_data/vio_release_audit_20260930/archived_probes/`;
it is not applied to production source. Current verification is in
`COLD_START_FIX_20260930.md` and `RELEASE_AUDIT_20260930.md`.

Status: timing sensitivity and visual-update failure reproduced and isolated;
no new production timing policy or default has been installed. Shelf starvation
fix remains unchanged. Neither the runtime image nor source recordings were
modified. No commits or pushes.

Artifacts: `/home/user/ego_data/vio_cashier_cold_20260930/`.
Runtime image: `sha256:4474a8157e80308ed46e72914614c5ee13e2b9fe9cbd1bb9639a347783c7ace8`.
Native/library baseline: previous shelf-fixed build under
`/home/user/ego_data/vio_cold_diagnosis_20260930/build/fix/`.
Per-run commands and binary/library/config/input hashes are retained.

## Hypotheses

1. Inappropriate initial effective timing causes a poor startup and prevents
   reliable visual corrections. Prediction: changing only the initial timing
   guess toward the independent visual/gyro alignment should restore updates.
2. The initial state makes candidate features fail triangulation or innovation
   checks. Prediction: there are incoming candidates, not just absent tracks,
   and rejection counters should distinguish those stages.
3. Feature selection/solver conditioning is independently unstable. Prediction:
   repeated aligned runs would still diverge or show materially different states.

The first two predictions are supported below; the third is not exhausted.
These checks do not establish the acquisition/cutting origin of the offset or
separate initialization sensitivity from all subsequent filter sensitivity.

## Controlled input boundary

Changing timing can change the first camera's IMU-coverage decision. To avoid
that confound, `cold_skip_first` removes the first already-unavailable pair
from both experimental pair tables. Both cashier runs feed the same 239 camera
pairs (frames 601..839) and the same cold-only IMU CSV. No pre-window IMU is added.
IMU scheduling naturally changes with the tested alignment; that is part of
the timing intervention, not an independent history change.

All video rows remain in diagnostic NPZs; omitted pairs have `no_stereo_pair`.
This is only a controlled experiment, not a change to delivered status policy
or permission to discard divergent poses.

The baseline still reproduces 18.546230 m / 179 available poses and exit 1 in
the differential diagnostic. The modified calibration is a copy with a common
37.775533 ms shift added to both cameras, preserving their relative offset,
all spatial calibration, estimator parameters, and online offset estimation.
The -10 ms value was selected by the earlier independent visual/gyro diagnostic,
not fitted to make a trajectory short. It remains a counterfactual, not a default.

```bash
python3 ov_offline/diagnostics/replay_cold_start.py \
  --output /home/user/ego_data/vio_cashier_cold_20260930/timing_baseline \
  --native-probe /home/user/ego_data/vio_cold_diagnosis_20260930/build/fix/openvins_offline_native \
  --reference /home/user/ego_data/vio_cold_diagnosis_20260929/baseline_03 \
  --modes cold_skip_first
```

For the timing counterfactual, use a fresh output and append
`--initial-offset-s -0.01`. Successful artifacts are `timing_minus10ms_v2`.

| Cashier measure | Original initial offset | -10 ms counterfactual |
| --- | ---: | ---: |
| Input camera pairs processed | 239 | 239 |
| Available poses | 179 | 181 |
| First available original frame | 661 | 659 |
| Last available original frame | 839 | 839 |
| Total path (m) | 18.546230 | 0.870339 |
| Path on identical frames 661..839 (m) | 18.546230 | 0.850595 |
| Final estimated offset (ms) | -39.9293 | -7.4116 |
| Available states with zero exported MSCKF/SLAM counts | 163 / 179 | 13 / 181 |

The original full-run path on frames 661..839 is 0.824331 m. This is not ground
truth. Initial gyro bias changes from `(0.0575, 0.0374, -0.0210)` to
`(0.0175, -0.0074, 0.0003)` rad/s; initial velocity changes from
`(0.4105, 0.3407, 0.0056)` to `(0.0963, 0.2279, 0.0036)` m/s.
Initialization is therefore not producing the same state under the two guesses.

## Rejection instrumentation

`cashier_msckf_probe.patch` adds counters only to `UpdaterMSCKF::update`.
An instrumented executable was compiled from that source and `offline.cpp`,
using `-rdynamic` and the shelf-fixed shared library. The patch was then removed
from production source. The diagnostic executable and identical copied library
remain under `probe/`; the patch remains under `diagnostics/` for reproduction.
Its marked log lines demonstrate that the instrumented update method executes.
Baseline and -10 ms outputs match their noninstrumented counterparts to less
than 1e-7 m in path, so these counters did not change the observed outcome.

Counters classify each rejected candidate once: insufficient observations,
failed triangulation, refinement failure after successful triangulation,
innovation rejection, or accepted. They cover MSCKF only, not SLAM. Calls with
an empty candidate list are not counted. A feature candidate is not a frame.

| MSCKF candidates | Original timing | -10 ms |
| --- | ---: | ---: |
| Input | 760 | 749 |
| Insufficient observations | 77 | 71 |
| Triangulation failed | 393 | 380 |
| Refinement failed after triangulation | 46 | 27 |
| Innovation rejected | 223 | 196 |
| Accepted | 21 | 75 |
| Accepted in last 60 nonempty calls | 0 | 18 |

Artifacts: `rejections_baseline` and `rejections_minus10ms`. This rules out
"there are simply no feature candidates" as an explanation of the entire
failure. It does not prove that every rejected feature was correct, nor justify
loosening geometric or innovation acceptance thresholds.

## Input adaptation and upstream provenance

For both shelf and cashier, prepared stereo timestamps equal the original left
timestamps selected by frame index (maximum numeric difference zero).
Prepared IMU timestamp, gyro, and acceleration fields equal the original CSV
values after deduplication (all maximum numeric differences zero). Counts:
shelf 4733 raw/unique/prepared samples; cashier 9014 in each.
No double application of a camera offset was observed in these prepared CSVs.
This comparison does not validate that each source video's image content was
assigned the correct source timestamp before it reached this repository.

Both sessions' metadata describe recovery and timestamp alignment. Cashier's
`cut_info.json` identifies source frame range [54813, 56173), t0=1827.108095 s,
1360 frames and a `reexport_gop30` source path. Shelf was also recovered and
reexported. Metadata alone does not prove a recovery/cutting bug. The original
uncut recordings or recovery/cutting implementation are needed to trace that
possibility; their location was requested from the user without blocking replay.

## Broader counterfactual checks

`timing_minus10ms_regression_retry` changes the same initial-offset factor for
history-window and full runs, without modifying production configurations:

| Case | Available poses | Path (m) | Final offset (ms) |
| --- | ---: | ---: | ---: |
| Shelf history | 70 | 0.362251 | -39.3676 |
| Shelf full | 650 | 4.321485 | -38.9899 |
| Cashier history | 194 | 1.031778 | -7.19375 |
| Cashier full | 1299 | 8.527004 | -7.11334 |

Shelf cold counterfactual has 61 available poses and 0.319542 m path, converging
back near -38 ms. Full-run IMU boundary counts change because the alignment
changes; these full runs are regression observations, not boundary-controlled
causal comparisons. No universal -10 ms default is justified by this table.

## Experiment errors and verification

- `timing_minus10ms` is an invalid experiment: PyYAML's indentless nested matrix
  sequences were rejected by OpenCV before replay. `OpenCVDumper` fixes only
  diagnostic serialization; `timing_minus10ms_v2` is the actual comparison.
- One host harness launch aborted before creating an output directory with a
  allocator error. A fresh retry completed all four history/full runs. This was
  not an observed native VIO failure; the host error was not root-caused here.
- Added tests for nested OpenCV YAML compatibility and rejection of nonfinite
  timing guesses before starting a replay. Production calibration files remain
  untouched. Source update instrumentation is removed; output contracts unchanged.
- Shelf-fixed native build with source offline tests and bundled integration/
  output tests: **32 passed, 1 skipped** (shelf private fixture explicitly enabled).
  `git diff --check` passed; integration repository remains clean.

## Repair boundary

Evidence now supports a startup timing mismatch interacting with initialization
and subsequent loss of visual constraints. Online calibration after startup is
not sufficient for this reproduced window. The safe next repair investigation
is a confidence-checked timing bootstrap before dynamic initialization, or an
upstream timestamp repair if provenance confirms one. It must infer alignment
from the available window, retain uncertainty when motion is uninformative,
and avoid hard-coded per-session offsets, hidden historical IMU, or relaxed
acceptance tests. No such production bootstrap is implemented by this checkpoint.
