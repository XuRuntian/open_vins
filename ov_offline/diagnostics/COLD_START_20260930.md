# Shelf initialization: replenishment starvation, not image clarity

Historical checkpoint. During release cleanup the experimental C++ probes were
archived, not deleted, under
`/home/user/ego_data/vio_release_audit_20260930/archived_probes/`.
They are not production entrypoints; use their original pinned baseline when
reproducing the experiments below. Current verification is in
`COLD_START_FIX_20260930.md` and `RELEASE_AUDIT_20260930.md`.

Follow-up to `COLD_START_20260929.md`. User confirmed the shelf video is clear.
This investigation does not equate a low feature database count with blur or
lack of scene texture. Cashier drift remains a separate unresolved issue.

Artifacts: `/home/user/ego_data/vio_cold_diagnosis_20260930/`.
Same immutable runtime image and source baseline as the previous checkpoint.
All probes disable networking and mount recordings read-only. The runtime tag
and the integration repository are unchanged; no commits or pushes.

## Direct tracking evidence

`track_budget_probe.cpp` links against the original image's tracking library.
It replays the frozen lossless shelf fixture starting at local frame 1, matching
the original first-frame IMU rejection. It records tracked IDs and window DB
size, and independently detects features on each current stereo pair with a
fresh tracker using exactly the original calibration and detection parameters.
It does not inject these fresh detections into the continuous tracker.

Two executions produced byte-identical CSVs (`shelf_tracks.csv` and
`shelf_tracks_repeat.csv`). The tracker-only replay is not an estimator replay;
its window cleanup directly uses the two-second window. Its left-track and
unique-DB counts agree with the observed initialization failure.

| Local frame | Left tracks | Right tracks | Shared IDs | Unique live IDs | Window DB | Fresh unique IDs |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 45 | 32 | 32 | 45 | 0 | 45 |
| 2 | 33 | 27 | 24 | 36 | 36 | 45 |
| 11 | 16 | 20 | 13 | 23 | 36 | 46 |
| 61 | 16 | 16 | 13 | 19 | 36 | 49 |
| 119 | 16 | 16 | 13 | 19 | 19 | 48 |

The initial frame detects points but does not yet write temporal observations
to the database. No new IDs enter the continuous tracker after the first frame.
Fresh detection finds **39-50 unique IDs** across these same images. Those
detections alone do not prove long-lived geometric validity; the subsequent
unchanged-initializer replay below provides that additional evidence.

Code mechanism:

1. `VioManager` divides `init_max_features=50` by two cameras: target 25 per camera.
2. Stereo KLT replenishes only if `25 - current_count > 12`.
   With 16 surviving points per camera, neither detector replenishes.
3. Thirteen IDs are shared between the two cameras: 16 + 16 - 13 = 19,
   not 32 independent features.
4. Dynamic initialization requires at least 38 unique database entries
   (`size >= 0.75 * 50`). Historical tracks age out, so the system remains
   below the gate even though the images still provide detectable features.

This is a demonstrated feature-budget/replenishment failure in this case.
It is not an observed IMU motion threshold rejection or proof of bad images.

## Counterfactuals

Both probes change only the initialization tracker budget, never the initializer
acceptance thresholds, camera/IMU timing, input bounds, or post-init target.
They subclass the original image's manager inside a diagnostic executable;
all estimator library code is unchanged.

- `initial_budget_probe.cpp`: 50 rather than 25 features per camera from startup.
- `starvation_budget_probe.cpp`: increase to 50 only after an unsuccessful
  initialization with a nonempty DB containing fewer than 38 unique IDs.

| Case | Original | Unconditional budget | Starvation feedback |
| --- | --- | --- | --- |
| Shelf cold | 0 poses | 59 poses, 0.319 m | 59 poses, 0.324 m |
| Shelf history | 74 poses, 0.371 m | 69 poses, 0.360 m | 70 poses, 0.364 m |
| Shelf full | 649 poses, 4.323 m | 650 poses, 4.323 m | 649 poses, 4.323 m |
| Cashier cold | 179 poses, 18.546 m | 130 poses, 0.669 m | 179 poses, 18.546 m |
| Cashier history | 194 poses, 1.446 m | 181 poses, **31.362 m** | 194 poses, 1.446 m |
| Cashier full | 1290 poses, 8.922 m | 1297 poses, 9.504 m | 1290 poses, 8.922 m |

**Rejected:** unconditional increase, despite improving the two cold outputs.
Cashier history becomes severely worse. Fewer available poses in the cold
counterfactual also mean its shorter path must not be compared without matching
frame IDs. On the unconditional history run's 181 IDs, original full replay is
0.843 m, versus 31.362 m in the counterfactual.

The starvation feedback restores shelf initialization without changing the
cashier paths in this matrix. Its shelf cold path on 59 available IDs is 0.324 m
versus 0.311 m in the original full replay on those IDs. This is a plausibility
comparison, not ground-truth accuracy certification.

The original harness only flagged the two recorded cold symptoms and would
miss the newly introduced history regression. `--reference` now additionally
compares every replay mode against the immutable full-run baseline on common
frame IDs. Its generous path alarm is not an accuracy or coverage acceptance
criterion. Reports retain counts and the native outputs without clipping.

## Scoped implementation and regression

`VioManager.cpp` applies starvation feedback only to synchronous, two-camera
stereo KLT dynamic initialization. The nonempty feature database must be below
the existing initializer gate, and its divided target must still be below the
requested initialization feature budget. Normal tracking resets its target via
the existing successful-initialization path. No asynchronous initialization
thread is inspected or modified by this fallback.

This is a bounded recovery for the demonstrated dead zone, not a claim that
every low-feature scene can or should initialize. Monocular, descriptor,
asynchronous, and static-only paths retain their previous behavior.

`test_cold_start_regression.py` uses the real frozen 120-frame fixture and real
native replay. Baseline: **1 failed**, `not_initialized`, exit 3. The independent
budget counterfactual: **1 passed**. The test requires sustained valid estimates
through the final frame, unchanged first-frame IMU rejection, all 120 rows,
v2 status-only schema, and entirely NaN matrices for unavailable estimates.
It neither enables stationary initialization nor supplies historical IMU.

Formal rebuilt-source verification:

- Fresh ROS-free Release build in the pinned container passed (37 build steps).
  Build directory: `build/fix/` under the artifact root. `ldd` confirmed the
  rebuilt binary loads `/probe/libov_msckf_lib.so`, not the original image library.
- Source offline tests plus bundled integration/output tests: **30 passed,
  1 skipped**. Shelf private fixture was explicitly enabled and passed; the
  separate old cashier private test was not supplied in this suite.
- `fix_01/`: six runs using the rebuilt executable/library and immutable
  `baseline_03` reference. Shelf cold: 59 valid, 60 initializing, 1 IMU-unavailable;
  path 0.324134 m. All 120 frozen-fixture rows are also checked by the private test.
  Shelf history: 70 valid, 0.363546 m. Cashier cold remains 18.546230 m / 179 valid;
  history remains 1.445944 m / 194 valid. Overall diagnostic exit is correctly 1,
  naming cashier, not a claim that both failures have been repaired.
- Four full rectified recordings were run with original and fixed code:
  target shelf (649 valid), cashier (1290), other shelf (2075), packing (2737).
  Complete status arrays match their respective baselines exactly. Full paths
  remain 4.323159, 8.921579, 18.704444, 25.665842 m respectively.
  These numerical comparisons do not establish absolute trajectory accuracy.
- NaN/status-only checks and `git diff --check` passed. No production debug
  instrumentation remains; counterfactual executables are retained explicitly
  under `diagnostics/`. Existing image `roboego-vio-runtime:dev` was NOT replaced.

The new private test is skipped unless `VIO_SHELF_COLD_FIXTURE` is explicitly
supplied. It exercises the actual failure path instead of substituting a
synthetic scene that might fail for a different reason.

## Reproduction

From the OpenVINS checkout, build the diagnostic executables against the pinned
image's library using C++17, include paths `ov_core/src`, `ov_init/src`,
`ov_msckf/src`, Eigen, and OpenCV; link `ov_msckf_lib`, OpenCV, Boost system,
filesystem, thread, date_time, and pthread. Place them in the artifact `build/`
directory. They are explicitly experimental, not production entrypoints.

```bash
python3 ov_offline/diagnostics/replay_cold_start.py \
  --output /home/user/ego_data/vio_cold_diagnosis_20260930/starvation_budget_repeat \
  --native-probe /home/user/ego_data/vio_cold_diagnosis_20260930/build/starvation_budget_probe \
  --reference /home/user/ego_data/vio_cold_diagnosis_20260929/baseline_03
```

Expected overall result is still red because cashier cold drift remains.
Every output directory contains the exact Docker command, CSV/config hashes,
binary hash, and sibling library hash when using a rebuilt library. Never reuse
a nonempty output directory or replace the baseline image to run these probes.
