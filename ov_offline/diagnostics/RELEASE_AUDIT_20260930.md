# Full-session release audit, 2026-09-30

## Scope and fixed inputs

User requested all recordings in `/home/user/ego_data/processed_data` be replayed
before cleanup, commit and release build. There are four extracted sessions;
the sibling ZIP duplicates one session and is not counted as another recording.
Both raw and rectified modes were tested through the public container `run`
entry, with only data/output mounts and networking disabled. No estimator
parameters were changed during this audit.

Baseline image: `sha256:4474a8157e80308ed46e72914614c5ee13e2b9fe9cbd1bb9639a347783c7ace8`.
Candidate image: `sha256:636fc8e9b200267534d11c219708bc38069c35d029255ae4cb40ba0aad102482`.
Audit root: `/home/user/ego_data/vio_release_audit_20260930/`.
Per-run commands, input metadata hashes, logs, debug states and full NPZs are
retained in `baseline_mounted` and `candidate`; each contains `summary.json`.

## Input path issue, not a VIO failure

The first pass (`baseline`) discovered that rectified manifests still contain
absolute `/mnt/nas/...` paths from the original machine. Their four rectified
runs failed before estimation. Raw runs completed. These failed attempts are
retained and are not counted as algorithm regressions or successful replays.

The verified reruns explicitly mount each local session read-only at its
original manifest root as well. `--manifest-mount-alias` validates the root
session name and both expected video paths before adding that alias. Neither
source manifests nor recordings nor calibration were rewritten. This is the
documented rectified-container path requirement, not a new algorithm fix.

## Results

All eight corrected candidate runs complete. Every input video frame remains
in the NPZ (7,010 source frames, tested in each mode). Invalid transforms are
all NaN, valid transforms are finite rigid transforms, timestamps increase,
status counts agree with reports, and there are no redundant validity arrays.
Every run has a continuous valid tail through the last video frame.

| Session suffix | Mode | Baseline valid / path m | Candidate valid / path m | Candidate first valid s |
| --- | --- | --- | --- | ---: |
| Shelf 20260827190358 | raw | 2071 / 17.408844 | 2071 / 17.408840 | 2.200 |
| Shelf 20260827190358 | rectified | 2075 / 18.704449 | 2075 / 18.683074 | 2.067 |
| Shelf 20260828102934 | raw | 652 / 3.786154 | 652 / 3.786154 | 2.067 |
| Shelf 20260828102934 | rectified | 649 / 4.323159 | 649 / 4.309932 | 2.167 |
| Packing 20260831144710 | raw | 2737 / 23.088412 | 2737 / 23.088412 | 2.067 |
| Packing 20260831144710 | rectified | 2737 / 25.665843 | 2737 / 25.665842 | 2.067 |
| Cashier 20260903162506 | raw | 1298 / 8.779702 | 1272 / 7.422920 | 2.933 |
| Cashier 20260903162506 | rectified | 1290 / 8.921578 | 1287 / 8.594048 | 2.433 |

Across the candidate matrix, maximum frame-to-frame translation is 5.91 cm,
maximum finite-difference speed is 1.771 m/s, and maximum rotation increment is
12.23 degrees. No gross-increment audit alarm fired. Consecutive valid states
with zero exported MSCKF/SLAM counts last at most 11 frames; these counts are
only a diagnostic proxy, not proof that a filter update did/did not happen.

Cashier raw starts 26 frames (0.867 s) later than baseline; rectified starts
three frames (0.100 s) later. These are explicit availability tradeoffs, not
removed output rows or pose clipping. Bootstrap is offline look-ahead, so the
first valid timestamp is not real-time startup latency. To avoid attributing
shorter paths merely to omitted startup estimates, common-frame paths were
compared: cashier raw 8.0740 -> 7.4229 m, rectified 8.8790 -> 8.5940 m. The
largest old/new position difference after aligning the first common camera
pose is 0.302 m (raw cashier); no scale fitting was used. This is a differential
comparison, not an accuracy error, because neither trajectory is ground truth.

Audit review thresholds (step >0.25 m, speed >5 m/s, rotation step >30 degrees,
long zero-count stretches, common-frame path growth, significant availability
loss) are deliberately symptom checks for these recordings. They are not
production filters and never remove samples. Passing them does not certify
absolute trajectory accuracy or cover all possible VIO failures.

## Cleanup and tests

- No algorithm changes after the tested candidate; cleanup adds audit tooling,
  documentation and tests only.
- Rejected budget probes and logging patch are preserved outside production
  source in `archived_probes/` under the audit root. Existing untracked build,
  install and log directories remain untouched.
- Maintained reproductions: `replay_cold_start.py`, `audit_runtime.py`, and the
  independent diagnostic `check_rotation_alignment.py`.
- Source offline tests, both private cold regressions, bundled project tests
  and Docker output tests: **39 passed, 1 skipped**. The remaining skip is the
  older optional cashier-manifest test, not either fixed cold window.
- Native timing tests pass, including an independent constant-axis integration
  check for sign, extrinsic rotation and fractional IMU endpoints.
- Bootstrap-disabled negative control reproduces the known cashier cold drift;
  the passing cold cases remain recorded in `COLD_START_FIX_20260930.md`.
- No production probe instrumentation or pose clipping is installed. Source
  calibration and default output schema/artifacts are unchanged.

## Reproduction and release verification

```bash
PYTHONDONTWRITEBYTECODE=1 python3 ov_offline/diagnostics/audit_runtime.py \
  --data-root /home/user/ego_data/processed_data \
  --output /home/user/ego_data/vio_release_audit_20260930/NEW_OUTPUT \
  --image IMAGE_ID --manifest-mount-alias \
  --reference /home/user/ego_data/vio_release_audit_20260930/baseline_mounted
```

Use a fresh output directory. The audit explicitly requests debug artifacts;
normal delivery still publishes only NPZ/report on successful runs.

The release build is to be made after committing both repositories, with image
revision labels equal to those commits (no `-dirty`). The final image ID,
bundled tests and repeated full-session audit are recorded alongside the audit
artifacts after build verification, avoiding self-referential commit hashes.
The intended immutable tag is `roboego-vio-runtime:20260930-cold-start`.
Do not promote `:dev` or overwrite an old export until final verification passes.
