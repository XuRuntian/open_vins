"""Pinned-image differential replay of the two private cold-start cases.

Diagnostic artifacts only. Does not tune the estimator or certify accuracy.
Exit 1 detects the recorded symptoms; exit 2 indicates a broken experiment.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import yaml


class OpenCVDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)


CASES = {
    "shelf": ("session_20260826_100651_整理货架_20260828102934", "start_19_duration_4"),
    "cashier": ("session_20260827_113215_收银_20260903162506", "start_20_duration_8"),
}


def measure(path):
    report = json.loads((path / "trajectory/run_report.json").read_text())
    with np.load(path / "trajectory/trajectory.npz", allow_pickle=False) as data:
        status = data["status"]
        valid = status == "valid"
        for key in ("T_world_imu", "T_world_camera"):
            assert np.isnan(data[key][~valid]).all(), key
        assert not {"valid", "processed", "initialized"}.intersection(data.files)
        ids = data["frame_index"][valid].copy()
        positions = data["T_world_camera"][valid, :3, 3].copy()
    return {
        "status_counts": dict(zip(*[a.tolist() for a in np.unique(status, return_counts=True)])),
        "path_m": float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
        "displacement_m": float(np.linalg.norm(positions[-1] - positions[0])) if len(ids) else None,
        "offline": report["offline"],
    }, ids, positions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=Path("/home/user/ego_data"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image", default="sha256:4474a8157e80308ed46e72914614c5ee13e2b9fe9cbd1bb9639a347783c7ace8")
    parser.add_argument("--modes", nargs="+", choices=("cold", "history", "full", "history_skip_first", "cold_skip_first", "boundary"),
                        default=("cold", "history", "full"))
    parser.add_argument("--verbosity", choices=("INFO", "DEBUG", "ALL"), default="INFO")
    parser.add_argument("--native-probe", type=Path,
                        help="Diagnostic/rebuilt binary; a sibling library overrides the image library")
    parser.add_argument("--reference", type=Path,
                        help="Immutable baseline replay output; compare every mode on common frame IDs")
    parser.add_argument("--initial-offset-s", type=float,
                        help="Counterfactual only: override initial timing in a copied calibration")
    args = parser.parse_args()
    if args.initial_offset_s is not None and not np.isfinite(args.initial_offset_s):
        parser.error("Initial offset must be finite")
    args.output.mkdir(parents=True, exist_ok=False)
    summary = {"image": args.image, "cases": {}}
    for name, (session, window) in CASES.items():
        cold = Path("/e/vio_window_audit_20260929/cold_v2") / session / window
        full = Path("/e/vio_init_audit_20260929_191449") / session / "debug/run"
        videos = Path("/e/processed_data") / session / "videos"
        runs = {}
        for mode in args.modes:
            output = args.output / name / mode
            output.mkdir(parents=True)
            pair_path = full / "input/stereo_pairs.csv" if mode == "full" else cold / "pairs.csv"
            imu_path = cold / "imu.csv" if mode in ("cold", "cold_skip_first", "boundary") else full / "input/imu_clean.csv"
            config_path = cold / "config/estimator_config.yaml"
            if args.verbosity != "INFO" or args.initial_offset_s is not None:
                shutil.copytree(args.data_root / cold.relative_to("/e") / "config", output / "config")
                config = output / "config/estimator_config.yaml"
                settings = yaml.safe_load(config.read_text().removeprefix("%YAML:1.0\n"))
                settings["verbosity"] = args.verbosity
                config.write_text("%YAML:1.0\n" + yaml.safe_dump(settings, default_flow_style=None, sort_keys=False))
                config_path = Path("/out/config/estimator_config.yaml")
                if args.initial_offset_s is not None:
                    calibration = output / "config/kalibr_imucam_chain.yaml"
                    cameras = yaml.safe_load(calibration.read_text().removeprefix("%YAML:1.0\n"))
                    delta = args.initial_offset_s - cameras["cam0"]["timeshift_cam_imu"]
                    for camera in cameras.values():
                        camera["timeshift_cam_imu"] += delta
                    calibration.write_text("%YAML:1.0\n" + yaml.dump(
                        cameras, Dumper=OpenCVDumper, default_flow_style=None, sort_keys=False))
            if mode in ("history_skip_first", "cold_skip_first"):
                with (args.data_root / cold.relative_to("/e") / "pairs.csv").open() as handle:
                    reader = csv.DictReader(handle)
                    fields, rows = reader.fieldnames, list(reader)[1:]
                with (output / "pairs.csv").open("w") as handle:
                    writer = csv.DictWriter(handle, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(rows)
                pair_path = Path("/out/pairs.csv")
            if mode == "boundary":
                with (args.data_root / cold.relative_to("/e") / "imu.csv").open() as handle:
                    reader = csv.DictReader(handle)
                    fields, rows = reader.fieldnames, list(reader)
                with (args.data_root / full.relative_to("/e") / "input/imu_clean.csv").open() as handle:
                    earlier = [row for row in csv.DictReader(handle)
                               if float(row["timestamp"]) < float(rows[0]["timestamp"])]
                with (output / "imu.csv").open("w") as handle:
                    writer = csv.DictWriter(handle, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(earlier[-1:] + rows)
                imu_path = Path("/out/imu.csv")
            command = ["docker", "run", "--rm", "--network", "none", "--user", f"{os.getuid()}:{os.getgid()}",
                       "--mount", f"type=bind,src={args.data_root.resolve()},dst=/e,readonly",
                       "--mount", f"type=bind,src={output.resolve()},dst=/out", args.image,
                       "python3", "-m", "ov_offline.run", "--binary", "/opt/open_vins/ov_msckf/build/openvins_offline_native",
                       "--config", str(config_path),
                       "--left_video", str(videos / "left_rectified.mp4"),
                       "--right_video", str(videos / "right_rectified.mp4"),
                       "--stereo_pairs", str(pair_path), "--imu", str(imu_path),
                       "--output_dir", "/out", "--time-offset-mode", "estimate"]
            if args.native_probe:
                probe = args.native_probe.resolve(strict=True)
                command[command.index(args.image):command.index(args.image)] = [
                    "--mount", f"type=bind,src={probe.parent},dst=/probe,readonly",
                    "-e", "LD_LIBRARY_PATH=/probe:/opt/open_vins/ov_msckf/build"]
                command[command.index("--binary") + 1] = str(Path("/probe") / probe.name)
                (output / "probe_sha256.txt").write_text(hashlib.sha256(probe.read_bytes()).hexdigest() + "\n")
                library = probe.parent / "libov_msckf_lib.so"
                if library.exists():
                    (output / "library_sha256.txt").write_text(hashlib.sha256(library.read_bytes()).hexdigest() + "\n")
            (output / "command.json").write_text(json.dumps(command, indent=2) + "\n")
            hashes = {}
            for source in (pair_path, imu_path, config_path, config_path.parent / "kalibr_imu_chain.yaml",
                           config_path.parent / "kalibr_imucam_chain.yaml"):
                local = (output / source.relative_to("/out") if source.is_relative_to("/out")
                         else args.data_root / source.relative_to("/e"))
                hashes[str(source)] = hashlib.sha256(local.read_bytes()).hexdigest()
            (output / "input_sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")
            with (output / "console.log").open("w") as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
            if result.returncode not in (0, 3):
                raise RuntimeError(f"Replay failed ({result.returncode}): {output}")
            stats, ids, positions = measure(output)
            stats["returncode"] = result.returncode
            runs[mode] = (stats, ids, positions)
            print(name, mode, json.dumps(stats), flush=True)
        summary["cases"][name] = {mode: run[0] for mode, run in runs.items()}
        if {"cold", "full"}.issubset(runs):
            cold_stats, cold_ids, cold_positions = runs["cold"]
            _, full_ids, full_positions = runs["full"]
            matched = np.isin(full_ids, cold_ids)
            assert np.array_equal(full_ids[matched], cold_ids)
            path = float(np.linalg.norm(np.diff(full_positions[matched], axis=0), axis=1).sum())
            summary["cases"][name]["full_on_cold_ids_path_m"] = path
            summary["cases"][name]["symptom_detected"] = (
                not len(cold_ids) if name == "shelf" else cold_stats["path_m"] > max(10 * path, 5.0))
        if args.reference:
            _, reference_ids, reference_positions = measure(args.reference / name / "full")
            comparisons = {}
            for mode, (_, ids, positions) in runs.items():
                common, candidate_index, reference_index = np.intersect1d(ids, reference_ids, return_indices=True)
                candidate_path = float(np.linalg.norm(np.diff(positions[candidate_index], axis=0), axis=1).sum())
                reference_path = float(np.linalg.norm(np.diff(reference_positions[reference_index], axis=0), axis=1).sum())
                anomaly = len(common) >= 2 and candidate_path > max(10 * reference_path, 5.0)
                comparisons[mode] = dict(common_frames=len(common), candidate_path_m=candidate_path,
                                         reference_path_m=reference_path, path_anomaly=anomaly)
                summary["cases"][name]["symptom_detected"] = (
                    summary["cases"][name].get("symptom_detected", False) or anomaly or not len(ids))
            summary["cases"][name]["reference_comparisons"] = comparisons
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    detected = [key for key, value in summary["cases"].items() if value.get("symptom_detected")]
    assessed = [key for key, value in summary["cases"].items() if "symptom_detected" in value]
    print("Symptom checks assessed:", assessed, "detected:", detected)
    raise SystemExit(1 if detected else 0)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, AssertionError, RuntimeError) as exc:
        print(f"Experiment error: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
