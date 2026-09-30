"""Replay complete sessions through a pinned delivery image and audit symptoms.

Diagnostic only: thresholds flag human review, never edit poses or certify
accuracy. A ZIP next to an extracted session is not a second recording.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np


def manifest_alias(session):
    """Explicit read-only alias for relocated manifests; never rewrite inputs."""
    manifest = json.loads((session / "dataset_manifest/dataset_manifest.json").read_text())
    original = Path(manifest["root_dir"])
    if not original.is_absolute() or original.name != session.name or ".." in original.parts:
        raise ValueError("Manifest root must be an absolute path to this session")
    if not str(original).startswith(("/mnt/", "/home/", "/data/")):
        raise ValueError("Refusing to overlay a container system directory")
    for side in ("left", "right"):
        relative = Path(manifest["videos"][side]["path"]).relative_to(original)
        if relative != Path("videos") / f"{side}_rectified.mp4" or not (session / relative).is_file():
            raise ValueError("Manifest video does not match this session's rectified input")
    return str(original)


def measure(output):
    report = json.loads((output / "report.json").read_text())
    if report["status"] == "complete":
        assert {p.name for p in output.iterdir()} == {"trajectory.npz", "report.json", "debug"}
    with np.load(output / "trajectory.npz", allow_pickle=False) as data:
        status = data["status"]
        valid = status == "valid"
        assert data["schema_version"].item() == "openvins.frame_aligned.v2"
        assert not {"valid", "processed", "initialized"}.intersection(data.files)
        assert status.dtype.kind == "U" and status.ndim == 1
        assert np.array_equal(data["frame_index"], np.arange(len(status)))
        assert valid.sum() == report["pose_count"] == report["frame_alignment"]["valid_pose_count"]
        assert len(status) == report["frame_alignment"]["frame_count"]
        for key in ("T_world_camera", "T_world_imu"):
            matrices = data[key]
            assert matrices.shape == (len(status), 4, 4)
            assert np.isnan(matrices[~valid]).all() and np.isfinite(matrices[valid]).all()
            assert np.allclose(matrices[valid, 3], [0, 0, 0, 1])
            rotation = matrices[valid, :3, :3]
            assert np.allclose(rotation.transpose(0, 2, 1) @ rotation, np.eye(3), atol=1e-6)
            assert np.allclose(np.linalg.det(rotation), 1, atol=1e-6)
        ids = data["frame_index"][valid].copy()
        positions = data["T_world_camera"][valid, :3, 3].copy()
        rotations = data["T_world_camera"][valid, :3, :3].copy()
        times = data["timestamp_camera"][valid].copy()
        camera_times = data["timestamp_camera"].copy()
        assert np.isfinite(times).all() and np.all(np.diff(times) > 0)
        assert np.isfinite(data["timestamp_imu"][valid]).all()
    if len(ids) < 2:
        raise ValueError("No sustained estimates")
    step = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    speed = step / np.diff(times)
    relative = rotations[:-1].transpose(0, 2, 1) @ rotations[1:]
    angles = np.rad2deg(np.arccos(np.clip((np.trace(relative, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    with (output / "debug/run/trajectory/frame_status.csv").open() as handle:
        rows = list(csv.DictReader(handle))
    missing_updates = [row for row in rows if row["status"] == "valid" and
                       int(row["msckf_features"]) + int(row["slam_features"]) == 0]
    longest = current = 0
    for row in rows:
        empty = row["status"] == "valid" and int(row["msckf_features"]) + int(row["slam_features"]) == 0
        current = current + 1 if empty else 0
        longest = max(longest, current)
    result = {
        "execution_status": report["status"], "frame_count": len(status), "valid_count": len(ids),
        "status_counts": dict(zip(*[v.tolist() for v in np.unique(status, return_counts=True)])),
        "first_valid_frame": int(ids[0]), "last_valid_frame": int(ids[-1]),
        "first_valid_s": float(times[0] - camera_times[np.isfinite(camera_times)][0]),
        "valid_gaps": int(np.count_nonzero(np.diff(ids) != 1)),
        "path_m": float(step.sum()), "displacement_m": float(np.linalg.norm(positions[-1] - positions[0])),
        "max_step_m": float(step.max()), "max_speed_m_s": float(speed.max()),
        "p99_speed_m_s": float(np.percentile(speed, 99)), "max_rotation_step_deg": float(angles.max()),
        "zero_exported_visual_count_frames": len(missing_updates),
        "longest_zero_exported_visual_count_frames": longest,
        "offline": report["offline"], "review_flags": [],
    }
    flags = result["review_flags"]
    if report["status"] != "complete":
        flags.append("execution_not_complete")
    if result["valid_gaps"]:
        flags.append("gaps_after_initialization")
    if step.max() > 0.25 or speed.max() > 5 or angles.max() > 30:
        flags.append("large_pose_increment")
    if longest > 60:
        flags.append("long_zero_exported_visual_counts_not_proof_of_update_loss")
    return result, ids, positions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--modes", nargs="+", choices=("raw", "rectified"), default=("raw", "rectified"))
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--session", action="append", help="Optional exact directory names")
    parser.add_argument("--manifest-mount-alias", action="store_true",
                        help="For rectified mode, also mount each session at its validated original manifest root")
    args = parser.parse_args()
    root = args.data_root.resolve(strict=True)
    sessions = sorted(p for p in root.iterdir() if p.is_dir() and (p / "videos").is_dir())
    if args.session:
        sessions = [p for p in sessions if p.name in args.session]
        if set(args.session) != {p.name for p in sessions}:
            parser.error("Requested session not found")
    if not sessions:
        parser.error("No extracted sessions found")
    image = subprocess.check_output(["docker", "image", "inspect", args.image, "--format", "{{.Id}}"], text=True).strip()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    summary = {"image": image, "data_root": str(root), "quality_status": "not_validated", "runs": []}
    for session in sessions:
        for mode in args.modes:
            run_root = output / session.name / mode
            delivery = run_root / "delivery"
            delivery.mkdir(parents=True)
            command = ["docker", "run", "--rm", "--init", "--network", "none", "--user", f"{os.getuid()}:{os.getgid()}",
                       "-e", f"VIO_IMAGE_ID={image}", "--mount", f"type=bind,src={root},dst={root},readonly",
                       "--mount", f"type=bind,src={delivery},dst=/out", image,
                       "run", "--session", str(session), "--output_dir", "/out", "--input_mode", mode, "--debug"]
            if mode == "rectified" and args.manifest_mount_alias:
                command[command.index(image):command.index(image)] = [
                    "--mount", f"type=bind,src={session},dst={manifest_alias(session)},readonly"]
            (run_root / "command.json").write_text(json.dumps(command, indent=2) + "\n")
            hashes = {str(p.relative_to(session)): hashlib.sha256(p.read_bytes()).hexdigest()
                      for folder in ("timestamps", "calibrations", "imu", "dataset_manifest")
                      for p in sorted((session / folder).glob("*")) if p.is_file()}
            (run_root / "input_sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")
            with (run_root / "console.log").open("w") as log:
                code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT).returncode
            result = {"session": session.name, "mode": mode, "returncode": code}
            try:
                measured, ids, positions = measure(delivery)
                result.update(measured)
                source_count = len(np.loadtxt(session / "timestamps/left_timestamps.txt", ndmin=1))
                assert source_count == result["frame_count"], "Source timestamp/video frame count mismatch"
                if args.reference:
                    reference, old_ids, old_positions = measure(args.reference / session.name / mode / "delivery")
                    _, a, b = np.intersect1d(ids, old_ids, return_indices=True)
                    new_path = float(np.linalg.norm(np.diff(positions[a], axis=0), axis=1).sum())
                    old_path = float(np.linalg.norm(np.diff(old_positions[b], axis=0), axis=1).sum())
                    result["comparison"] = {"common_frames": len(a), "candidate_common_path_m": new_path,
                                            "baseline_common_path_m": old_path,
                                            "baseline_valid_count": reference["valid_count"],
                                            "baseline_review_flags": reference["review_flags"]}
                    if len(a) < 2 or new_path > max(1.5 * old_path, old_path + 1):
                        result["review_flags"].append("path_growth_against_baseline")
                    if result["valid_count"] < reference["valid_count"] - max(30, .05 * reference["valid_count"]):
                        result["review_flags"].append("availability_loss_against_baseline")
            except (AssertionError, ValueError, KeyError, OSError) as error:
                result["audit_error"] = repr(error)
            summary["runs"].append(result)
            (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
            print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    failed = any(r["returncode"] or r.get("audit_error") or r.get("review_flags") for r in summary["runs"])
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
