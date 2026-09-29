from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import yaml
import cv2

try:
    from .frame_export import align_frames
    from .trajectory import camera_pose_from_imu_pose, quaternion_xyzw_to_rotation, trajectory_metrics
except ImportError:
    from frame_export import align_frames
    from trajectory import camera_pose_from_imu_pose, quaternion_xyzw_to_rotation, trajectory_metrics


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a ROS-free OpenVINS offline binary from a prepared manifest.")
    parser.add_argument("--binary", required=True, type=Path)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--manifest", type=Path)
    source.add_argument("--config", type=Path)
    for name in ("left_video", "right_video", "stereo_pairs", "imu", "output_dir"):
        parser.add_argument("--" + name, type=Path)
    parser.add_argument("--start_s", type=float, default=0.0)
    parser.add_argument("--duration_s", type=float, default=None)
    parser.add_argument("--time-offset-mode", choices=("estimate", "config"),
                        default=os.environ.get("OPENVINS_TIME_OFFSET_MODE", "estimate"),
                        help="Estimate camera/IMU offset online, or honor calib_cam_timeoffset in the YAML")
    args = parser.parse_args(argv)
    direct = ("left_video", "right_video", "stereo_pairs", "imu", "output_dir")
    if args.config and any(getattr(args, name) is None for name in direct):
        parser.error("--config requires --left_video, --right_video, --stereo_pairs, --imu and --output_dir")
    if args.manifest and any(getattr(args, name) is not None for name in direct):
        parser.error("Do not combine manifest input with individual input paths")
    return args


def load_yaml(path: Path) -> dict:
    text = path.read_text()
    if text.startswith("%YAML:"):
        text = text.split("\n", 1)[1]
    result = yaml.safe_load(text)
    if not isinstance(result, dict):
        raise ValueError(f"Expected YAML mapping: {path}")
    return result


def load_inputs(args: argparse.Namespace) -> dict:
    if args.manifest is not None:
        path = args.manifest.expanduser().resolve()
        manifest = json.loads(path.read_text(encoding="utf-8"))
        base = path.parent
    else:
        config = args.config.expanduser().resolve()
        settings = load_yaml(config)
        cameras = load_yaml(config.parent / settings["relative_config_imucam"])
        manifest = {"output_dir": str(args.output_dir), "estimator_config_path": str(config),
                    "videos": {"left": {"path": str(args.left_video)}, "right": {"path": str(args.right_video)}},
                    "stereo_pairs_path": str(args.stereo_pairs), "imu_clean_path": str(args.imu),
                    "T_imu_camera0": cameras["cam0"]["T_imu_cam"]}
        base = Path.cwd()
    def resolve(value):
        path = Path(value).expanduser()
        return str((base / path).resolve())
    for key in ("output_dir", "estimator_config_path", "stereo_pairs_path", "imu_clean_path"):
        manifest[key] = resolve(manifest[key])
    for side in ("left", "right"):
        manifest["videos"][side]["path"] = resolve(manifest["videos"][side]["path"])
    extrinsic = np.asarray(manifest["T_imu_camera0"], dtype=float)
    if (extrinsic.shape != (4, 4) or not np.isfinite(extrinsic).all()
            or not np.allclose(extrinsic[3], [0, 0, 0, 1])
            or not np.allclose(extrinsic[:3, :3].T @ extrinsic[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(extrinsic[:3, :3]), 1, atol=1e-6)):
        raise ValueError("T_imu_camera0 must be a finite rigid transform")
    return manifest


def export_states(path: Path, manifest: dict) -> list[dict]:
    poses = []
    if not path.is_file():
        return poses
    with path.open() as handle:
        for row in csv.DictReader(handle):
            timestamp = float(row["timestamp"])
            timestamp_imu = float(row["timestamp_imu"])
            position = np.array([float(row[key]) for key in ("px", "py", "pz")])
            quaternion = np.array([float(row[key]) for key in ("qx", "qy", "qz", "qw")])
            velocity = [float(row[key]) for key in ("vx", "vy", "vz")]
            legacy_quaternion = [float(row[key]) for key in ("legacy_qx", "legacy_qy", "legacy_qz", "legacy_qw")]
            if not np.isfinite([timestamp, timestamp_imu, *position, *quaternion, *velocity, *legacy_quaternion]).all():
                raise ValueError("Non-finite offline state")
            if poses and timestamp <= poses[-1]["timestamp_camera"]:
                raise ValueError("Offline pose timestamps are not strictly increasing")
            world_imu = np.eye(4)
            world_imu[:3, :3] = quaternion_xyzw_to_rotation(quaternion)
            world_imu[:3, 3] = position
            world_camera, camera_quaternion = camera_pose_from_imu_pose(
                position, quaternion, np.asarray(manifest["T_imu_camera0"]))
            poses.append({"timestamp_camera": timestamp, "timestamp_imu": timestamp_imu,
                          "frame_index": int(row["left_frame_index"]),
                          "right_frame_index": int(row["right_frame_index"]),
                          "position_world_imu": position.tolist(),
                          "velocity_world_imu": velocity,
                          "orientation_world_imu_xyzw": legacy_quaternion,
                          "time_alignment_error_s": 0.0,
                          "T_world_imu": world_imu.tolist(), "T_world_camera": world_camera.tolist(),
                          "orientation_world_camera_xyzw": camera_quaternion.tolist()})
    return poses


def run(args: argparse.Namespace, *, output_dir: Path | None = None, legacy_report: bool = False) -> int:
    manifest = load_inputs(args)
    output_dir = (output_dir.expanduser().resolve() if output_dir is not None
                  else Path(manifest["output_dir"]) / "trajectory")
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"Use a fresh output directory; refusing to mix trajectories: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(args.binary.expanduser().resolve()),
        "--config",
        str(Path(manifest["estimator_config_path"]).expanduser().resolve()),
        "--left_video",
        str(Path(manifest["videos"]["left"]["path"]).expanduser().resolve()),
        "--right_video",
        str(Path(manifest["videos"]["right"]["path"]).expanduser().resolve()),
        "--stereo_pairs",
        str(Path(manifest["stereo_pairs_path"]).expanduser().resolve()),
        "--imu",
        str(Path(manifest["imu_clean_path"]).expanduser().resolve()),
        "--output_dir",
        str(output_dir),
        "--start_s",
        str(args.start_s),
    ]
    if args.duration_s is not None:
        command.extend(["--duration_s", str(args.duration_s)])
    mode = getattr(args, "time_offset_mode", "estimate")
    if mode not in {"estimate", "config"}:
        raise ValueError("time_offset_mode must be estimate or config")
    command.extend(["--time-offset-mode", mode])
    started = time.monotonic()
    report = {"schema_version": "roboego.vio.offline.v1", "status": "failed", "command": command,
              "pose_count": 0, "covariance_exported": False}
    if legacy_report:
        report["schema_version"] = "1.0"
    report.update(backend="openvins_offline", initialized=False,
                  published_stereo_pairs=0, published_imu_samples=0)
    try:
        with (output_dir.parent / "openvins.log").open("w") as log:
            with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as process:
                for line in process.stdout:
                    print(line, end="", flush=True)
                    log.write(line)
                    log.flush()
                code = process.wait()
        report["returncode"] = code
        native_report = output_dir / "offline_report.json"
        if native_report.is_file():
            report["offline"] = json.loads(native_report.read_text())
            report["published_stereo_pairs"] = report["offline"]["processed_camera_frames"]
            report["published_imu_samples"] = report["offline"]["published_imu_samples"]
        poses = export_states(output_dir / "states.csv", manifest)
        report["pose_count"] = len(poses)
        report["initialized"] = bool(poses)
        report["wall_duration_s"] = time.monotonic() - started
        report["metrics"] = trajectory_metrics(poses, report["wall_duration_s"])
        with (output_dir / "trajectory.jsonl").open("w") as handle:
            for pose in poses:
                handle.write(json.dumps(pose, allow_nan=False) + "\n")
        capture = cv2.VideoCapture(manifest['videos']['left']['path'])
        frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        capture.release()
        arrays, alignment = align_frames(frame_count, manifest['stereo_pairs_path'],
                                         output_dir / 'frame_status.csv', poses,
                                         args.start_s, args.duration_s)
        np.savez_compressed(output_dir / 'trajectory.npz', **arrays)
        report['frame_alignment'] = alignment
        report["artifacts"] = {"trajectory_jsonl": str(output_dir / "trajectory.jsonl"),
                               "trajectory_npz": str(output_dir / "trajectory.npz"),
                               "frame_status": str(output_dir / "frame_status.csv")}
        if code == 0 and len(poses) >= 2:
            report["status"] = "complete"
            return 0
        report["status"] = report.get("offline", {}).get("status", "failed")
        if report["status"] == "completed":
            report["status"] = "failed"
        return code or 3
    except BaseException as exc:
        report["error"] = str(exc)
        raise
    finally:
        (output_dir / "run_report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


def main() -> None:
    raise SystemExit(run(parse_args()))


if __name__ == "__main__":
    main()
