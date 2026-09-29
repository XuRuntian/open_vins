"""Coordinate conversion for OpenVINS trajectory output."""

from __future__ import annotations

import numpy as np


def quaternion_xyzw_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    if quaternion.shape != (4,):
        raise ValueError(f"Expected quaternion shape (4,), got {quaternion.shape}")
    norm = np.linalg.norm(quaternion)
    if norm <= 0:
        raise ValueError("Quaternion norm must be positive")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f"Expected rotation shape (3, 3), got {matrix.shape}")
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.array([x, y, z, w], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


def camera_pose_from_imu_pose(
    position_world_imu: np.ndarray,
    orientation_world_imu_xyzw: np.ndarray,
    transform_imu_camera: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Compose T_world_imu with T_imu_camera."""
    position = np.asarray(position_world_imu, dtype=np.float64)
    if position.shape != (3,):
        raise ValueError(f"Expected position shape (3,), got {position.shape}")
    extrinsic = np.asarray(transform_imu_camera, dtype=np.float64)
    if extrinsic.shape != (4, 4):
        raise ValueError(f"Expected extrinsic shape (4, 4), got {extrinsic.shape}")
    transform_world_imu = np.eye(4, dtype=np.float64)
    transform_world_imu[:3, :3] = quaternion_xyzw_to_rotation(
        orientation_world_imu_xyzw
    )
    transform_world_imu[:3, 3] = position
    transform_world_camera = transform_world_imu @ extrinsic
    orientation = rotation_to_quaternion_xyzw(transform_world_camera[:3, :3])
    return transform_world_camera, orientation


def trajectory_metrics(
    poses: list[dict[str, object]], wall_duration_s: float
) -> dict[str, float]:
    """Summarize a recorded trajectory without making quality claims."""
    if not poses:
        return {}
    timestamps = np.asarray(
        [pose["timestamp_camera"] for pose in poses], dtype=np.float64
    )
    transforms = np.asarray(
        [pose["T_world_camera"] for pose in poses], dtype=np.float64
    )
    alignment_errors = np.asarray(
        [pose["time_alignment_error_s"] for pose in poses], dtype=np.float64
    )
    positions = transforms[:, :3, 3]
    duration_s = float(timestamps[-1] - timestamps[0])
    path_length_m = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
    return {
        "trajectory_duration_s": duration_s,
        "trajectory_rate_hz": float((len(poses) - 1) / duration_s)
        if duration_s > 0
        else 0.0,
        "wall_output_rate_hz": float(len(poses) / wall_duration_s)
        if wall_duration_s > 0
        else 0.0,
        "path_length_m": path_length_m,
        "net_displacement_m": float(np.linalg.norm(positions[-1] - positions[0])),
        "max_abs_time_alignment_error_s": float(np.max(np.abs(alignment_errors))),
    }
