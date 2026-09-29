"""Frame-aligned delivery; unavailable states are never interpolated or filled."""
import csv
from pathlib import Path

import numpy as np

SCHEMA = 'openvins.frame_aligned.v2'
DIAGNOSTIC_STATUSES = {'valid', 'initializing', 'imu_unavailable',
                       'state_not_advanced', 'invalid_state'}


def align_frames(frame_count, pairs_path, diagnostics_path, poses, start_s=0., duration_s=None):
    if (frame_count <= 0 or not np.isfinite(start_s) or start_s < 0
            or (duration_s is not None and (not np.isfinite(duration_s) or duration_s <= 0))):
        raise ValueError('Invalid frame count or replay interval')
    with Path(pairs_path).open() as handle:
        pairs = list(csv.DictReader(handle))
    if not pairs:
        raise ValueError('Empty stereo pairs')
    arrays = dict(
        schema_version=np.asarray(SCHEMA),
        quality_status=np.asarray('not_validated'),
        frame_index=np.arange(frame_count, dtype=np.int64),
        right_frame_index=np.full(frame_count, -1, dtype=np.int64),
        timestamp_camera=np.full(frame_count, np.nan),
        timestamp_imu=np.full(frame_count, np.nan),
        status=np.full(frame_count, 'no_stereo_pair', dtype='<U32'),
        T_world_imu=np.full((frame_count, 4, 4), np.nan),
        T_world_camera=np.full((frame_count, 4, 4), np.nan),
    )
    begin = float(pairs[0]['timestamp']) + start_s
    end = begin + duration_s if duration_s is not None else float('inf')
    previous_index, previous_time, previous_right = -1, -float('inf'), -1
    requested = set()
    for row in pairs:
        index, right = int(row['left_frame_index']), int(row['right_frame_index'])
        timestamp = float(row['timestamp'])
        if not (previous_index < index < frame_count and right > previous_right
                and np.isfinite(timestamp) and timestamp > previous_time):
            raise ValueError('Invalid or non-monotonic stereo frame indices/timestamps')
        previous_index, previous_time, previous_right = index, timestamp, right
        arrays['right_frame_index'][index] = right
        arrays['timestamp_camera'][index] = timestamp
        selected = begin <= timestamp < end
        arrays['status'][index] = 'not_processed' if selected else 'not_requested'
        if selected:
            requested.add(index)

    def check_row(row, index, timestamp):
        if index not in requested or arrays['right_frame_index'][index] != int(row['right_frame_index']):
            raise ValueError('State/diagnostic does not match a requested stereo pair')
        if not np.isfinite(timestamp) or abs(arrays['timestamp_camera'][index] - timestamp) > 1e-6:
            raise ValueError('State/diagnostic timestamp does not match stereo pair')

    seen = set()
    if Path(diagnostics_path).is_file():
        with Path(diagnostics_path).open() as handle:
            for row in csv.DictReader(handle):
                index = int(row['left_frame_index'])
                check_row(row, index, float(row['timestamp']))
                status = row['status']
                if index in seen or status not in DIAGNOSTIC_STATUSES:
                    raise ValueError('Duplicate or unknown frame diagnostic')
                seen.add(index)
                arrays['status'][index] = status
                imu_time = float(row['timestamp_imu'])
                if not np.isfinite(imu_time):
                    raise ValueError('Non-finite diagnostic IMU timestamp')
                arrays['timestamp_imu'][index] = imu_time

    exported = set()
    for pose in poses:
        index = pose['frame_index']
        check_row(pose, index, pose['timestamp_camera'])
        if index in exported or arrays['status'][index] != 'valid':
            raise ValueError('Pose has no unique valid frame diagnostic')
        if abs(arrays['timestamp_imu'][index] - pose['timestamp_imu']) > 1e-6:
            raise ValueError('Pose IMU time differs from diagnostic')
        for key in ('T_world_imu', 'T_world_camera'):
            matrix = np.asarray(pose[key], dtype=float)
            if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
                raise ValueError('Invalid pose matrix')
            arrays[key][index] = matrix
        exported.add(index)
    if set(np.flatnonzero(arrays['status'] == 'valid')) != exported:
        raise ValueError('Valid diagnostic without an exported pose')
    names, counts = np.unique(arrays['status'], return_counts=True)
    summary = dict(schema_version=SCHEMA, frame_count=frame_count,
                   requested_pair_count=len(requested), valid_pose_count=len(poses),
                   invalid_pose_count=frame_count - len(poses),
                   status_counts={str(name): int(count) for name, count in zip(names, counts)})
    return arrays, summary
