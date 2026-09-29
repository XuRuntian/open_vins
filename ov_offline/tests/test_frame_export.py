import csv

import numpy as np
import pytest

from ov_offline.frame_export import align_frames


def write_rows(path, rows):
    with path.open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def fixture(tmp_path):
    pairs = tmp_path / 'pairs.csv'
    diagnostics = tmp_path / 'frame_status.csv'
    write_rows(pairs, [dict(timestamp=10 + i, left_frame_index=i, right_frame_index=i) for i in range(5)])
    states = ['imu_unavailable', 'initializing', 'valid', 'state_not_advanced', 'valid']
    write_rows(diagnostics, [dict(timestamp=10 + i, left_frame_index=i, right_frame_index=i,
                                 timestamp_imu=10 + i - .01, initialized=int(i >= 2),
                                 state_advanced=int(i in (2, 4)), status=status)
                             for i, status in enumerate(states)])
    poses = [dict(timestamp_camera=10 + i, timestamp_imu=10 + i - .01,
                  frame_index=i, right_frame_index=i, T_world_imu=np.eye(4),
                  T_world_camera=np.eye(4)) for i in (2, 4)]
    return pairs, diagnostics, poses


def test_every_video_frame_has_explicit_status_and_invalid_nan(tmp_path):
    pairs, diagnostics, poses = fixture(tmp_path)
    arrays, summary = align_frames(6, pairs, diagnostics, poses)
    assert arrays['frame_index'].tolist() == list(range(6))
    assert arrays['status'].tolist() == ['imu_unavailable', 'initializing', 'valid',
                                       'state_not_advanced', 'valid', 'no_stereo_pair']
    valid = arrays['status'] == 'valid'
    assert valid.tolist() == [False, False, True, False, True, False]
    assert np.isnan(arrays['T_world_camera'][~valid]).all()
    assert np.isfinite(arrays['T_world_camera'][valid]).all()
    assert not {'valid', 'processed', 'initialized'}.intersection(arrays)
    assert str(arrays['schema_version']) == 'openvins.frame_aligned.v2'
    assert np.isnan(arrays['timestamp_camera'][5])
    assert arrays['right_frame_index'][5] == -1
    assert summary['frame_count'] == 6
    assert summary['valid_pose_count'] == 2
    assert summary['invalid_pose_count'] == 4
    np.savez_compressed(tmp_path / 'trajectory.npz', **arrays)
    with np.load(tmp_path / 'trajectory.npz', allow_pickle=False) as data:
        assert data['status'].dtype.kind == 'U'


def test_partial_run_keeps_video_indices_and_marks_unprocessed(tmp_path):
    pairs, diagnostics, _ = fixture(tmp_path)
    diagnostics.unlink()
    arrays, summary = align_frames(6, pairs, diagnostics, [], start_s=1, duration_s=2)
    assert arrays['status'].tolist() == ['not_requested', 'not_processed', 'not_processed',
                                       'not_requested', 'not_requested', 'no_stereo_pair']
    assert summary['requested_pair_count'] == 2
    assert not (arrays['status'] == 'valid').any()


def test_export_refuses_pose_without_matching_successful_diagnostic(tmp_path):
    pairs, diagnostics, poses = fixture(tmp_path)
    poses[0]['frame_index'] = 1
    with pytest.raises(ValueError):
        align_frames(6, pairs, diagnostics, poses)


def test_export_refuses_out_of_video_pair_indices(tmp_path):
    pairs, diagnostics, poses = fixture(tmp_path)
    with pytest.raises(ValueError):
        align_frames(4, pairs, diagnostics, poses)


def test_invalid_state_and_interrupted_tail_are_not_filled(tmp_path):
    pairs, diagnostics, _ = fixture(tmp_path)
    write_rows(diagnostics, [dict(timestamp=10, left_frame_index=0, right_frame_index=0,
                                 timestamp_imu=9.99, initialized=1, state_advanced=1,
                                 status='invalid_state')])
    arrays, summary = align_frames(5, pairs, diagnostics, [])
    assert arrays['status'].tolist() == ['invalid_state'] + ['not_processed'] * 4
    assert np.isnan(arrays['T_world_imu']).all()
    assert summary['invalid_pose_count'] == 5
