"""Audit detectors must flag invalid delivery data, not repair it."""
import csv
import json

import numpy as np
import pytest

from ov_offline.diagnostics.audit_runtime import manifest_alias, measure


def fixture_output(path, jump=0.01, corrupt=False):
    matrices = np.repeat(np.eye(4)[None], 4, axis=0)
    matrices[0] = np.nan
    matrices[1:, 0, 3] = np.arange(3) * jump
    if corrupt:
        matrices[0] = np.eye(4)
    status = np.array(["initializing", "valid", "valid", "valid"])
    np.savez(path / "trajectory.npz", schema_version="openvins.frame_aligned.v2", status=status,
             frame_index=np.arange(4), timestamp_camera=np.arange(4) / 30,
             timestamp_imu=np.arange(4) / 30, T_world_camera=matrices, T_world_imu=matrices)
    report = {"status": "complete", "pose_count": 3, "offline": {},
              "frame_alignment": {"valid_pose_count": 3, "frame_count": 4}}
    (path / "report.json").write_text(json.dumps(report))
    debug = path / "debug/run/trajectory"
    debug.mkdir(parents=True)
    with (debug / "frame_status.csv").open("w") as handle:
        writer = csv.writer(handle)
        writer.writerow(["status", "msckf_features", "slam_features"])
        writer.writerows((s, 1, 0) for s in status)


def test_runtime_audit_accepts_contract_and_measures_motion(tmp_path):
    fixture_output(tmp_path)
    result, ids, positions = measure(tmp_path)
    assert result["path_m"] == pytest.approx(0.02)
    assert result["max_speed_m_s"] == pytest.approx(0.3)
    assert result["review_flags"] == []
    assert ids.tolist() == [1, 2, 3]
    assert positions.shape == (3, 3)


def test_runtime_audit_rejects_fabricated_invalid_pose(tmp_path):
    fixture_output(tmp_path, corrupt=True)
    with pytest.raises(AssertionError):
        measure(tmp_path)


def test_runtime_audit_flags_large_jump_without_clipping(tmp_path):
    fixture_output(tmp_path, jump=1.0)
    result, _, positions = measure(tmp_path)
    assert "large_pose_increment" in result["review_flags"]
    assert result["path_m"] == 2.0
    assert positions[-1, 0] == 2.0


def test_manifest_alias_validates_session_and_video_paths(tmp_path):
    session = tmp_path / "session_example"
    (session / "dataset_manifest").mkdir(parents=True)
    (session / "videos").mkdir()
    original = "/mnt/nas/session_example"
    manifest = {"root_dir": original, "videos": {}}
    for side in ("left", "right"):
        filename = f"{side}_rectified.mp4"
        (session / "videos" / filename).touch()
        manifest["videos"][side] = {"path": f"{original}/videos/{filename}"}
    path = session / "dataset_manifest/dataset_manifest.json"
    path.write_text(json.dumps(manifest))
    assert manifest_alias(session) == original
    manifest["videos"]["right"]["path"] = "/mnt/nas/another_session/videos/right_rectified.mp4"
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        manifest_alias(session)
