"""Diagnostic configuration copies must remain valid for OpenCV's YAML parser."""
import sys

import cv2
import numpy as np
import pytest
import yaml

from ov_offline.diagnostics.replay_cold_start import OpenCVDumper, main


def test_timing_counterfactual_preserves_opencv_nested_matrix(tmp_path):
    settings = {"cam0": {"timeshift_cam_imu": -0.01, "T_imu_cam": np.eye(4).tolist()}}
    path = tmp_path / "camera.yaml"
    path.write_text("%YAML:1.0\n" + yaml.dump(settings, Dumper=OpenCVDumper,
                                              default_flow_style=None, sort_keys=False))
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    try:
        camera = storage.getNode("cam0")
        assert camera.getNode("timeshift_cam_imu").real() == -0.01
        matrix = camera.getNode("T_imu_cam")
        actual = [[matrix.at(i).at(j).real() for j in range(4)] for i in range(4)]
        np.testing.assert_array_equal(actual, np.eye(4))
    finally:
        storage.release()


def test_nonfinite_timing_counterfactual_rejected_before_replay(tmp_path, monkeypatch):
    output = tmp_path / "results"
    monkeypatch.setattr(sys, "argv", ["replay", "--output", str(output), "--initial-offset-s", "nan"])
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2
    assert not output.exists()
