import numpy as np
import pytest

from ov_offline.run import export_states

HEADER = "timestamp,timestamp_imu,left_frame_index,right_frame_index,px,py,pz,qx,qy,qz,qw,vx,vy,vz,legacy_qx,legacy_qy,legacy_qz,legacy_qw\n"

def test_export_preserves_frame_and_composes_camera_extrinsic(tmp_path):
    path = tmp_path / "states.csv"
    q = np.sqrt(0.5)
    path.write_text(HEADER + f"10,10.02,7,8,1,2,3,0,0,{q},{q},0.1,-0.2,0.3,0,0,{q},{q}\n")
    extrinsic = np.eye(4)
    extrinsic[0, 3] = 0.1
    poses = export_states(path, {"T_imu_camera0": extrinsic.tolist()})
    assert poses[0]["frame_index"] == 7
    assert poses[0]["right_frame_index"] == 8
    assert poses[0]["timestamp_imu"] == 10.02
    assert poses[0]["position_world_imu"] == [1, 2, 3]
    assert poses[0]["velocity_world_imu"] == [0.1, -0.2, 0.3]
    assert poses[0]["orientation_world_imu_xyzw"] == [0, 0, q, q]
    np.testing.assert_allclose(np.array(poses[0]["T_world_camera"])[:3, 3], [1, 2.1, 3])


def test_export_rejects_repeated_timestamps(tmp_path):
    path = tmp_path / "states.csv"
    row = "10,10,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1\n"
    path.write_text(HEADER + row * 2)
    with pytest.raises(ValueError, match="strictly increasing"):
        export_states(path, {"T_imu_camera0": np.eye(4).tolist()})
