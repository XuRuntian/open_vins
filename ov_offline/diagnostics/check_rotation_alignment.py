"""Compare short-baseline visual rotations with calibrated gyro predictions.

Diagnostic only: neither edits calibration nor changes estimator parameters.
Translation/dynamic objects can bias essential-matrix rotation estimates.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument("--start-seconds", type=float, default=0,
                        help="Window start relative to the first camera timestamp")
    parser.add_argument("--seconds", type=float, default=10)
    args = parser.parse_args()
    if args.start_seconds < 0 or args.seconds <= 0:
        parser.error("Window start must be nonnegative and duration must be positive")
    cv2.setRNGSeed(0)
    root = args.session
    calibration = yaml.safe_load((root / "calibrations/camera_imu-camchain-imucam.yaml").read_text())["cam0"]
    fx, fy, cx, cy = calibration["intrinsics"]
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])
    distortion = np.asarray(calibration["distortion_coeffs"])
    if calibration["distortion_model"] != "equidistant":
        parser.error("This diagnostic currently expects raw equidistant images")
    rci = np.asarray(calibration["T_cam_imu"])[:3, :3]
    ts = np.loadtxt(root / "timestamps/left_timestamps.txt")
    if ts.ndim > 1:
        ts = ts[:, -1]
    imu = np.genfromtxt(root / "imu/imu0.csv", delimiter=",", names=True)
    it = imu["timestamp"]
    gyro = np.column_stack([imu[f"angular_velocity_{axis}"] for axis in "xyz"])
    cap = cv2.VideoCapture(str(root / "videos/left.mp4"))
    previous = None
    samples = []
    for index, timestamp in enumerate(ts):
        elapsed = timestamp - ts[0]
        if elapsed > args.start_seconds + args.seconds:
            break
        ok, frame = cap.read()
        if not ok:
            break
        if elapsed < args.start_seconds or index % 3:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if previous is not None:
            old, t0 = previous
            points = cv2.goodFeaturesToTrack(old, maxCorners=600, qualityLevel=.01, minDistance=15)
            if points is None:
                previous = gray, timestamp
                continue
            nxt, valid, error = cv2.calcOpticalFlowPyrLK(old, gray, points, None)
            p = points[valid.ravel() == 1]
            q = nxt[valid.ravel() == 1]
            p = cv2.fisheye.undistortPoints(p, k, distortion).reshape(-1, 2)
            q = cv2.fisheye.undistortPoints(q, k, distortion).reshape(-1, 2)
            keep = (np.linalg.norm(p, axis=1) < 2) & (np.linalg.norm(q, axis=1) < 2)
            p, q = p[keep], q[keep]
            if len(p) >= 30:
                essential, mask = cv2.findEssentialMat(p, q, np.eye(3), method=cv2.RANSAC, prob=.999, threshold=1/fx)
                if essential is not None and essential.shape == (3, 3) and mask.sum() >= 30:
                    r1, r2, _ = cv2.decomposeEssentialMat(essential)
                    rotation = min((r1, r2), key=lambda r: Rotation.from_matrix(r).magnitude())
                    samples.append((t0, timestamp, Rotation.from_matrix(rotation).as_rotvec()))
        previous = gray, timestamp
    cap.release()
    if len(samples) < 10:
        parser.error("Too few reliable visual rotation samples; do not infer a time correction")
    scores = []
    for shift in np.arange(-.5, .501, .01):
        observed, predicted = [], []
        for t0, t1, rotvec in samples:
            times = np.linspace(t0 + shift, t1 + shift, 8)
            avg = np.array([np.interp(times, it, gyro[:, i]).mean() for i in range(3)])
            observed.append(rotvec)
            predicted.append(-rci @ avg * (t1 - t0))
        observed, predicted = np.asarray(observed), np.asarray(predicted)
        error = np.linalg.norm(observed - predicted, axis=1)
        scores.append({"shift_s": float(shift), "median_error_deg": float(np.rad2deg(np.median(error))),
                       "p90_error_deg": float(np.rad2deg(np.percentile(error, 90))),
                       "correlation": float(np.corrcoef(observed.ravel(), predicted.ravel())[0, 1])})
    configured = min(scores, key=lambda x: abs(x["shift_s"]-calibration["timeshift_cam_imu"]))
    print(json.dumps({"session": root.name, "start_seconds": args.start_seconds,
                      "duration_seconds": args.seconds, "pairs": len(samples), "configured_nearest": configured,
                      "best": sorted(scores, key=lambda x: x["median_error_deg"])[:5]}, indent=2))


if __name__ == "__main__":
    main()
