"""Collect AprilGrid observations and solve left wrist eye-in-hand calibration."""

from __future__ import annotations

import argparse
import io
import json
import math
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np


DISTORTION = np.array(
    [-0.05253094434738159, 0.05721227824687958, -0.0008405927801504731,
     0.0004406056832522154, -0.01869192160665989],
    dtype=np.float64,
)


def quaternion_matrix(q: list[float]) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=np.float64)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_quaternion(r: np.ndarray) -> list[float]:
    # Stable conversion via Rodrigues-compatible rotation matrix branches.
    trace = float(np.trace(r))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
            x, y, z, w = 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s
        elif i == 1:
            s = math.sqrt(1 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
            x, y, z, w = (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s
        else:
            s = math.sqrt(1 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
            x, y, z, w = (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s
    return [float(x), float(y), float(z), float(w)]


def transform(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = r
    out[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return out


def board_points(tag_size: float, spacing_ratio: float) -> dict[int, np.ndarray]:
    pitch = tag_size * (1.0 + spacing_ratio)
    points: dict[int, np.ndarray] = {}
    for tag_id in range(36):
        row, col = divmod(tag_id, 6)
        x, y = col * pitch, row * pitch
        points[tag_id] = np.array(
            [[x, y, 0], [x + tag_size, y, 0],
             [x + tag_size, y + tag_size, 0], [x, y + tag_size, 0]],
            dtype=np.float64,
        )
    return points


def capture(snapshot_url: str, tag_size: float, spacing_ratio: float):
    with urlopen(snapshot_url, timeout=8) as response:
        data = np.load(io.BytesIO(response.read()))
        image = data["rgb"].copy()
        camera_k = np.asarray(data["camera_k"], dtype=np.float64)
        stamp = float(data["rgb_stamp"])
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        raise RuntimeError("no_apriltags_detected")
    by_id = {int(tag_id): corner[0] for tag_id, corner in zip(ids.ravel(), corners)}
    expected = board_points(tag_size, spacing_ratio)
    valid_ids = sorted(set(by_id).intersection(expected))
    if len(valid_ids) < 12:
        raise RuntimeError(f"insufficient_apriltags:{len(valid_ids)}")
    object_points = np.concatenate([expected[i] for i in valid_ids]).astype(np.float64)
    image_points = np.concatenate([by_id[i] for i in valid_ids]).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_k, DISTORTION,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or float(tvec[2, 0]) <= 0:
        raise RuntimeError("solvepnp_failed")
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_k, DISTORTION)
    reprojection = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
    r_target2cam, _ = cv2.Rodrigues(rvec)
    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    cv2.drawFrameAxes(image, camera_k, DISTORTION, rvec, tvec, tag_size * 2)
    return {
        "image": image,
        "camera_k": camera_k,
        "stamp": stamp,
        "ids": valid_ids,
        "r_target2cam": r_target2cam,
        "t_target2cam": tvec.reshape(3),
        "reprojection_mean_px": float(np.mean(reprojection)),
        "reprojection_max_px": float(np.max(reprojection)),
    }


def append_sample(args) -> None:
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    dataset_path = output / "samples.json"
    samples = json.loads(dataset_path.read_text(encoding="utf-8")) if dataset_path.exists() else []
    obs = capture(args.snapshot_url, args.tag_size, args.spacing_ratio)
    sample_id = len(samples)
    image_name = f"sample_{sample_id:02d}.jpg"
    cv2.imwrite(str(output / image_name), obs.pop("image"))
    sample = {
        "sample_id": sample_id,
        "image": image_name,
        "camera_stamp": obs["stamp"],
        "tag_ids": obs["ids"],
        "reprojection_mean_px": obs["reprojection_mean_px"],
        "reprojection_max_px": obs["reprojection_max_px"],
        "r_target2cam": obs["r_target2cam"].tolist(),
        "t_target2cam_m": obs["t_target2cam"].tolist(),
        "robot_pose_frame": "base",
        "robot_position_m": args.position,
        "robot_quaternion_xyzw": args.quaternion,
    }
    samples.append(sample)
    dataset_path.write_text(json.dumps(samples, indent=2), encoding="utf-8")
    print(json.dumps({"accepted": True, **sample}, indent=2))


def rotation_angle(r: np.ndarray) -> float:
    return math.acos(float(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)))


def solve(args) -> None:
    output = Path(args.output)
    samples = json.loads((output / "samples.json").read_text(encoding="utf-8"))
    if len(samples) < 8:
        raise RuntimeError(f"need_at_least_8_samples:{len(samples)}")
    rg2b = [quaternion_matrix(s["robot_quaternion_xyzw"]) for s in samples]
    tg2b = [np.asarray(s["robot_position_m"], dtype=np.float64) for s in samples]
    rt2c = [np.asarray(s["r_target2cam"], dtype=np.float64) for s in samples]
    tt2c = [np.asarray(s["t_target2cam_m"], dtype=np.float64) for s in samples]
    methods = {
        "tsai": cv2.CALIB_HAND_EYE_TSAI,
        "park": cv2.CALIB_HAND_EYE_PARK,
        "horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    results = []
    for name, method in methods.items():
        try:
            rc2g, tc2g = cv2.calibrateHandEye(rg2b, tg2b, rt2c, tt2c, method=method)
            base_targets = [transform(a, b) @ transform(rc2g, tc2g) @ transform(c, d)
                            for a, b, c, d in zip(rg2b, tg2b, rt2c, tt2c)]
            translations = np.array([t[:3, 3] for t in base_targets])
            reference = base_targets[0][:3, :3]
            angles = np.array([rotation_angle(reference.T @ t[:3, :3]) for t in base_targets])
            translation_rms = float(np.sqrt(np.mean(np.sum((translations - translations.mean(axis=0)) ** 2, axis=1))))
            rotation_rms_deg = float(np.rad2deg(np.sqrt(np.mean(angles ** 2))))
            if not np.isfinite(rc2g).all() or not np.isfinite(tc2g).all():
                continue
            results.append({
                "method": name,
                "translation_rms_m": translation_rms,
                "rotation_rms_deg": rotation_rms_deg,
                "camera_to_gripper_translation_m": np.asarray(tc2g).reshape(3).tolist(),
                "camera_to_gripper_quaternion_xyzw": matrix_quaternion(rc2g),
                "gripper_to_camera_matrix": np.linalg.inv(transform(rc2g, tc2g)).tolist(),
                "base_to_target_mean_translation_m": translations.mean(axis=0).tolist(),
            })
        except cv2.error:
            continue
    if not results:
        raise RuntimeError("all_hand_eye_methods_failed")
    results.sort(key=lambda r: r["translation_rms_m"] + math.radians(r["rotation_rms_deg"]) * 0.05)
    report = {
        "valid": results[0]["translation_rms_m"] < args.max_translation_rms
                 and results[0]["rotation_rms_deg"] < args.max_rotation_rms_deg,
        "sample_count": len(samples),
        "tag_size_m": args.tag_size,
        "tag_spacing_ratio": args.spacing_ratio,
        "best": results[0],
        "all_methods": results,
        "semantics": "camera_to_gripper is wrist_camera_left_color_optical_frame -> measured left EE frame; gripper_to_camera is its inverse",
    }
    (output / "hand_eye_result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["capture", "solve"])
    parser.add_argument("--output", default="outputs/hand_eye_calibration")
    parser.add_argument("--snapshot-url", default="http://127.0.0.1:18080/snapshot.npz")
    parser.add_argument("--tag-size", type=float, default=0.018)
    parser.add_argument("--spacing-ratio", type=float, default=0.3)
    parser.add_argument("--position", type=float, nargs=3)
    parser.add_argument("--quaternion", type=float, nargs=4)
    parser.add_argument("--max-translation-rms", type=float, default=0.01)
    parser.add_argument("--max-rotation-rms-deg", type=float, default=2.0)
    args = parser.parse_args()
    if args.mode == "capture":
        if args.position is None or args.quaternion is None:
            parser.error("capture requires --position and --quaternion")
        append_sample(args)
    else:
        solve(args)


if __name__ == "__main__":
    main()
