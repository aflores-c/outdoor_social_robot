#!/usr/bin/env python3
"""
Offline script: estimate the LiDAR → camera extrinsic transform from collected samples.

Uses plane-correspondence SVD (Kabsch algorithm):
  - For N board poses, we have N pairs of plane normals (n_lidar_i, n_cam_i).
  - Build cross-covariance H = sum_i outer(n_lidar_i, n_cam_i).
  - SVD of H gives rotation R such that R @ n_lidar ≈ n_cam.
  - Translation t = mean_i(c_cam_i - R @ c_lidar_i).

Usage:
    ros2 run lidar_camera_calibration estimate_transform
    ros2 run lidar_camera_calibration estimate_transform \\
        --samples /path/to/samples.json \\
        --output  /path/to/lidar_to_camera.yaml
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

try:
    from scipy.spatial.transform import Rotation
except ImportError:
    print('[ERROR] scipy not found. Install with: pip3 install scipy', file=sys.stderr)
    sys.exit(1)


# ── Math ────────────────────────────────────────────────────────────────────────

def estimate_rotation(n_lidar_list: list, n_cam_list: list) -> np.ndarray:
    """
    Kabsch SVD rotation from N unit-normal correspondences.

    Finds R (3×3 rotation matrix) minimising sum_i ||R @ n_lidar_i - n_cam_i||².
    """
    H = np.zeros((3, 3))
    for n_l, n_c in zip(n_lidar_list, n_cam_list):
        H += np.outer(n_l, n_c)

    U, _, Vt = np.linalg.svd(H)
    V = Vt.T
    # Correct for reflection (det = -1 would mean improper rotation)
    d = np.linalg.det(V @ U.T)
    R = V @ np.diag([1.0, 1.0, d]) @ U.T
    return R


def estimate_translation(
    R: np.ndarray, c_lidar_list: list, c_cam_list: list
) -> np.ndarray:
    """Least-squares translation: t = mean_i(c_cam_i - R @ c_lidar_i)."""
    return np.mean(
        [c_c - R @ c_l for c_c, c_l in zip(c_cam_list, c_lidar_list)], axis=0
    )


def compute_residuals(
    R: np.ndarray,
    t: np.ndarray,
    n_lidar_list: list,
    n_cam_list: list,
    c_lidar_list: list,
    c_cam_list: list,
) -> tuple:
    """Return per-sample (angular_deg, translation_mm) residuals."""
    ang, tra = [], []
    for n_l, n_c, c_l, c_c in zip(n_lidar_list, n_cam_list, c_lidar_list, c_cam_list):
        cos_a = float(np.clip(np.dot(R @ n_l, n_c), -1.0, 1.0))
        ang.append(np.degrees(np.arccos(cos_a)))
        tra.append(np.linalg.norm((R @ c_l + t) - c_c) * 1000.0)
    return ang, tra


# ── Main ─────────────────────────────────────────────────────────────────────────

def main():
    default_dir = Path.home() / '.ros' / 'lidar_camera_calibration'
    parser = argparse.ArgumentParser(
        description='Estimate LiDAR-camera extrinsic from collected plane samples.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        '--samples',
        default=str(default_dir / 'samples.json'),
        help='Path to samples.json produced by collect_samples_node',
    )
    parser.add_argument(
        '--output',
        default='',
        help='Output YAML path (default: same directory as samples)',
    )
    parser.add_argument(
        '--min-samples', type=int, default=6,
        help='Minimum number of samples required (default: 6)',
    )
    args = parser.parse_args()

    samples_path = Path(args.samples)
    if not samples_path.exists():
        print(f'[ERROR] Samples file not found: {samples_path}', file=sys.stderr)
        print(
            'Run the collector first:\n'
            '  ros2 launch lidar_camera_calibration collect.launch.py\n'
            '  ros2 service call /calibration/capture std_srvs/srv/Trigger',
            file=sys.stderr,
        )
        sys.exit(1)

    output_path = (
        Path(args.output) if args.output else samples_path.parent / 'lidar_to_camera.yaml'
    )

    # ── Load samples ─────────────────────────────────────────────────────────────
    with open(samples_path) as f:
        data = json.load(f)
    samples = data.get('samples', [])
    n = len(samples)
    print(f'Loaded {n} samples from {samples_path}')

    if n < 3:
        print('[ERROR] Need at least 3 samples (6+ recommended).', file=sys.stderr)
        sys.exit(1)
    if n < args.min_samples:
        print(
            f'[WARNING] Only {n} samples collected; {args.min_samples}+ recommended.\n'
            '  Accuracy may be lower. Collect more samples with varied board orientations.'
        )

    n_lidar_list = [np.array(s['n_lidar'], dtype=np.float64) for s in samples]
    n_cam_list = [np.array(s['n_cam'], dtype=np.float64) for s in samples]
    c_lidar_list = [np.array(s['c_lidar'], dtype=np.float64) for s in samples]
    c_cam_list = [np.array(s['c_cam'], dtype=np.float64) for s in samples]

    # Ensure unit normals
    n_lidar_list = [v / np.linalg.norm(v) for v in n_lidar_list]
    n_cam_list = [v / np.linalg.norm(v) for v in n_cam_list]

    # ── Estimate ──────────────────────────────────────────────────────────────────
    R = estimate_rotation(n_lidar_list, n_cam_list)
    t = estimate_translation(R, c_lidar_list, c_cam_list)
    ang_err, tra_err = compute_residuals(R, t, n_lidar_list, n_cam_list, c_lidar_list, c_cam_list)

    rot_obj = Rotation.from_matrix(R)
    quat = rot_obj.as_quat()          # [x, y, z, w]
    euler_deg = rot_obj.as_euler('xyz', degrees=True)

    # ── Print result ─────────────────────────────────────────────────────────────
    W = 64
    print(f'\n{"=" * W}')
    print(' LIDAR → CAMERA EXTRINSIC CALIBRATION RESULT')
    print(f'{"=" * W}')
    print(f'\nTransform:  velodyne  →  camera_color_optical_frame')
    print(f'\nRotation matrix R (R @ p_lidar = p_cam):')
    for row in R:
        print(f'  [{row[0]:+.6f}  {row[1]:+.6f}  {row[2]:+.6f}]')
    print(f'\nTranslation t [metres]:')
    print(f'  x = {t[0]:+.8f}')
    print(f'  y = {t[1]:+.8f}')
    print(f'  z = {t[2]:+.8f}')
    print(f'\nQuaternion (x, y, z, w):')
    print(f'  x={quat[0]:+.8f}  y={quat[1]:+.8f}  z={quat[2]:+.8f}  w={quat[3]:+.8f}')
    print(f'\nEuler angles XYZ [degrees]:')
    print(f'  roll={euler_deg[0]:+.4f}°  pitch={euler_deg[1]:+.4f}°  yaw={euler_deg[2]:+.4f}°')

    print(f'\nPer-sample residuals:')
    print(f'  {"#":>3}  {"angular [°]":>12}  {"translation [mm]":>16}  note')
    print(f'  {"-"*3}  {"-"*12}  {"-"*16}  ----')
    for i, (ae, te) in enumerate(zip(ang_err, tra_err)):
        note = '← check/remove?' if ae > 3.0 else ''
        print(f'  {i+1:>3}  {ae:>12.3f}  {te:>16.1f}  {note}')
    print(f'\n  Mean angular error  : {np.mean(ang_err):.3f}°  (std {np.std(ang_err):.3f}°)')
    print(f'  Mean translation err: {np.mean(tra_err):.1f} mm')
    print(f'  (good calibration: mean angular < 1°,  translation < 20 mm)')

    print(f'\n{"=" * W}')
    print(' static_transform_publisher command')
    print(f'{"=" * W}')
    print(
        f'  ros2 run tf2_ros static_transform_publisher \\\n'
        f'    --x {t[0]:.8f} --y {t[1]:.8f} --z {t[2]:.8f} \\\n'
        f'    --qx {quat[0]:.8f} --qy {quat[1]:.8f} '
        f'--qz {quat[2]:.8f} --qw {quat[3]:.8f} \\\n'
        f'    --frame-id camera_color_optical_frame --child-frame-id velodyne'
    )
    print(f'\nOr via launch file:\n'
          f'  ros2 launch lidar_camera_calibration publish_transform.launch.py\n'
          f'  (reads from {output_path})')
    print(f'{"=" * W}\n')

    # ── Save YAML ─────────────────────────────────────────────────────────────────
    result = {
        'lidar_to_camera': {
            'parent_frame': 'camera_color_optical_frame',
            'child_frame': 'velodyne',
            'translation': {
                'x': float(t[0]),
                'y': float(t[1]),
                'z': float(t[2]),
            },
            'rotation': {
                'quaternion': {
                    'x': float(quat[0]),
                    'y': float(quat[1]),
                    'z': float(quat[2]),
                    'w': float(quat[3]),
                },
                'matrix': R.tolist(),
                'euler_degrees': {
                    'roll': float(euler_deg[0]),
                    'pitch': float(euler_deg[1]),
                    'yaw': float(euler_deg[2]),
                },
            },
            'residuals': {
                'mean_angular_deg': float(np.mean(ang_err)),
                'std_angular_deg': float(np.std(ang_err)),
                'mean_translation_mm': float(np.mean(tra_err)),
                'per_sample_angular_deg': [float(e) for e in ang_err],
                'per_sample_translation_mm': [float(e) for e in tra_err],
            },
            'n_samples': n,
        }
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        yaml.dump(result, f, default_flow_style=False, sort_keys=False)

    print(f'Result saved: {output_path}')
    print('\nNext steps:')
    print('  1. Validate:  ros2 launch lidar_camera_calibration validate.launch.py')
    print('  2. Deploy:    ros2 launch lidar_camera_calibration publish_transform.launch.py')


if __name__ == '__main__':
    main()
