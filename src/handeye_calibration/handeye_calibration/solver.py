"""Offline solver, independent of ROS and camera hardware."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def rigid_transform(value):
    """Validate a rigid transform instead of silently accepting bad poses."""
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Transform must be a finite 4x4 matrix')
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5)):
        raise ValueError('Invalid rigid transform')
    return matrix


def quaternion_matrix(xyzw):
    """Convert a ROS quaternion to a rotation matrix, rejecting invalid input."""
    q = np.asarray(xyzw, dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-10:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def matrix_quaternion(rotation):
    """Convert R to xyzw, including rotations close to 180 degrees."""
    vector = cv2.Rodrigues(rotation)[0].ravel()
    angle = np.linalg.norm(vector)
    if angle < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return np.r_[vector / angle * np.sin(angle / 2), np.cos(angle / 2)].tolist()


def board_error(poses, reference):
    """Report fixed-board drift relative to the first training board pose."""
    delta = [np.linalg.inv(reference) @ pose for pose in poses]
    translation = [float(np.linalg.norm(t[:3, 3]) * 1000) for t in delta]
    rotation = [float(np.rad2deg(np.linalg.norm(cv2.Rodrigues(t[:3, :3])[0])))
                for t in delta]
    return {
        'count': len(delta),
        'translation_rms_mm': float(np.sqrt(np.mean(np.square(translation)))),
        'translation_max_mm': max(translation),
        'rotation_rms_deg': float(np.sqrt(np.mean(np.square(rotation)))),
        'rotation_max_deg': max(rotation),
    }


def solve_session(session):
    """Solve training samples and evaluate unused validation samples."""
    training, validation = [], []
    for sample in session['samples']:
        if sample.get('excluded', False):
            continue
        pair = (rigid_transform(sample['T_base_tool']),
                rigid_transform(sample['T_camera_board']))
        split = sample['split']
        if split not in ('training', 'validation'):
            raise ValueError('Unknown sample split')
        (training if split == 'training' else validation).append(pair)
    if len(training) < 15:
        raise ValueError('Need at least 15 training poses with varied rotations')

    # At least two significantly excited rotation directions are necessary.
    rotations = [cv2.Rodrigues(training[0][0][:3, :3].T @ a[:3, :3])[0].ravel()
                 for a, _ in training[1:]]
    singular = np.linalg.svd(np.asarray(rotations), compute_uv=False)
    if singular[1] < np.deg2rad(10):
        raise ValueError('Insufficient rotation diversity; rotate around different axes')
    r, t = cv2.calibrateHandEye(
        [a[:3, :3] for a, _ in training], [a[:3, 3:4] for a, _ in training],
        [b[:3, :3] for _, b in training], [b[:3, 3:4] for _, b in training],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = r, t.ravel()
    rigid_transform(result)
    boards = [a @ result @ b for a, b in training]
    report = {
        'T_tool_camera': result.tolist(),
        'units': 'metres',
        'method': 'PARK',
        'training': board_error(boards, boards[0]),
        'validation': (board_error([a @ result @ b for a, b in validation], boards[0])
                       if validation else None),
        'note': 'Candidate only. Inspect independent validation before applying.',
    }
    return report


def save_result(session, directory):
    """Write a candidate config without modifying any installed camera config."""
    report = solve_session(session)
    result = np.asarray(report['T_tool_camera'])
    config = {'astraS_camera': {'ros__parameters': {
        'parent_frame_id': session['tool_frame'],
        'color_frame_id': session['camera_frame'],
        'tf_translation': result[:3, 3].tolist(),
        'tf_rotation': matrix_quaternion(result[:3, :3]),
    }}}
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'result.json').write_text(
        json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    (directory / 'camera_tf_candidate.yaml').write_text(
        yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path, help='Recorded session.json')
    args = parser.parse_args()
    try:
        session = json.loads(args.session.read_text(encoding='utf-8'))
        report = save_result(session, args.session.parent)
    except (ValueError, KeyError, OSError, cv2.error) as exc:
        parser.exit(1, f'Calibration failed: {exc}\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
