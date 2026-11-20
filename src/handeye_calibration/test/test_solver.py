"""Check physical transform composition against a known synthetic installation."""

import copy

import cv2
import numpy as np
import pytest
import yaml

from handeye_calibration.solver import (
    matrix_quaternion, quaternion_matrix, rigid_transform, save_result, solve_session,
)


def transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = cv2.Rodrigues(np.asarray(rotation, dtype=float))[0]
    result[:3, 3] = translation
    return result


def synthetic_session():
    rng = np.random.default_rng(42)
    installation = transform([0.5, -0.3, 0.8], [0.02, 0.22, 0.14])
    fixed_board = transform([0.1, 0.2, -0.1], [0.6, 0.1, 0.3])
    session = {'tool_frame': 'arm2_tool_link', 'camera_frame': 'camera_color_frame',
               'samples': []}
    for index in range(30):
        tool = transform(rng.uniform(-0.8, 0.8, 3), rng.uniform(-0.3, 0.3, 3))
        observed_board = np.linalg.inv(installation) @ np.linalg.inv(tool) @ fixed_board
        session['samples'].append({
            'split': 'training' if index < 25 else 'validation',
            'T_base_tool': tool.tolist(), 'T_camera_board': observed_board.tolist(),
        })
    return session, installation


def test_recovers_camera_to_tool_in_metres_and_exports_yaml(tmp_path):
    session, expected = synthetic_session()
    report = save_result(session, tmp_path)
    np.testing.assert_allclose(report['T_tool_camera'], expected, atol=1e-9)
    assert report['validation']['count'] == 5
    assert report['validation']['translation_max_mm'] < 1e-6
    config = yaml.safe_load((tmp_path / 'camera_tf_candidate.yaml').read_text())
    params = config['astraS_camera']['ros__parameters']
    assert params['parent_frame_id'] == 'arm2_tool_link'
    np.testing.assert_allclose(params['tf_translation'], expected[:3, 3], atol=1e-9)
    np.testing.assert_allclose(quaternion_matrix(params['tf_rotation']),
                               expected[:3, :3], atol=1e-9)


def test_validation_is_independent_and_detects_board_motion():
    session, expected = synthetic_session()
    for sample in session['samples'][25:]:
        sample['T_camera_board'][0][3] += 0.03
    report = solve_session(session)
    np.testing.assert_allclose(report['T_tool_camera'], expected, atol=1e-9)
    assert report['training']['translation_max_mm'] < 1e-6
    assert report['validation']['translation_max_mm'] > 29


def test_rejects_single_axis_rotations():
    session, _ = synthetic_session()
    for index, sample in enumerate(session['samples']):
        sample['T_base_tool'] = transform([0, 0, index * 0.03], [0, 0, 0]).tolist()
    with pytest.raises(ValueError, match='rotation diversity'):
        solve_session(session)


def test_requires_enough_training_samples():
    session, _ = synthetic_session()
    session['samples'] = session['samples'][:14]
    with pytest.raises(ValueError, match='15 training'):
        solve_session(session)


def test_no_validation_does_not_claim_validated():
    session, _ = synthetic_session()
    session['samples'] = session['samples'][:25]
    assert solve_session(session)['validation'] is None


@pytest.mark.parametrize('rotation', [[0, 0, 0], [np.pi, 0, 0], [0, np.pi, 0],
                                     [0, 0, np.pi], [0.2, -0.8, 2.1]])
def test_quaternion_xyzw_round_trip(rotation):
    expected = transform(rotation, [0, 0, 0])[:3, :3]
    np.testing.assert_allclose(quaternion_matrix(matrix_quaternion(expected)),
                               expected, atol=1e-7)


def test_rejects_nonrigid_nonfinite_and_wrong_bottom_row():
    for matrix in [np.diag([1., 1., -1., 1.]), np.full((4, 4), np.nan)]:
        with pytest.raises(ValueError):
            rigid_transform(matrix)
    matrix = np.eye(4)
    matrix[3, 0] = 0.1
    with pytest.raises(ValueError):
        rigid_transform(matrix)
    session, _ = synthetic_session()
    invalid = copy.deepcopy(session)
    invalid['samples'][0]['T_camera_board'] = matrix.tolist()
    with pytest.raises(ValueError):
        solve_session(invalid)
