import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from zerolab.converter import (
    BODY_JOINT_COUNT,
    SMPL24_PARENTS,
    TPoseCalibrator,
    align_quaternion_signs,
    apply_rest_alignment,
    synthesize_smpl_world_quats,
)


DIRECT_CASES = [
    (10, 0), (11, 1), (14, 2), (12, 4), (15, 5),
    (13, 7), (16, 8), (2, 13), (6, 14), (0, 15),
    (3, 16), (7, 17), (4, 18), (8, 19), (5, 20), (9, 21),
]


def identity_body():
    quats = np.zeros((BODY_JOINT_COUNT, 4), dtype=np.float32)
    quats[:, 3] = 1.0
    return quats


def test_quaternion_sign_flip_is_continuous():
    previous = identity_body()
    current = -previous

    aligned = align_quaternion_signs(current, previous)

    np.testing.assert_array_equal(aligned, previous)
    assert aligned.dtype == np.float32


def test_stable_t_pose_completes_on_frame_100():
    calibrator = TPoseCalibrator()
    for index in range(99):
        frame = identity_body() if index % 2 == 0 else -identity_body()
        assert calibrator.observe(frame) is False
        assert calibrator.frames_collected == index + 1

    assert calibrator.observe(-identity_body()) is True
    assert calibrator.is_calibrated
    np.testing.assert_allclose(
        calibrator.rest_quats_xyzw, identity_body(), atol=1e-7
    )
    assert calibrator.rest_quats_xyzw.dtype == np.float32


def test_motion_over_five_degrees_restarts_calibration_window():
    calibrator = TPoseCalibrator(required_frames=4, max_deviation_degrees=5.0)
    for _ in range(3):
        calibrator.observe(identity_body())
    moved = identity_body()
    moved[4] = Rotation.from_euler("x", 10.0, degrees=True).as_quat()

    assert calibrator.observe(moved) is False
    assert calibrator.frames_collected == 1


def test_motion_at_five_degrees_stays_in_calibration_window():
    boundary = identity_body()
    boundary[4] = Rotation.from_euler("x", 5.0, degrees=True).as_quat()
    normalized = boundary.astype(np.float64)
    normalized /= np.linalg.norm(normalized, axis=1, keepdims=True)
    normalized = normalized.astype(np.float32)
    boundary_angle = np.degrees(
        2.0 * np.arccos(
            np.clip(abs(np.sum(normalized[4] * normalized[0])), 0.0, 1.0)
        )
    )
    calibrator = TPoseCalibrator(
        required_frames=3,
        max_deviation_degrees=float(boundary_angle),
    )
    calibrator.observe(identity_body())

    assert calibrator.observe(boundary) is False
    assert calibrator.frames_collected == 2


def test_motion_is_compared_to_the_current_window_mean():
    calibrator = TPoseCalibrator(required_frames=4, max_deviation_degrees=5.0)
    calibrator.observe(identity_body())
    four_degrees = identity_body()
    four_degrees[4] = Rotation.from_euler("x", 4.0, degrees=True).as_quat()
    eight_degrees = identity_body()
    eight_degrees[4] = Rotation.from_euler("x", 8.0, degrees=True).as_quat()

    assert calibrator.observe(four_degrees) is False
    assert calibrator.frames_collected == 2
    assert calibrator.observe(eight_degrees) is False
    assert calibrator.frames_collected == 1


def test_rest_alignment_uses_raw_times_inverse_rest():
    rest = np.repeat(
        Rotation.from_euler("x", 25.0, degrees=True).as_quat()[None, :],
        BODY_JOINT_COUNT,
        axis=0,
    )
    yaw = Rotation.from_euler("y", 30.0, degrees=True)
    raw = (yaw * Rotation.from_quat(rest)).as_quat()

    aligned = apply_rest_alignment(raw, rest)
    expected = np.repeat(yaw.as_quat()[None, :], BODY_JOINT_COUNT, axis=0)

    np.testing.assert_allclose(
        Rotation.from_quat(aligned).as_matrix(),
        Rotation.from_quat(expected).as_matrix(),
        atol=1e-6,
    )
    assert aligned.dtype == np.float32


def test_approved_mapping_slerps_spine_neck_and_copies_toes():
    body = identity_body()
    body[10] = Rotation.from_euler("y", 0.0, degrees=True).as_quat()
    body[1] = Rotation.from_euler("y", 60.0, degrees=True).as_quat()
    body[0] = Rotation.from_euler("y", 100.0, degrees=True).as_quat()
    body[13] = Rotation.from_euler("x", 12.0, degrees=True).as_quat()
    body[16] = Rotation.from_euler("x", -12.0, degrees=True).as_quat()

    smpl = synthesize_smpl_world_quats(body)

    for index, angle in zip(
        (3, 6, 9, 12, 15), (20.0, 40.0, 60.0, 80.0, 100.0)
    ):
        np.testing.assert_allclose(
            Rotation.from_quat(smpl[index]).as_matrix(),
            Rotation.from_euler("y", angle, degrees=True).as_matrix(),
            atol=1e-6,
        )
    np.testing.assert_allclose(smpl[10], smpl[7])
    np.testing.assert_allclose(smpl[11], smpl[8])
    np.testing.assert_allclose(smpl[22], smpl[20])
    np.testing.assert_allclose(smpl[23], smpl[21])
    assert smpl.dtype == np.float32
    assert SMPL24_PARENTS == [
        -1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8,
        9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21,
    ]


@pytest.mark.parametrize("source_index,target_index", DIRECT_CASES)
def test_each_measured_joint_uses_its_approved_direct_smpl_target(
    source_index, target_index
):
    body = identity_body()
    angle = 7.0 + source_index
    body[source_index] = Rotation.from_euler(
        "z", angle, degrees=True
    ).as_quat()

    smpl = synthesize_smpl_world_quats(body)

    np.testing.assert_allclose(
        Rotation.from_quat(smpl[target_index]).as_matrix(),
        Rotation.from_euler("z", angle, degrees=True).as_matrix(),
        atol=1e-6,
    )


@pytest.mark.parametrize(
    "bad_quaternions",
    [
        np.zeros((BODY_JOINT_COUNT - 1, 4), dtype=np.float32),
        np.full((BODY_JOINT_COUNT, 4), np.nan, dtype=np.float32),
        np.zeros((BODY_JOINT_COUNT, 4), dtype=np.float32),
    ],
)
def test_body_quaternion_inputs_require_finite_nonzero_17_by_4_arrays(
    bad_quaternions,
):
    with pytest.raises(ValueError):
        synthesize_smpl_world_quats(bad_quaternions)


@pytest.mark.parametrize(
    "bad_quaternions",
    [
        identity_body().astype("U32"),
        identity_body().astype(np.complex128) + 1j,
    ],
)
def test_all_converter_entry_points_reject_non_real_numeric_dtypes(
    bad_quaternions,
):
    with pytest.raises(ValueError):
        align_quaternion_signs(bad_quaternions)
    with pytest.raises(ValueError):
        apply_rest_alignment(bad_quaternions, identity_body())
    with pytest.raises(ValueError):
        TPoseCalibrator().observe(bad_quaternions)
    with pytest.raises(ValueError):
        synthesize_smpl_world_quats(bad_quaternions)
