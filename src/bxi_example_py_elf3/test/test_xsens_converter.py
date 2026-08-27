from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import xsens.converter as converter_module
from xsens.converter import (
    XsensMotionConverter,
    align_quaternion_signs,
    shortest_path_slerp,
    synthesize_smpl_world_quats,
    world_to_parent_local_quats,
    xsens_positions_to_xrt,
    xsens_world_quaternions_to_xrt,
)
from xsens_test_helpers import make_packet


IDENTITY_XYZW = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
IDENTITY_WXYZ = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
EXPECTED_XSENS_TO_XRT = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
    dtype=np.float64,
)
EXPECTED_SMPL24_PARENTS = [
    -1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8,
    9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21,
]
EXPECTED_DIRECT_MAPPING = [
    (0, 1), (1, 20), (2, 16), (3, 2), (4, 21), (5, 17),
    (7, 22), (8, 18), (9, 5), (10, 23), (11, 19), (12, 6),
    (13, 12), (14, 8), (15, 7), (16, 13), (17, 9), (18, 14),
    (19, 10), (20, 15), (21, 11),
]


def axis_angle_xyzw(axis: int, degrees: float) -> np.ndarray:
    value = np.zeros(4, dtype=np.float64)
    value[axis] = np.sin(np.deg2rad(degrees) / 2.0)
    value[3] = np.cos(np.deg2rad(degrees) / 2.0)
    return value


def identity_world() -> np.ndarray:
    return np.tile(IDENTITY_XYZW, (23, 1))


def identity_wxyz() -> np.ndarray:
    return np.tile(IDENTITY_WXYZ, (23, 1))


def test_positions_apply_exact_x_z_negative_y_basis_without_mutating_input():
    source = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    original = source.copy()

    converted = xsens_positions_to_xrt(source)

    np.testing.assert_array_equal(
        converted,
        np.array([[1.0, 3.0, -2.0]], dtype=np.float32),
    )
    np.testing.assert_array_equal(source, original)
    assert converted.dtype == np.float32
    assert converted.flags.c_contiguous


def test_wxyz_component_order_maps_source_z_rotation_to_xrt_y_rotation():
    half_sqrt_two = np.float32(0.70710677)
    source_wxyz = np.array(
        [[half_sqrt_two, 0.0, 0.0, half_sqrt_two]], dtype=np.float32
    )

    converted = xsens_world_quaternions_to_xrt(source_wxyz)

    np.testing.assert_allclose(
        converted,
        np.array([[0.0, half_sqrt_two, 0.0, half_sqrt_two]]),
        atol=1e-7,
    )
    assert converted.dtype == np.float32


def test_basis_rotation_uses_matrix_conjugation():
    source = Rotation.from_euler(
        "xyz", [17.0, -23.0, 41.0], degrees=True
    ).as_matrix()
    source_wxyz = Rotation.from_matrix(source).as_quat(
        scalar_first=True
    ).reshape(1, 4)

    converted = xsens_world_quaternions_to_xrt(source_wxyz)
    actual = Rotation.from_quat(converted[0]).as_matrix()

    np.testing.assert_allclose(
        actual,
        EXPECTED_XSENS_TO_XRT @ source @ EXPECTED_XSENS_TO_XRT.T,
        atol=1e-6,
    )


def test_alignment_normalizes_and_selects_previous_quaternion_hemisphere():
    current = np.array(
        [[0.0, 0.0, 0.0, -2.0], [0.0, -3.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    previous = np.array(
        [[0.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 0.0]],
        dtype=np.float32,
    )
    original = current.copy()

    aligned = align_quaternion_signs(current, previous)

    np.testing.assert_array_equal(aligned, previous)
    np.testing.assert_array_equal(current, original)
    assert aligned.dtype == np.float32


def test_shortest_path_slerp_ignores_opposite_endpoint_sign():
    start = axis_angle_xyzw(2, 20.0)
    end = -axis_angle_xyzw(2, 60.0)

    midpoint = shortest_path_slerp(start, end, 0.5)

    expected = axis_angle_xyzw(2, 40.0)
    assert abs(float(np.dot(midpoint, expected))) == pytest.approx(1.0)


@pytest.mark.parametrize(
    "smpl_index,segment_id", EXPECTED_DIRECT_MAPPING
)
def test_each_direct_mapping_row_uses_declared_segment(
    smpl_index, segment_id
):
    world = identity_world()
    marker = axis_angle_xyzw((segment_id - 1) % 3, 7.0 + segment_id)
    world[segment_id - 1] = marker

    virtual = synthesize_smpl_world_quats(world)

    assert abs(float(np.dot(virtual[smpl_index], marker))) == pytest.approx(
        1.0
    )


def test_spine_midpoint_uses_shortest_path_and_endpoints_copy_wrists():
    world = identity_world()
    world[2] = axis_angle_xyzw(2, 20.0)  # L3, ID 3
    world[3] = -axis_angle_xyzw(2, 60.0)  # T12, ID 4
    world[14] = axis_angle_xyzw(0, 31.0)  # left hand, ID 15
    world[10] = axis_angle_xyzw(1, -29.0)  # right hand, ID 11

    virtual = synthesize_smpl_world_quats(world)

    expected_midpoint = axis_angle_xyzw(2, 40.0)
    assert abs(float(np.dot(virtual[6], expected_midpoint))) == pytest.approx(
        1.0
    )
    np.testing.assert_array_equal(virtual[22], virtual[20])
    np.testing.assert_array_equal(virtual[23], virtual[21])


def test_toes_remain_distinct_and_hand_endpoint_locals_are_identity():
    world = identity_world()
    world[22] = axis_angle_xyzw(0, 12.0)  # left toe ID 23
    world[18] = axis_angle_xyzw(1, 19.0)  # right toe ID 19

    virtual = synthesize_smpl_world_quats(world)
    local = world_to_parent_local_quats(virtual, EXPECTED_SMPL24_PARENTS)

    assert not np.allclose(virtual[10], virtual[11])
    np.testing.assert_allclose(local[22], IDENTITY_XYZW, atol=1e-6)
    np.testing.assert_allclose(local[23], IDENTITY_XYZW, atol=1e-6)


def test_parent_local_rotation_multiplies_inverse_parent_before_child():
    world = np.tile(IDENTITY_XYZW, (24, 1))
    half_sqrt_two = np.sqrt(0.5)
    world[0] = [half_sqrt_two, 0.0, 0.0, half_sqrt_two]
    world[1] = [0.0, half_sqrt_two, 0.0, half_sqrt_two]

    local = world_to_parent_local_quats(world, EXPECTED_SMPL24_PARENTS)

    np.testing.assert_allclose(
        local[0],
        [half_sqrt_two, 0.0, 0.0, half_sqrt_two],
        atol=1e-7,
    )
    np.testing.assert_allclose(
        local[1], [-0.5, 0.5, -0.5, 0.5], atol=1e-7
    )


def test_real_fk_output_has_exact_contract_and_only_six_wrist_slots():
    frame = XsensMotionConverter().convert(make_packet(), frame_index=17)

    assert frame.frame_index == 17
    assert frame.receive_timestamp_ns == 0
    assert frame.segment_positions_xrt.shape == (23, 3)
    assert frame.segment_quat_xrt_xyzw.shape == (23, 4)
    assert frame.smpl_joints.shape == (24, 3)
    assert frame.body_quat_w.shape == (4,)
    assert frame.smpl_body_pose.shape == (21, 3)
    assert frame.joint_pos.shape == (29,)
    arrays = (
        frame.segment_positions_xrt,
        frame.segment_quat_xrt_xyzw,
        frame.smpl_body_pose,
        frame.smpl_joints,
        frame.body_quat_w,
        frame.joint_pos,
    )
    assert all(array.dtype == np.float32 for array in arrays)
    assert all(array.flags.c_contiguous for array in arrays)
    assert all(np.isfinite(array).all() for array in arrays)
    assert all(not array.flags.writeable for array in arrays)
    assert set(np.flatnonzero(frame.joint_pos)) <= {
        19,
        20,
        21,
        26,
        27,
        28,
    }
    assert np.linalg.norm(frame.body_quat_w) == pytest.approx(1.0, abs=1e-6)
    with pytest.raises(FrozenInstanceError):
        frame.frame_index = 18


def test_frame_owns_read_only_copies_of_packet_transform_arrays():
    packet = make_packet(
        positions=np.arange(69, dtype=np.float32).reshape(23, 3) / 100.0
    )

    frame = XsensMotionConverter().convert(packet, frame_index=1)

    assert not np.shares_memory(
        frame.segment_positions_xrt, packet.segment_positions_xsens
    )
    assert not np.shares_memory(
        frame.segment_quat_xrt_xyzw, packet.segment_quat_wxyz_xsens
    )
    with pytest.raises(ValueError, match="read-only"):
        frame.segment_positions_xrt[0, 0] = 9.0


def test_frame_and_diagnostic_arrays_cannot_be_made_writeable_again():
    converter = XsensMotionConverter()
    frame = converter.convert(make_packet(), frame_index=1)
    arrays = (
        frame.segment_positions_xrt,
        frame.segment_quat_xrt_xyzw,
        frame.smpl_body_pose,
        frame.smpl_joints,
        frame.body_quat_w,
        frame.joint_pos,
        converter.previous_raw_quats_xyzw,
    )

    for array in arrays:
        with pytest.raises(ValueError, match="cannot set WRITEABLE flag"):
            array.setflags(write=True)


def test_converted_pelvis_translation_is_only_fk_position_input(monkeypatch):
    positions = np.arange(69, dtype=np.float32).reshape(23, 3) / 10.0
    packet = make_packet(positions=positions)
    captured = {}
    real_fk = converter_module.compute_from_body_poses

    def capture_fk(parents, body_poses):
        captured["parents"] = list(parents)
        captured["body_poses"] = body_poses.copy()
        return real_fk(parents, body_poses)

    monkeypatch.setattr(
        converter_module, "compute_from_body_poses", capture_fk
    )

    XsensMotionConverter().convert(packet, frame_index=1)

    assert captured["parents"] == EXPECTED_SMPL24_PARENTS
    np.testing.assert_array_equal(
        captured["body_poses"][0, :3],
        np.array([0.0, 0.2, -0.1], dtype=np.float32),
    )
    np.testing.assert_array_equal(
        captured["body_poses"][1:, :3],
        np.zeros((23, 3), dtype=np.float32),
    )


def test_xsens_positions_do_not_rescale_or_translate_fixed_skeleton():
    base = np.arange(69, dtype=np.float32).reshape(23, 3) / 100.0
    first = XsensMotionConverter().convert(
        make_packet(positions=base), frame_index=1
    )
    second = XsensMotionConverter().convert(
        make_packet(
            positions=base * 1.8 + np.array([3, -2, 7], np.float32)
        ),
        frame_index=2,
    )

    np.testing.assert_allclose(
        first.smpl_joints, second.smpl_joints, atol=1e-6
    )


def test_real_fk_pose_feeds_shared_elf3_wrist_axis_and_side_conventions():
    angle_degrees = 20.0
    angle_radians = np.deg2rad(angle_degrees)
    quaternions = identity_wxyz()
    wrist_wxyz = np.array(
        [
            np.cos(angle_radians / 2.0),
            np.sin(angle_radians / 2.0),
            0.0,
            0.0,
        ],
        dtype=np.float32,
    )
    quaternions[14] = wrist_wxyz  # left hand, ID 15
    quaternions[10] = wrist_wxyz  # right hand, ID 11

    frame = XsensMotionConverter().convert(
        make_packet(quaternions_wxyz=quaternions), frame_index=1
    )

    np.testing.assert_allclose(
        frame.smpl_body_pose[[19, 20]],
        [[-angle_radians, 0.0, 0.0], [-angle_radians, 0.0, 0.0]],
        atol=1e-6,
    )
    np.testing.assert_allclose(
        frame.joint_pos[[19, 26]], [-angle_radians, angle_radians], atol=1e-6
    )
    np.testing.assert_allclose(
        np.delete(frame.joint_pos, [19, 20, 21, 26, 27, 28]),
        np.zeros(23, dtype=np.float32),
        atol=1e-7,
    )


def test_wxyz_quaternions_are_normalized_and_sign_continuous():
    converter = XsensMotionConverter()
    identity = identity_wxyz()

    first = converter.convert(
        make_packet(quaternions_wxyz=identity), frame_index=1
    )
    second = converter.convert(
        make_packet(quaternions_wxyz=-identity), frame_index=2
    )

    dots = np.einsum(
        "ij,ij->i",
        first.segment_quat_xrt_xyzw,
        second.segment_quat_xrt_xyzw,
    )
    assert np.all(dots >= 0.0)
    np.testing.assert_allclose(
        np.linalg.norm(second.segment_quat_xrt_xyzw, axis=1),
        1.0,
        atol=1e-7,
    )


def test_reset_epoch_clears_sign_history():
    converter = XsensMotionConverter()
    positive = identity_wxyz()
    negative = -positive
    converter.convert(
        make_packet(quaternions_wxyz=positive), frame_index=1
    )
    continuous = converter.convert(
        make_packet(quaternions_wxyz=negative), frame_index=2
    )

    converter.reset_epoch()
    reset = converter.convert(
        make_packet(quaternions_wxyz=negative), frame_index=3
    )

    assert np.all(continuous.segment_quat_xrt_xyzw[:, 3] > 0)
    assert np.all(reset.segment_quat_xrt_xyzw[:, 3] < 0)


def test_previous_quaternion_diagnostic_is_a_read_only_copy():
    converter = XsensMotionConverter()
    assert converter.previous_raw_quats_xyzw is None
    converter.convert(make_packet(), frame_index=1)

    first = converter.previous_raw_quats_xyzw
    second = converter.previous_raw_quats_xyzw

    assert first is not second
    assert not np.shares_memory(first, second)
    assert not first.flags.writeable
    np.testing.assert_array_equal(first, np.tile(IDENTITY_XYZW, (23, 1)))
    with pytest.raises(ValueError, match="read-only"):
        first[0, 3] = -1.0


def test_successful_batch_is_ordered_atomic_and_reset_scoped():
    converter = XsensMotionConverter()
    converter.convert(make_packet(), frame_index=0)
    negative = -identity_wxyz()

    continuous = converter.convert_many(
        [(make_packet(quaternions_wxyz=negative), 1)]
    )
    reset = converter.convert_many(
        [
            (make_packet(sample_counter=2, quaternions_wxyz=negative), 2),
            (make_packet(sample_counter=3, quaternions_wxyz=negative), 3),
        ],
        reset_epoch=True,
    )

    assert isinstance(continuous, tuple)
    assert [frame.frame_index for frame in reset] == [2, 3]
    assert np.all(continuous[0].segment_quat_xrt_xyzw[:, 3] > 0)
    assert np.all(reset[0].segment_quat_xrt_xyzw[:, 3] < 0)
    assert np.all(reset[1].segment_quat_xrt_xyzw[:, 3] < 0)
    assert np.all(converter.previous_raw_quats_xyzw[:, 3] < 0)


def test_convert_many_rolls_back_reset_and_first_frame_when_second_fk_fails(
    monkeypatch,
):
    converter = XsensMotionConverter()
    converter.convert(make_packet(), frame_index=0)
    before = converter.previous_raw_quats_xyzw
    real_fk = converter_module.compute_from_body_poses
    calls = 0

    def fail_second(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise RuntimeError("synthetic FK failure")
        return real_fk(*args, **kwargs)

    monkeypatch.setattr(
        converter_module, "compute_from_body_poses", fail_second
    )

    with pytest.raises(RuntimeError, match="synthetic FK failure"):
        converter.convert_many(
            [
                (make_packet(sample_counter=1), 1),
                (make_packet(sample_counter=2), 2),
            ],
            reset_epoch=True,
        )

    np.testing.assert_array_equal(converter.previous_raw_quats_xyzw, before)


def test_failed_single_conversion_leaves_sign_history_unchanged(monkeypatch):
    converter = XsensMotionConverter()
    converter.convert(make_packet(), frame_index=0)
    before = converter.previous_raw_quats_xyzw

    def fail_fk(*args, **kwargs):
        raise RuntimeError("synthetic FK failure")

    monkeypatch.setattr(converter_module, "compute_from_body_poses", fail_fk)

    with pytest.raises(RuntimeError, match="synthetic FK failure"):
        converter.convert(
            make_packet(quaternions_wxyz=-identity_wxyz()), frame_index=1
        )

    np.testing.assert_array_equal(converter.previous_raw_quats_xyzw, before)


@pytest.mark.parametrize(
    "helper,bad",
    [
        (xsens_positions_to_xrt, np.zeros((23, 2), dtype=np.float32)),
        (xsens_positions_to_xrt, np.full((23, 3), np.nan, dtype=np.float32)),
        (
            xsens_world_quaternions_to_xrt,
            np.zeros((23, 4), dtype=np.float32),
        ),
        (
            xsens_world_quaternions_to_xrt,
            np.full((23, 4), np.inf, dtype=np.float32),
        ),
        (synthesize_smpl_world_quats, np.zeros((22, 4), dtype=np.float32)),
        (world_to_parent_local_quats, np.zeros((24, 4), dtype=np.float32)),
    ],
)
def test_transform_helpers_reject_bad_shape_nonfinite_or_zero_quaternions(
    helper, bad
):
    if helper is world_to_parent_local_quats:
        with pytest.raises(ValueError):
            helper(bad, EXPECTED_SMPL24_PARENTS)
    else:
        with pytest.raises(ValueError):
            helper(bad)


def test_nonfinite_packet_conversion_is_rejected_without_state_commit():
    converter = XsensMotionConverter()
    packet = make_packet()
    converter.convert(packet, frame_index=0)
    before = converter.previous_raw_quats_xyzw
    bad_positions = packet.segment_positions_xsens.copy()
    bad_positions[0, 0] = np.nan
    malformed = replace(packet, segment_positions_xsens=bad_positions)

    with pytest.raises(ValueError):
        converter.convert(malformed, frame_index=1)

    np.testing.assert_array_equal(converter.previous_raw_quats_xyzw, before)


@pytest.mark.parametrize(
    "field,bad_value",
    [
        ("smpl_pose", np.full((1, 69), np.nan, dtype=np.float32)),
        (
            "smpl_joints_local",
            np.full((1, 24, 3), np.nan, dtype=np.float32),
        ),
        (
            "global_orient_quat",
            np.array([[2.0, 0.0, 0.0, 0.0]], dtype=np.float32),
        ),
    ],
)
def test_invalid_fk_derived_arrays_are_rejected_without_state_commit(
    monkeypatch, field, bad_value
):
    converter = XsensMotionConverter()
    converter.convert(make_packet(), frame_index=0)
    before = converter.previous_raw_quats_xyzw
    real_fk = converter_module.compute_from_body_poses

    def invalid_fk(*args, **kwargs):
        result = real_fk(*args, **kwargs)
        result[field] = bad_value
        return result

    monkeypatch.setattr(
        converter_module, "compute_from_body_poses", invalid_fk
    )

    with pytest.raises(ValueError):
        converter.convert(make_packet(), frame_index=1)

    np.testing.assert_array_equal(converter.previous_raw_quats_xyzw, before)


def test_invalid_shared_wrist_output_is_rejected_without_state_commit(
    monkeypatch,
):
    converter = XsensMotionConverter()
    converter.convert(make_packet(), frame_index=0)
    before = converter.previous_raw_quats_xyzw

    def invalid_wrist(*args, **kwargs):
        return np.full((1, 29), np.nan, dtype=np.float32)

    monkeypatch.setattr(
        converter_module, "build_elf3_joint_pos", invalid_wrist
    )

    with pytest.raises(ValueError):
        converter.convert(make_packet(), frame_index=1)

    np.testing.assert_array_equal(converter.previous_raw_quats_xyzw, before)
