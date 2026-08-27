from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest

from pico.pose_to_smpl_ref_bridge import _decode_packed_message
from pico.zmq_messages import pack_pose_message
from xsens.converter import XsensMotionConverter
from xsens.source_core import (
    ArmCommand,
    ArmCommandClassification,
    ArmCommandResult,
    XsensReason,
)
from xsens_test_helpers import (
    accept_packet,
    feed_frame_sequence,
    feed_stable_frames,
    make_core,
    ready_core,
)


POSE_KEYS = {
    "frame_index",
    "smpl_joints",
    "body_quat_w",
    "joint_pos",
    "stream_mode",
    "calibration_ready",
    "producer_monotonic_ns",
    "source_epoch",
}
STATUS_KEYS = {
    "status_sequence",
    "status_monotonic_ns",
    "source_epoch",
    "last_arm_command_id",
    "last_arm_target_epoch",
    "last_requested_arm_epoch",
    "accepted_arm_epoch",
    "producer_monotonic_ns",
    "newest_frame_index",
    "ready",
    "reference_window_ready",
    "source_stale",
    "ready_frames",
    "recovery_frames",
    "reason_code",
}
POSE_SCHEMA = {
    "frame_index": (np.dtype(np.int64), (10,)),
    "smpl_joints": (np.dtype(np.float32), (10, 24, 3)),
    "body_quat_w": (np.dtype(np.float32), (10, 4)),
    "joint_pos": (np.dtype(np.float32), (10, 29)),
    "stream_mode": (np.dtype(np.int32), (1,)),
    "calibration_ready": (np.dtype(np.bool_), (1,)),
    "producer_monotonic_ns": (np.dtype(np.int64), (1,)),
    "source_epoch": (np.dtype(np.int64), (1,)),
}
STATUS_SCHEMA = {
    "status_sequence": (np.dtype(np.int64), (1,)),
    "status_monotonic_ns": (np.dtype(np.int64), (1,)),
    "source_epoch": (np.dtype(np.int64), (1,)),
    "last_arm_command_id": (np.dtype(np.int64), (1,)),
    "last_arm_target_epoch": (np.dtype(np.int64), (1,)),
    "last_requested_arm_epoch": (np.dtype(np.int64), (1,)),
    "accepted_arm_epoch": (np.dtype(np.int64), (1,)),
    "producer_monotonic_ns": (np.dtype(np.int64), (1,)),
    "newest_frame_index": (np.dtype(np.int64), (1,)),
    "ready": (np.dtype(np.bool_), (1,)),
    "reference_window_ready": (np.dtype(np.bool_), (1,)),
    "source_stale": (np.dtype(np.bool_), (1,)),
    "ready_frames": (np.dtype(np.int32), (1,)),
    "recovery_frames": (np.dtype(np.int32), (1,)),
    "reason_code": (np.dtype(np.int32), (1,)),
}


def status_scalar(core, clock, name):
    return core.build_status_fields(clock.now_ns)[name].tolist()


def test_arm_contract_types_are_frozen_and_enum_values_are_stable():
    command = ArmCommand(1, 91, 91)
    result = ArmCommandResult(
        ArmCommandClassification.ARMED, True, True, True
    )
    with pytest.raises(FrozenInstanceError):
        command.command_id = 2
    with pytest.raises(FrozenInstanceError):
        result.first_seen = False
    assert [item.value for item in ArmCommandClassification] == [
        "armed",
        "disarmed",
        "target_mismatch",
        "not_ready",
        "duplicate",
        "reused_id",
        "invalid",
    ]
    assert [int(item) for item in XsensReason] == list(range(12))


def test_valid_arm_requires_current_ready_fresh_complete_epoch():
    core, clock = ready_core(epoch=91)
    result = core.handle_arm_command(ArmCommand(7, 91, 91))
    assert result == ArmCommandResult(
        ArmCommandClassification.ARMED, True, True, True
    )
    status = core.build_status_fields(clock.now_ns)
    assert status["last_arm_command_id"].tolist() == [7]
    assert status["last_arm_target_epoch"].tolist() == [91]
    assert status["last_requested_arm_epoch"].tolist() == [91]
    assert status["accepted_arm_epoch"].tolist() == [91]
    with pytest.raises(AttributeError):
        core.accepted_arm_epoch = 0


def test_arm_clock_failure_rolls_back_receipt_and_allows_first_seen_retry():
    core, clock = ready_core(epoch=91)
    now_before = clock.now_ns
    window_before = core.current_pose_window()
    stats_before = core.stats
    clock.now_ns = None

    with pytest.raises(ValueError, match="clock_ns"):
        core.handle_arm_command(ArmCommand(777, 91, 91))

    clock.now_ns = now_before
    assert core.accepted_arm_epoch == 0
    assert core._seen_arm_commands == {}
    assert core.stats == stats_before
    assert all(
        actual is expected
        for actual, expected in zip(core.current_pose_window(), window_before)
    )
    status = core.build_status_fields(clock.now_ns)
    assert status["last_arm_command_id"].tolist() == [0]
    assert status["last_arm_target_epoch"].tolist() == [0]
    assert status["last_requested_arm_epoch"].tolist() == [0]
    assert core.handle_arm_command(ArmCommand(777, 91, 91)) == (
        ArmCommandResult(
            ArmCommandClassification.ARMED, True, True, True
        )
    )


def test_first_seen_current_arm_is_rejected_when_not_ready_or_fresh():
    collecting, clock = make_core(epoch_draws=(91, 92))
    feed_stable_frames(collecting, clock, 10)
    assert collecting.handle_arm_command(
        ArmCommand(18, 91, 91)
    ) == ArmCommandResult(
        ArmCommandClassification.NOT_READY, True, True, True
    )
    assert status_scalar(collecting, clock, "accepted_arm_epoch") == [0]

    stale, stale_clock = ready_core(91)
    stale_clock.advance_ns(500_000_001)
    assert stale.handle_arm_command(
        ArmCommand(19, 91, 91)
    ).classification is ArmCommandClassification.NOT_READY
    status = stale.build_status_fields(stale_clock.now_ns)
    assert status["source_stale"].tolist() == [True]
    assert status["accepted_arm_epoch"].tolist() == [0]


def test_disarm_clears_readiness_even_when_already_unarmed():
    core, clock = ready_core(epoch=91)
    stats_before = core.stats
    assert core.handle_arm_command(ArmCommand(8, 91, 0)) == ArmCommandResult(
        ArmCommandClassification.DISARMED, True, True, True
    )
    status = core.build_status_fields(clock.now_ns)
    assert status["accepted_arm_epoch"].tolist() == [0]
    assert status["ready_frames"].tolist() == [0]
    assert not status["reference_window_ready"][0]
    assert core.stats == stats_before


def test_armed_stale_retains_authorization_and_requires_ten_recovery_frames():
    core, clock = ready_core(epoch=91)
    core.handle_arm_command(ArmCommand(9, 91, 91))
    newest = core.newest_producer_monotonic_ns
    clock.advance_ns(newest + 500_000_001 - clock.now_ns)
    assert core.check_stale(clock.now_ns)
    assert core.accepted_arm_epoch == 91
    assert core.current_pose_window() is None

    feed_stable_frames(core, clock, count=9, epoch_counter_start=31)
    status = core.build_status_fields(clock.now_ns)
    assert status["recovery_frames"].tolist() == [9]
    assert status["reason_code"].tolist() == [XsensReason.ARMED_RECOVERING]
    assert core.build_pose_fields() is None

    feed_stable_frames(core, clock, count=1, epoch_counter_start=40)
    status = core.build_status_fields(clock.now_ns)
    assert status["recovery_frames"].tolist() == [10]
    assert status["reason_code"].tolist() == [XsensReason.ARMED_FRESH]
    assert core.build_pose_fields()["frame_index"].tolist() == list(
        range(31, 41)
    )


@pytest.mark.parametrize(
    "command",
    [
        ArmCommand(0, 91, 91),
        ArmCommand(-1, 91, 91),
        ArmCommand(True, 91, 91),
        ArmCommand(np.bool_(True), 91, 91),
        ArmCommand(1.0, 91, 91),
        ArmCommand(2**63, 91, 91),
        ArmCommand(1, 0, 0),
        ArmCommand(1, -1, 0),
        ArmCommand(1, True, 0),
        ArmCommand(1, np.bool_(True), 0),
        ArmCommand(1, 91.0, 0),
        ArmCommand(1, 2**63, 0),
        ArmCommand(1, 91, True),
        ArmCommand(1, 91, np.bool_(True)),
        ArmCommand(1, 91, 91.0),
        ArmCommand(1, 91, 90),
        ArmCommand(1, 91, -1),
        ArmCommand(1, 91, 2**63),
    ],
)
def test_structurally_invalid_commands_do_not_replace_receipt(command):
    core, clock = ready_core(epoch=91)
    result = core.handle_arm_command(command)
    assert result == ArmCommandResult(
        ArmCommandClassification.INVALID, False, False, False
    )
    status = core.build_status_fields(clock.now_ns)
    assert status["last_arm_command_id"].tolist() == [0]
    assert status["reason_code"].tolist() == [XsensReason.INVALID_INPUT]


def test_malformed_command_preserves_existing_receipt_and_source_state():
    core, clock = ready_core(epoch=91)
    assert core.handle_arm_command(ArmCommand(801, 91, 91)) == (
        ArmCommandResult(
            ArmCommandClassification.ARMED, True, True, True
        )
    )
    status_before = core.build_status_fields(clock.now_ns)
    receipt_before = {
        name: status_before[name].tolist()
        for name in (
            "last_arm_command_id",
            "last_arm_target_epoch",
            "last_requested_arm_epoch",
            "accepted_arm_epoch",
        )
    }
    seen_before = core._seen_arm_commands.copy()
    window_before = core.current_pose_window()
    stats_before = core.stats
    authorization_before = core.accepted_arm_epoch

    assert core.handle_arm_command(ArmCommand(0, 91, 91)) == (
        ArmCommandResult(
            ArmCommandClassification.INVALID, False, False, False
        )
    )

    status_after = core.build_status_fields(clock.now_ns)
    assert {
        name: status_after[name].tolist() for name in receipt_before
    } == receipt_before
    assert status_after["reason_code"].tolist() == [11]
    assert core._seen_arm_commands == seen_before
    assert core.accepted_arm_epoch == authorization_before
    assert core.stats == stats_before
    assert all(
        actual is expected
        for actual, expected in zip(core.current_pose_window(), window_before)
    )


def test_signed_int64_numpy_values_are_accepted_without_bool_coercion():
    core, clock = ready_core(91)
    result = core.handle_arm_command(
        ArmCommand(np.int64(2**63 - 1), np.int64(91), np.int64(0))
    )
    assert result.classification is ArmCommandClassification.DISARMED
    assert status_scalar(core, clock, "last_arm_command_id") == [2**63 - 1]


def test_invalid_id_is_not_reserved_and_valid_reuse_is_first_seen():
    core, clock = ready_core(91)
    core.handle_arm_command(ArmCommand(30, 91, 90))
    result = core.handle_arm_command(ArmCommand(30, 91, 0))
    assert result == ArmCommandResult(
        ArmCommandClassification.DISARMED, True, True, True
    )
    assert status_scalar(core, clock, "last_arm_command_id") == [30]


def test_duplicate_and_reused_command_id_are_distinct():
    core, clock = ready_core(epoch=91)
    command = ArmCommand(12, 91, 91)
    assert (
        core.handle_arm_command(command).classification
        is ArmCommandClassification.ARMED
    )
    receipt = core.build_status_fields(clock.now_ns)
    assert core.handle_arm_command(command) == ArmCommandResult(
        ArmCommandClassification.DUPLICATE, False, False, False
    )
    changed = core.handle_arm_command(ArmCommand(12, 91, 0))
    assert changed == ArmCommandResult(
        ArmCommandClassification.REUSED_ID, False, False, False
    )
    latest = core.build_status_fields(clock.now_ns)
    for name in (
        "last_arm_command_id",
        "last_arm_target_epoch",
        "last_requested_arm_epoch",
        "accepted_arm_epoch",
    ):
        np.testing.assert_array_equal(latest[name], receipt[name])


def test_duplicate_disarm_does_not_restart_post_barrier_readiness():
    core, clock = ready_core(91)
    command = ArmCommand(13, 91, 0)
    core.handle_arm_command(command)
    feed_stable_frames(core, clock, 1)
    stats_before = core.stats
    assert core.handle_arm_command(command) == ArmCommandResult(
        ArmCommandClassification.DUPLICATE, False, False, False
    )
    assert core.ready_frames == 1
    assert core.stats == stats_before


def test_not_ready_replay_is_exact_duplicate_without_state_changes():
    core, clock = make_core(epoch_draws=(91, 92))
    feed_stable_frames(core, clock, 10)
    command = ArmCommand(811, 91, 91)
    assert core.handle_arm_command(command) == ArmCommandResult(
        ArmCommandClassification.NOT_READY, True, True, True
    )
    status_before = core.build_status_fields(clock.now_ns)
    observed_before = {
        name: status_before[name].tolist()
        for name in (
            "last_arm_command_id",
            "last_arm_target_epoch",
            "last_requested_arm_epoch",
            "accepted_arm_epoch",
            "ready",
            "reference_window_ready",
            "ready_frames",
            "recovery_frames",
            "reason_code",
        )
    }
    seen_before = core._seen_arm_commands.copy()
    window_before = core.current_pose_window()
    stats_before = core.stats

    assert core.handle_arm_command(command) == ArmCommandResult(
        ArmCommandClassification.DUPLICATE, False, False, False
    )

    status_after = core.build_status_fields(clock.now_ns)
    assert {
        name: status_after[name].tolist() for name in observed_before
    } == observed_before
    assert core._seen_arm_commands == seen_before
    assert core.accepted_arm_epoch == 0
    assert core.stats == stats_before
    assert all(
        actual is expected
        for actual, expected in zip(core.current_pose_window(), window_before)
    )


def test_first_seen_not_ready_arm_is_echoed_but_not_accepted():
    core, clock = make_core(epoch_draws=(91, 92))
    feed_stable_frames(core, clock, 10)
    result = core.handle_arm_command(ArmCommand(19, 91, 91))
    assert result == ArmCommandResult(
        ArmCommandClassification.NOT_READY, True, True, True
    )
    status = core.build_status_fields(clock.now_ns)
    assert status["last_arm_command_id"].tolist() == [19]
    assert status["accepted_arm_epoch"].tolist() == [0]


def test_accepted_arm_bypasses_later_stillness_but_not_freshness_or_window():
    core, clock = ready_core(91)
    assert (
        core.handle_arm_command(
            ArmCommand(20, 91, 91)
        ).classification
        is ArmCommandClassification.ARMED
    )

    positions = np.zeros((30, 23, 3), np.float32)
    positions[:, 0, 0] = np.linspace(0.0, 1.0, 30, dtype=np.float32)
    quats = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    feed_frame_sequence(core, clock, positions, quats)
    status = core.build_status_fields(clock.now_ns)
    assert status["ready"].tolist() == [True]
    assert status["reason_code"].tolist() == [XsensReason.ARMED_FRESH]

    newest = core.newest_producer_monotonic_ns
    assert core.check_stale(newest + 500_000_001)
    stale = core.build_status_fields(newest + 500_000_001)
    assert stale["ready"].tolist() == [False]
    assert stale["reference_window_ready"].tolist() == [False]
    assert stale["reason_code"].tolist() == [XsensReason.ARMED_STALE]


def test_target_mismatch_is_echoed_without_mutating_authorization_or_windows():
    core, clock = ready_core(91)
    before_indices = [
        frame.frame_index for frame in core.current_pose_window()
    ]
    result = core.handle_arm_command(ArmCommand(21, 90, 90))
    assert result == ArmCommandResult(
        ArmCommandClassification.TARGET_MISMATCH, True, True, True
    )
    status = core.build_status_fields(clock.now_ns)
    assert status["last_arm_command_id"].tolist() == [21]
    assert status["last_arm_target_epoch"].tolist() == [90]
    assert status["last_requested_arm_epoch"].tolist() == [90]
    assert status["accepted_arm_epoch"].tolist() == [0]
    assert status["reason_code"].tolist() == [
        XsensReason.ARM_COMMAND_MISMATCH
    ]
    assert [
        frame.frame_index for frame in core.current_pose_window()
    ] == before_indices


def test_target_mismatch_preserves_existing_authorization():
    core, _ = ready_core(91)
    core.handle_arm_command(ArmCommand(31, 91, 91))
    window_before = core.current_pose_window()
    core.handle_arm_command(ArmCommand(32, 90, 0))
    assert core.accepted_arm_epoch == 91
    assert all(
        actual is expected
        for actual, expected in zip(core.current_pose_window(), window_before)
    )


def test_target_mismatch_replay_is_exact_duplicate_without_state_changes():
    core, clock = ready_core(91)
    command = ArmCommand(812, 90, 90)
    assert core.handle_arm_command(command) == ArmCommandResult(
        ArmCommandClassification.TARGET_MISMATCH, True, True, True
    )
    status_before = core.build_status_fields(clock.now_ns)
    observed_before = {
        name: status_before[name].tolist()
        for name in (
            "last_arm_command_id",
            "last_arm_target_epoch",
            "last_requested_arm_epoch",
            "accepted_arm_epoch",
            "ready",
            "reference_window_ready",
            "ready_frames",
            "recovery_frames",
            "reason_code",
        )
    }
    seen_before = core._seen_arm_commands.copy()
    window_before = core.current_pose_window()
    stats_before = core.stats

    assert core.handle_arm_command(command) == ArmCommandResult(
        ArmCommandClassification.DUPLICATE, False, False, False
    )

    status_after = core.build_status_fields(clock.now_ns)
    assert {
        name: status_after[name].tolist() for name in observed_before
    } == observed_before
    assert core._seen_arm_commands == seen_before
    assert core.accepted_arm_epoch == 0
    assert core.stats == stats_before
    assert all(
        actual is expected
        for actual, expected in zip(core.current_pose_window(), window_before)
    )


def test_old_target_disarm_after_epoch_commit_preserves_two_replayed_frames():
    core, clock = ready_core(91)
    core.handle_arm_command(ArmCommand(22, 91, 91))
    clock.advance_ns(600_000_000)
    accept_packet(core, 5, timestamp_ns=clock.now_ns)
    clock.advance_ns(1)
    accept_packet(core, 6, timestamp_ns=clock.now_ns)
    assert core.source_epoch == 92
    assert core.ready_frames == 2
    result = core.handle_arm_command(ArmCommand(23, 91, 0))
    assert result.classification is ArmCommandClassification.TARGET_MISMATCH
    assert core.source_epoch == 92
    assert core.accepted_arm_epoch == 0
    assert core.ready_frames == 2


def test_epoch_commit_revokes_authorization_receipt_but_retains_seen_ids():
    core, clock = ready_core(91)
    command = ArmCommand(28, 91, 91)
    core.handle_arm_command(command)
    clock.advance_ns(600_000_000)
    accept_packet(core, 5, timestamp_ns=clock.now_ns)
    clock.advance_ns(1)
    accept_packet(core, 6, timestamp_ns=clock.now_ns)
    status = core.build_status_fields(clock.now_ns)
    assert status["source_epoch"].tolist() == [92]
    assert status["accepted_arm_epoch"].tolist() == [0]
    assert status["last_arm_command_id"].tolist() == [0]
    assert status["last_arm_target_epoch"].tolist() == [0]
    assert status["last_requested_arm_epoch"].tolist() == [0]
    assert core.handle_arm_command(command).classification is (
        ArmCommandClassification.DUPLICATE
    )


def test_pose_fields_have_exact_shapes_dtypes_and_metadata():
    core, _ = ready_core(91)
    fields = core.build_pose_fields()
    assert set(fields) == POSE_KEYS
    assert {
        key: (value.dtype, value.shape) for key, value in fields.items()
    } == POSE_SCHEMA
    assert fields["stream_mode"].tolist() == [1]
    assert fields["calibration_ready"].tolist() == [True]
    assert fields["source_epoch"].tolist() == [91]
    assert fields["producer_monotonic_ns"].tolist() == [
        core.newest_producer_monotonic_ns
    ]
    assert fields["frame_index"].tolist() == list(range(21, 31))


def test_pose_build_requires_fresh_complete_strictly_progressing_window():
    core, clock = make_core(epoch_draws=(91, 92))
    assert core.build_pose_fields() is None
    feed_stable_frames(core, clock, 9)
    assert core.build_pose_fields() is None
    feed_stable_frames(core, clock, 1)
    first = core.build_pose_fields()
    assert first["frame_index"].tolist() == list(range(1, 11))
    assert first["calibration_ready"].tolist() == [False]
    accept_packet(core, 10, timestamp_ns=clock.now_ns + 1)
    assert core.build_pose_fields()["frame_index"].tolist() == list(
        range(1, 11)
    )
    clock.advance_ns(500_000_001)
    assert core.build_pose_fields() is None


def test_full_output_window_with_repeated_index_is_not_reference_ready():
    core, _ = ready_core(91)
    window = core.current_pose_window()
    core._output_frames[-1] = replace(
        window[-1], frame_index=window[-2].frame_index
    )

    assert core.reference_window_ready is False
    assert core.current_pose_window() is None
    assert core.build_pose_fields() is None


def test_pose_and_status_fields_are_immutable_fresh_copies_without_aliasing():
    core, clock = ready_core(91)
    first_pose = core.build_pose_fields()
    second_pose = core.build_pose_fields()
    first_status = core.build_status_fields(clock.now_ns)
    second_status = core.build_status_fields(clock.now_ns)
    assert all(not value.flags.writeable for value in first_pose.values())
    assert all(not value.flags.writeable for value in first_status.values())
    assert all(value.flags.aligned for value in first_pose.values())
    assert all(value.flags.aligned for value in first_status.values())
    assert all(
        not np.shares_memory(first_pose[name], second_pose[name])
        for name in POSE_KEYS
    )
    assert all(
        not np.shares_memory(first_status[name], second_status[name])
        for name in STATUS_KEYS
    )
    with pytest.raises(ValueError):
        first_pose["frame_index"][0] = 0
    with pytest.raises(ValueError):
        first_status["ready"][0] = False
    for value in (*first_pose.values(), *first_status.values()):
        with pytest.raises(ValueError):
            value.setflags(write=True)


def test_status_fields_have_exact_shapes_dtypes_sentinels_and_ranges():
    empty, clock = make_core(epoch_draws=(91, 92))
    fields = empty.build_status_fields(clock.now_ns)
    assert set(fields) == STATUS_KEYS
    assert fields["producer_monotonic_ns"].tolist() == [0]
    assert fields["newest_frame_index"].tolist() == [-1]
    assert fields["last_arm_command_id"].tolist() == [0]
    assert fields["accepted_arm_epoch"].tolist() == [0]
    assert {
        key: (value.dtype, value.shape) for key, value in fields.items()
    } == STATUS_SCHEMA
    assert 0 <= int(fields["ready_frames"][0]) <= 30
    assert 0 <= int(fields["recovery_frames"][0]) <= 10
    assert int(fields["reason_code"][0]) in set(XsensReason)


def test_status_fields_preserve_exact_field_provenance():
    core, clock = ready_core(91)
    assert core.handle_arm_command(ArmCommand(707, 91, 0)) == (
        ArmCommandResult(
            ArmCommandClassification.DISARMED, True, True, True
        )
    )
    feed_stable_frames(core, clock, 10)
    core._status_sequence = 52

    fields = core.build_status_fields(777_777_777)

    assert set(fields) == STATUS_KEYS
    assert {
        name: (field.dtype, field.shape) for name, field in fields.items()
    } == STATUS_SCHEMA
    assert {
        name: field.tolist() for name, field in fields.items()
    } == {
        "status_sequence": [53],
        "status_monotonic_ns": [777_777_777],
        "source_epoch": [91],
        "last_arm_command_id": [707],
        "last_arm_target_epoch": [91],
        "last_requested_arm_epoch": [0],
        "accepted_arm_epoch": [0],
        "producer_monotonic_ns": [666_666_640],
        "newest_frame_index": [40],
        "ready": [False],
        "reference_window_ready": [True],
        "source_stale": [False],
        "ready_frames": [10],
        "recovery_frames": [0],
        "reason_code": [2],
    }


def test_status_sequence_increases_for_start_change_command_and_heartbeat():
    core, clock = make_core(epoch_draws=(91, 92))
    start = int(core.build_status_fields(clock.now_ns)["status_sequence"][0])
    accept_packet(core, 1, timestamp_ns=1)
    change = int(core.build_status_fields(1)["status_sequence"][0])
    core.handle_arm_command(ArmCommand(24, 91, 0))
    receipt = int(core.build_status_fields(2)["status_sequence"][0])
    heartbeat = int(core.build_status_fields(3)["status_sequence"][0])
    assert [start, change, receipt, heartbeat] == list(
        range(start, start + 4)
    )
    assert [1, 2, 3] == [
        int(core.build_status_fields(now)["status_monotonic_ns"][0])
        for now in (1, 2, 3)
    ]


def test_status_and_pose_integer_overflow_is_rejected_without_wrapping():
    core, clock = ready_core(91)
    core._status_sequence = 2**63 - 2
    assert status_scalar(core, clock, "status_sequence") == [2**63 - 1]
    with pytest.raises(OverflowError, match="status_sequence"):
        core.build_status_fields(clock.now_ns)
    assert core._status_sequence == 2**63 - 1
    with pytest.raises(ValueError, match="now_ns"):
        core.build_status_fields(2**63)

    core._output_frames[-1] = core._output_frames[-1].__class__(
        **{**core._output_frames[-1].__dict__, "frame_index": 2**63}
    )
    with pytest.raises(OverflowError, match="frame_index"):
        core.build_pose_fields()


def test_packed_pose_and_status_round_trip_through_existing_decoder():
    core, clock = ready_core(91)
    for topic, fields in (
        ("pose", core.build_pose_fields()),
        ("xsens_status", core.build_status_fields(clock.now_ns)),
    ):
        decoded = _decode_packed_message(
            pack_pose_message(fields, topic=topic), topic
        )
        assert set(decoded) == set(fields)
        for name in fields:
            np.testing.assert_array_equal(decoded[name], fields[name])


def test_stale_never_resets_converter_sign_history():
    class CountingConverter(XsensMotionConverter):
        def __init__(self):
            super().__init__()
            self.reset_calls = 0

        def reset_epoch(self):
            self.reset_calls += 1
            return super().reset_epoch()

    converter = CountingConverter()
    core, clock = make_core(epoch_draws=(91, 92), converter=converter)
    feed_stable_frames(core, clock, 30)
    core.handle_arm_command(ArmCommand(25, 91, 91))
    assert core.check_stale(
        core.newest_producer_monotonic_ns + 500_000_001
    )
    assert converter.reset_calls == 0


def test_reason_lifecycle_has_deterministic_precedence():
    core, clock = ready_core(91)
    core.handle_arm_command(ArmCommand(26, 90, 90))
    assert status_scalar(core, clock, "reason_code") == [
        XsensReason.ARM_COMMAND_MISMATCH
    ]
    core.note_invalid_input()
    assert status_scalar(core, clock, "reason_code") == [
        XsensReason.INVALID_INPUT
    ]
    feed_stable_frames(core, clock, 1)
    assert status_scalar(core, clock, "reason_code") == [
        XsensReason.ARM_COMMAND_MISMATCH
    ]
    core.handle_arm_command(ArmCommand(27, 91, 0))
    assert status_scalar(core, clock, "reason_code") != [
        XsensReason.ARM_COMMAND_MISMATCH
    ]
    clock.advance_ns(600_000_000)
    accept_packet(core, 5, timestamp_ns=clock.now_ns)
    clock.advance_ns(1)
    accept_packet(core, 6, timestamp_ns=clock.now_ns)
    core.handle_arm_command(ArmCommand(29, 91, 0))
    assert status_scalar(core, clock, "reason_code") == [
        XsensReason.SESSION_RESET
    ]
    core.note_invalid_input()
    assert status_scalar(core, clock, "reason_code") == [
        XsensReason.INVALID_INPUT
    ]
    clock.advance_ns(1)
    accept_packet(core, 7, timestamp_ns=clock.now_ns)
    assert status_scalar(core, clock, "reason_code") == [
        XsensReason.ARM_COMMAND_MISMATCH
    ]


def test_base_reason_codes_distinguish_window_stability_and_ready_states():
    empty, clock = make_core(epoch_draws=(91, 92))
    assert status_scalar(empty, clock, "reason_code") == [XsensReason.NO_DATA]
    feed_stable_frames(empty, clock, 9)
    assert status_scalar(empty, clock, "reason_code") == [
        XsensReason.COLLECTING_WINDOW
    ]
    feed_stable_frames(empty, clock, 1)
    assert status_scalar(empty, clock, "reason_code") == [
        XsensReason.COLLECTING_STABILITY
    ]
    feed_stable_frames(empty, clock, 20)
    assert status_scalar(empty, clock, "reason_code") == [XsensReason.READY]

    pelvis, pelvis_clock = make_core(epoch_draws=(91, 92))
    positions = np.zeros((30, 23, 3), np.float32)
    positions[15:, 0, 0] = 0.2
    quats = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    feed_frame_sequence(pelvis, pelvis_clock, positions, quats)
    assert status_scalar(pelvis, pelvis_clock, "reason_code") == [
        XsensReason.PELVIS_UNSTABLE
    ]

    segment, segment_clock = make_core(epoch_draws=(91, 92))
    moving = quats.copy()
    angle = np.deg2rad(50.0) / 2.0
    moving[:15, 4, 0] = np.cos(angle)
    moving[:15, 4, 3] = np.sin(angle)
    moving[15:, 4, 0] = np.cos(angle)
    moving[15:, 4, 3] = -np.sin(angle)
    feed_frame_sequence(
        segment,
        segment_clock,
        np.zeros_like(positions),
        moving,
    )
    assert status_scalar(segment, segment_clock, "reason_code") == [
        XsensReason.SEGMENT_UNSTABLE
    ]
