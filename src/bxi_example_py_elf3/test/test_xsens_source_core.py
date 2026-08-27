from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest

from xsens.converter import XsensMotionConverter
from xsens.source_core import (
    AcceptClassification,
    TimeCodeMode,
    angular_deviation_degrees,
)
from xsens_test_helpers import (
    SOURCE_PERIOD_NS,
    axis_angle_wxyz,
    feed_frame_sequence,
    feed_stable_frames,
    make_core,
    make_packet,
    ready_core,
)


def test_readiness_requires_thirty_stable_frames_and_complete_window():
    core, clock = make_core()
    feed_stable_frames(core, clock, count=29)
    assert core.ready is False
    assert core.ready_frames == 29
    feed_stable_frames(core, clock, count=1)
    assert core.ready_frames == 30
    assert core.reference_window_ready is True
    assert core.ready is True
    assert [
        frame.frame_index for frame in core.current_pose_window()
    ] == list(range(21, 31))


def test_reference_window_must_be_complete_even_if_evidence_limit_is_met():
    core, clock = make_core(ready_frames=5)
    feed_stable_frames(core, clock, count=5)
    assert core.ready_frames == 5
    assert core.reference_window_ready is False
    assert core.current_pose_window() is None
    assert core.ready is False

    feed_stable_frames(core, clock, count=5)
    assert core.ready_frames == 5
    assert core.reference_window_ready is True
    assert core.ready is True


def test_stale_boundary_is_strictly_greater_than_half_second():
    core, _ = ready_core(epoch=91)
    newest = core.newest_producer_monotonic_ns
    epoch_before = core.source_epoch
    stats_before = core.stats
    assert not core.check_stale(newest + 500_000_000)
    assert core.check_stale(newest + 500_000_001)
    assert core.ready is False
    assert core.ready_frames == 0
    assert core.current_pose_window() is None
    assert core.source_epoch == epoch_before
    assert core.newest_producer_monotonic_ns == newest
    assert core.stats == stats_before
    assert not core.check_stale(newest + 600_000_000)


def test_stale_clears_windows_without_resetting_converter_continuity():
    converter = XsensMotionConverter()
    core, clock = make_core(converter=converter)
    feed_stable_frames(core, clock, count=30)
    continuity_before = converter.previous_raw_quats_xyzw
    epoch_before = core.source_epoch

    assert core.check_stale(
        core.newest_producer_monotonic_ns + 500_000_001
    )

    np.testing.assert_array_equal(
        converter.previous_raw_quats_xyzw, continuity_before
    )
    assert core.source_epoch == epoch_before


def test_post_stale_output_and_readiness_refill_in_the_same_epoch():
    core, clock = ready_core(epoch=91)
    assert core.check_stale(
        core.newest_producer_monotonic_ns + 500_000_001
    )
    feed_stable_frames(core, clock, count=9)
    assert core.ready_frames == 9
    assert core.current_pose_window() is None
    feed_stable_frames(core, clock, count=1)
    assert core.reference_window_ready
    assert not core.ready
    assert [
        frame.frame_index for frame in core.current_pose_window()
    ] == list(range(31, 41))
    feed_stable_frames(core, clock, count=20)
    assert core.source_epoch == 91
    assert core.ready


def test_stale_with_pending_candidate_preserves_same_epoch_source_state():
    converter = XsensMotionConverter()
    core, clock = make_core(converter=converter)
    for offset in range(30):
        clock.advance_ns(SOURCE_PERIOD_NS)
        result = core.accept(
            make_packet(
                sample_counter=100 + offset,
                time_code=1000 + offset,
                receive_timestamp_ns=clock.now_ns,
            )
        )
        assert result.accepted
    assert core.ready
    assert core.time_code_mode is TimeCodeMode.ADVANCING

    clock.advance_ns(SOURCE_PERIOD_NS)
    candidate = make_packet(
        sample_counter=50,
        time_code=500,
        receive_timestamp_ns=clock.now_ns,
    )
    started = core.accept(candidate)
    assert started.classification is AcceptClassification.CANDIDATE_STARTED
    active_before = (
        core.source_epoch,
        core.locked_sender,
        core._sample_counter,
        core.newest_frame_index,
        core.newest_producer_monotonic_ns,
        core.time_code_mode,
        core._time_code_probe,
        core._trusted_time_code,
    )
    candidate_before = (
        core.candidate_sender,
        core._candidate_first_timestamp_ns,
    )
    stats_before = core.stats
    continuity_before = converter.previous_raw_quats_xyzw
    newest = core.newest_producer_monotonic_ns

    assert not core.check_stale(newest + 500_000_000)
    assert core.ready
    assert core.check_stale(newest + 500_000_001)

    assert core.ready_frames == 0
    assert core.current_pose_window() is None
    assert (
        core.source_epoch,
        core.locked_sender,
        core._sample_counter,
        core.newest_frame_index,
        core.newest_producer_monotonic_ns,
        core.time_code_mode,
        core._time_code_probe,
        core._trusted_time_code,
    ) == active_before
    assert (
        core.candidate_sender,
        core._candidate_first_timestamp_ns,
    ) == candidate_before
    assert core.candidate_frame_count == 1
    assert core._candidate_packets[0] is candidate
    assert core.stats == stats_before
    np.testing.assert_array_equal(
        converter.previous_raw_quats_xyzw, continuity_before
    )

    resumed = core.accept(
        make_packet(
            sample_counter=130,
            time_code=1030,
            receive_timestamp_ns=newest + 500_000_002,
        )
    )
    assert resumed.classification is AcceptClassification.FORWARD
    assert core.source_epoch == 91
    assert core.locked_sender == ("127.0.0.1", 4000, 0)
    assert core._sample_counter == 130
    assert core.newest_frame_index == 130
    assert core.time_code_mode is TimeCodeMode.ADVANCING
    assert core._trusted_time_code == 1030
    assert core.candidate_frame_count == 0
    assert core.ready_frames == 1


def test_approximate_neutral_need_not_match_t_pose():
    core, clock = make_core()
    positions = np.zeros((30, 23, 3), np.float32)
    pose = np.stack(
        [axis_angle_wxyz(i % 3, 5.0 + i) for i in range(23)]
    )
    quats = np.repeat(pose[None, :, :], 30, axis=0)
    feed_frame_sequence(core, clock, positions, quats)
    assert core.ready


def test_body_geometry_and_non_pelvis_positions_are_not_readiness_checks():
    core, clock = make_core()
    positions = np.zeros((30, 23, 3), np.float32)
    offsets = np.arange(1, 23, dtype=np.float32)
    positions[:, 1:, 0] = offsets[None, :] * 100.0
    positions[:, 1:, 1] = np.arange(30, dtype=np.float32)[:, None]
    quats = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    feed_frame_sequence(core, clock, positions, quats)
    assert core.ready


def test_global_pelvis_origin_does_not_affect_stability():
    for origin in ([0, 0, 0], [100, -40, 8]):
        core, clock = make_core()
        positions = np.zeros((30, 23, 3), np.float32)
        positions += np.asarray(origin, np.float32)
        quats = np.tile(
            np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
        )
        feed_frame_sequence(core, clock, positions, quats)
        assert core.ready


@pytest.mark.parametrize(
    "span,expected",
    [
        (np.float32(0.15), True),
        (
            np.nextafter(np.float32(0.15), np.float32(np.inf)),
            False,
        ),
    ],
)
def test_pelvis_diameter_boundary_is_inclusive(span, expected):
    core, clock = make_core(
        max_pelvis_span_m=float(np.float32(0.15))
    )
    positions = np.zeros((30, 23, 3), np.float32)
    positions[15:, 0, 0] = span
    quats = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    feed_frame_sequence(core, clock, positions, quats)
    assert core.ready is expected


def test_pelvis_stability_uses_pairwise_diameter_not_axis_span():
    core, clock = make_core()
    positions = np.zeros((30, 23, 3), np.float32)
    positions[10:20, 0, 0] = np.float32(0.11)
    positions[10:20, 0, 1] = np.float32(0.11)
    quats = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    feed_frame_sequence(core, clock, positions, quats)
    assert not core.ready


def test_angular_deviation_aligns_equivalent_quaternion_hemispheres():
    quaternion = np.array([0.0, 0.0, 0.5, 0.8660254], np.float32)
    samples = np.array([[quaternion], [-quaternion]], np.float32)
    deviations = angular_deviation_degrees(samples)
    np.testing.assert_allclose(deviations, 0.0, atol=1e-5)


@pytest.mark.parametrize("angle,expected", [(20.0, True), (20.01, False)])
def test_segment_p95_boundary_is_inclusive(angle, expected):
    core, clock = make_core(max_segment_deviation_deg=20.0)
    positions = np.zeros((30, 23, 3), np.float32)
    quats = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    quats[:15, 5] = axis_angle_wxyz(2, angle)
    quats[15:, 5] = axis_angle_wxyz(2, -angle)
    feed_frame_sequence(core, clock, positions, quats)
    assert core.ready is expected


def test_segment_metric_uses_p95_per_segment_not_a_global_average():
    core, clock = make_core()
    positions = np.zeros((30, 23, 3), np.float32)
    quats = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    quats[:15, 22] = axis_angle_wxyz(0, 21.0)
    quats[15:, 22] = axis_angle_wxyz(0, -21.0)
    feed_frame_sequence(core, clock, positions, quats)
    assert not core.ready


def test_segment_stability_uses_p95_instead_of_maximum_deviation():
    core, clock = make_core()
    positions = np.zeros((30, 23, 3), np.float32)
    quats = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    quats[0, 7] = axis_angle_wxyz(1, 90.0)
    feed_frame_sequence(core, clock, positions, quats)
    assert core.ready


def test_motion_above_either_threshold_blocks_ready():
    pelvis_core, pelvis_clock = make_core()
    positions = np.zeros((30, 23, 3), np.float32)
    positions[15:, 0, 0] = 0.151
    identity = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    feed_frame_sequence(pelvis_core, pelvis_clock, positions, identity)
    assert not pelvis_core.ready

    segment_core, segment_clock = make_core()
    moving = identity.copy()
    moving[:15, 9] = axis_angle_wxyz(1, 21.0)
    moving[15:, 9] = axis_angle_wxyz(1, -21.0)
    feed_frame_sequence(
        segment_core, segment_clock, np.zeros_like(positions), moving
    )
    assert not segment_core.ready


def test_rolling_readiness_recovers_after_unstable_evidence_exits():
    core, clock = make_core()
    positions = np.zeros((30, 23, 3), np.float32)
    positions[15:, 0, 0] = np.float32(0.151)
    quats = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    feed_frame_sequence(core, clock, positions, quats)
    assert not core.ready

    feed_stable_frames(core, clock, 29)
    assert not core.ready
    feed_stable_frames(core, clock, 1)
    assert core.ready


def test_forward_counter_gap_does_not_reset_readiness():
    core, clock = make_core()
    feed_stable_frames(core, clock, 15, epoch_counter_start=1)
    feed_stable_frames(core, clock, 15, epoch_counter_start=20)
    assert core.ready_frames == 30
    assert core.stats.inferred_missing_frames == 4
    assert core.ready


def test_only_accepted_frames_contribute_to_readiness_and_windows():
    core, clock = make_core()
    feed_stable_frames(core, clock, 10)
    duplicate = core.accept(
        make_packet(
            sample_counter=10,
            receive_timestamp_ns=clock.now_ns + 1,
        )
    )
    rejected = core.accept(
        make_packet(
            sample_counter=1,
            receive_timestamp_ns=clock.now_ns + 2,
        )
    )
    assert duplicate.classification is AcceptClassification.DUPLICATE
    assert rejected.accepted is False
    assert core.ready_frames == 10
    assert [
        frame.frame_index for frame in core.current_pose_window()
    ] == list(range(1, 11))

    feed_stable_frames(core, clock, 20, epoch_counter_start=11)
    assert core.ready_frames == 30
    assert core.stats.accepted == 30
    assert core.ready


def test_window_contains_ten_strictly_increasing_current_epoch_frames():
    core, clock = make_core()
    feed_stable_frames(core, clock, 30)
    indices = [frame.frame_index for frame in core.current_pose_window()]
    assert indices == list(range(21, 31))
    assert all(
        right > left for left, right in zip(indices, indices[1:])
    )

    feed_stable_frames(core, clock, 1)
    assert [
        frame.frame_index for frame in core.current_pose_window()
    ] == list(range(22, 32))


def test_pose_window_is_a_snapshot_when_the_internal_window_rolls():
    core, clock = make_core()
    feed_stable_frames(core, clock, 10)
    snapshot = core.current_pose_window()

    feed_stable_frames(core, clock, 1)

    assert [frame.frame_index for frame in snapshot] == list(range(1, 11))
    assert [
        frame.frame_index for frame in core.current_pose_window()
    ] == list(range(2, 12))


def test_epoch_change_clears_old_evidence_and_keeps_new_epoch_frames():
    core, clock = ready_core(epoch=91)
    first = feed_stable_frames(
        core, clock, 1, epoch_counter_start=1
    )[0]
    second = feed_stable_frames(
        core, clock, 1, epoch_counter_start=2
    )[0]
    assert first.accepted is False
    assert second.classification is AcceptClassification.EPOCH_COMMITTED
    assert core.source_epoch == 92
    assert core.ready_frames == 2
    assert core.current_pose_window() is None
    assert not core.ready

    feed_stable_frames(core, clock, 28, epoch_counter_start=3)
    assert core.ready
    assert [
        frame.frame_index for frame in core.current_pose_window()
    ] == list(range(21, 31))


def test_pose_window_tuple_frames_and_arrays_are_read_only():
    core, clock = make_core()
    feed_stable_frames(core, clock, 10)
    window = core.current_pose_window()
    assert isinstance(window, tuple)
    with pytest.raises(TypeError):
        window[0] = window[1]
    with pytest.raises(FrozenInstanceError):
        window[0].frame_index = 99
    with pytest.raises(ValueError):
        window[0].segment_positions_xrt[0, 0] = 99.0


def test_readiness_properties_are_getter_only_and_stats_stay_consistent():
    core, clock = make_core()
    feed_stable_frames(core, clock, 10)
    stats_before = core.stats
    assert core.ready_frames == stats_before.accepted == 10
    assert core.reference_window_ready
    assert core.current_pose_window() is not None
    assert core.stats == stats_before
    feed_stable_frames(core, clock, 21)
    assert core.ready_frames == 30
    assert core.stats.accepted == 31
    for name, value in (
        ("ready", True),
        ("ready_frames", 99),
        ("reference_window_ready", False),
    ):
        with pytest.raises(AttributeError):
            setattr(core, name, value)


def test_failed_conversion_rolls_back_readiness_and_output(monkeypatch):
    converter = XsensMotionConverter()
    core, clock = make_core(converter=converter)
    feed_stable_frames(core, clock, 29)
    window_before = core.current_pose_window()
    stats_before = core.stats
    newest_before = core.newest_frame_index
    convert = converter.convert
    monkeypatch.setattr(
        converter,
        "convert",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            RuntimeError("FK failed")
        ),
    )

    failed = feed_stable_frames(core, clock, 1)[0]

    assert failed.classification is AcceptClassification.CONVERSION_REJECTED
    assert core.ready_frames == 29
    assert core.current_pose_window() == window_before
    assert core.newest_frame_index == newest_before
    assert core.stats == replace(stats_before, conversion_failures=1)
    monkeypatch.setattr(converter, "convert", convert)
    feed_stable_frames(core, clock, 1)
    assert core.ready


def test_failed_epoch_conversion_preserves_ready_evidence_and_window(
    monkeypatch,
):
    converter = XsensMotionConverter()
    core, clock = make_core(converter=converter)
    positions = np.zeros((30, 23, 3), np.float32)
    positions[:, 0, 0] = np.linspace(0.0, 0.14, 30, dtype=np.float32)
    quats = np.tile(
        np.array([1, 0, 0, 0], np.float32), (30, 23, 1)
    )
    feed_frame_sequence(core, clock, positions, quats)
    assert core.ready

    evidence_before = tuple(core._readiness_frames)
    window_before = core.current_pose_window()
    clock.advance_ns(1)
    candidate = make_packet(
        sample_counter=1,
        time_code=0,
        receive_timestamp_ns=clock.now_ns,
    )
    started = core.accept(candidate)
    assert started.classification is AcceptClassification.CANDIDATE_STARTED
    active_before = (
        core.source_epoch,
        core.locked_sender,
        core._sample_counter,
        core.newest_frame_index,
        core.newest_producer_monotonic_ns,
        core.time_code_mode,
        core._time_code_probe,
        core._trusted_time_code,
        core.candidate_sender,
        core._candidate_first_timestamp_ns,
    )
    stats_before = core.stats
    continuity_before = converter.previous_raw_quats_xyzw

    def fail_epoch_conversion(items, *, reset_epoch=False):
        materialized = tuple(items)
        assert reset_epoch is True
        assert [frame_index for _, frame_index in materialized] == [1, 2]
        raise RuntimeError("epoch FK failed")

    monkeypatch.setattr(converter, "convert_many", fail_epoch_conversion)
    clock.advance_ns(1)
    failed = core.accept(
        make_packet(
            sample_counter=2,
            time_code=0,
            receive_timestamp_ns=clock.now_ns,
        )
    )

    assert failed.classification is AcceptClassification.CONVERSION_REJECTED
    assert core.ready
    assert core.ready_frames == 30
    assert [frame.frame_index for frame in evidence_before] == list(
        range(1, 31)
    )
    assert all(
        actual is expected
        for actual, expected in zip(core._readiness_frames, evidence_before)
    )
    window_after = core.current_pose_window()
    assert [frame.frame_index for frame in window_after] == list(
        range(21, 31)
    )
    assert all(
        actual is expected
        for actual, expected in zip(window_after, window_before)
    )
    assert (
        core.source_epoch,
        core.locked_sender,
        core._sample_counter,
        core.newest_frame_index,
        core.newest_producer_monotonic_ns,
        core.time_code_mode,
        core._time_code_probe,
        core._trusted_time_code,
        core.candidate_sender,
        core._candidate_first_timestamp_ns,
    ) == active_before
    assert core.candidate_frame_count == 1
    assert core._candidate_packets[0] is candidate
    assert core.stats == replace(stats_before, conversion_failures=1)
    np.testing.assert_array_equal(
        converter.previous_raw_quats_xyzw, continuity_before
    )
