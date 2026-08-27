from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from xsens.converter import XsensMotionConverter
from xsens.source_core import (
    AcceptClassification,
    AcceptResult,
    CounterDelta,
    CounterKind,
    SourceCoreStats,
    TimeCodeMode,
    classify_uint32_delta,
    draw_new_epoch,
)
from xsens_test_helpers import accept_packet, make_core, make_packet


@pytest.mark.parametrize(
    ("previous", "current", "kind", "missing"),
    [
        (10, 10, CounterKind.DUPLICATE, 0),
        (10, 11, CounterKind.FORWARD, 0),
        (10, 14, CounterKind.FORWARD, 3),
        (0xFFFFFFFF, 0, CounterKind.FORWARD, 0),
        (0, 0x80000000, CounterKind.AMBIGUOUS, 0),
        (1000, 900, CounterKind.BACKWARD, 0),
    ],
)
def test_uint32_counter_classifies_half_range(
    previous, current, kind, missing
):
    result = classify_uint32_delta(previous, current)
    assert result.kind is kind
    assert result.missing_frames == missing


def test_counter_delta_reports_literal_modular_values():
    assert classify_uint32_delta(0xFFFFFFFE, 1) == CounterDelta(
        CounterKind.FORWARD, 3, 2
    )
    assert classify_uint32_delta(0, 0x80000001) == CounterDelta(
        CounterKind.BACKWARD, 0x80000001, 0
    )


@pytest.mark.parametrize(
    ("previous", "current"),
    [
        (True, 1),
        (1, False),
        (1.0, 2),
        (1, 2.0),
        (-1, 0),
        (0, -1),
        (0x100000000, 0),
        (0, 0x100000000),
    ],
)
def test_uint32_counter_rejects_non_integer_bool_and_out_of_range(
    previous, current
):
    with pytest.raises(ValueError, match="uint32"):
        classify_uint32_delta(previous, current)


def test_epoch_draw_retries_zero_and_current_epoch():
    draws = iter([-1, 0, 1 << 63, 41, 42])
    assert draw_new_epoch(lambda: next(draws), current_epoch=41) == 42


def test_epoch_draw_retries_bool_non_integer_and_repeated_collision():
    draws = iter([True, False, 1.0, "42", 41, 41, 43])
    assert draw_new_epoch(lambda: next(draws), current_epoch=41) == 43


@pytest.mark.parametrize("current_epoch", [True, -1, 1.0, 1 << 63])
def test_epoch_draw_rejects_invalid_current_epoch(current_epoch):
    with pytest.raises(ValueError, match="current_epoch"):
        draw_new_epoch(lambda: 42, current_epoch=current_epoch)


@pytest.mark.parametrize(
    ("params", "match"),
    [
        ({"window_frames": 0}, "window_frames"),
        ({"window_frames": True}, "window_frames"),
        ({"same_epoch_resume_frames": 9}, "same_epoch_resume_frames"),
        ({"ready_frames": 0}, "ready_frames"),
        ({"epoch_candidate_frames": 1}, "epoch_candidate_frames"),
        ({"stale_seconds": 0.0}, "stale_seconds"),
        ({"epoch_candidate_timeout_s": 0.0}, "epoch_candidate_timeout_s"),
    ],
)
def test_core_rejects_invalid_counter_epoch_configuration(params, match):
    with pytest.raises(ValueError, match=match):
        make_core(**params)


def test_sender_lock_forward_gap_and_same_epoch_post_gap_recovery():
    core, _ = make_core(epoch_draws=(91, 92))
    first = accept_packet(core, 100, timestamp_ns=0)
    gap = accept_packet(core, 104, timestamp_ns=2_000_000_000)
    assert first.classification is AcceptClassification.INITIAL
    assert gap.classification is AcceptClassification.FORWARD
    assert gap.missing_frames == 3
    assert core.source_epoch == 91
    assert core.locked_sender == ("127.0.0.1", 4000, 0)
    assert core.newest_producer_monotonic_ns == 2_000_000_000
    assert core.stats == SourceCoreStats(
        accepted=2,
        inferred_missing_frames=3,
    )


def test_uint32_wrap_unwraps_to_strictly_increasing_frame_indices():
    core, _ = make_core(epoch_draws=(91, 92))
    expected = [
        (0xFFFFFFFE, 0xFFFFFFFE, 0),
        (0xFFFFFFFF, 0xFFFFFFFF, 0),
        (0, 0x100000000, 0),
        (3, 0x100000003, 2),
    ]
    for timestamp_ns, (counter, frame_index, missing) in enumerate(expected):
        result = accept_packet(
            core, counter, timestamp_ns=timestamp_ns
        )
        assert result.missing_frames == missing
        assert core.newest_frame_index == frame_index
    assert core.stats.inferred_missing_frames == 2


def test_active_packet_or_duplicate_cancels_pending_candidate():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 1000, timestamp_ns=0, time_code=1000)
    started = accept_packet(
        core, 900, timestamp_ns=10_000_000, time_code=900
    )
    assert started.classification is AcceptClassification.CANDIDATE_STARTED
    assert core.candidate_frame_count == 1
    accepted = accept_packet(
        core, 1001, timestamp_ns=20_000_000, time_code=1001
    )
    assert accepted.classification is AcceptClassification.FORWARD
    assert core.candidate_frame_count == 0

    accept_packet(core, 800, timestamp_ns=30_000_000, time_code=800)
    producer_before = core.newest_producer_monotonic_ns
    duplicate = accept_packet(
        core, 1001, timestamp_ns=40_000_000, time_code=1001
    )
    assert duplicate.classification is AcceptClassification.DUPLICATE
    assert core.candidate_frame_count == 0
    assert core.newest_producer_monotonic_ns == producer_before


def test_candidate_duplicate_replacement_and_strict_expiry():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 1000, timestamp_ns=0)
    accept_packet(core, 900, timestamp_ns=1_000_000_000)
    duplicate = accept_packet(core, 900, timestamp_ns=1_010_000_000)
    assert duplicate.classification is AcceptClassification.DUPLICATE
    assert core.candidate_frame_count == 1
    replacement = accept_packet(core, 800, timestamp_ns=1_020_000_000)
    assert (
        replacement.classification
        is AcceptClassification.CANDIDATE_REPLACED
    )
    assert core.candidate_frame_count == 1
    assert not core.expire(1_270_000_000)
    assert core.candidate_frame_count == 1
    assert core.expire(1_270_000_001)
    assert core.candidate_frame_count == 0
    assert not core.expire(2_000_000_000)


def test_accept_enforces_candidate_timeout_when_tick_has_not_expired_it():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 1000, timestamp_ns=0)
    accept_packet(core, 900, timestamp_ns=600_000_000)
    late = accept_packet(core, 901, timestamp_ns=850_000_001)
    assert late.classification is AcceptClassification.CANDIDATE_STARTED
    assert core.candidate_frame_count == 1
    assert core.source_epoch == 91


def test_two_candidate_frames_commit_atomically_with_original_time():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 1000, timestamp_ns=0)
    first = accept_packet(core, 900, timestamp_ns=600_000_000)
    second = accept_packet(core, 901, timestamp_ns=610_000_000)
    assert first.classification is AcceptClassification.CANDIDATE_STARTED
    assert second == AcceptResult(
        accepted=True,
        classification=AcceptClassification.EPOCH_COMMITTED,
        epoch_changed=True,
        missing_frames=0,
    )
    assert core.source_epoch == 92
    assert core.newest_frame_index == 901
    assert core.newest_producer_monotonic_ns == 610_000_000
    assert core.locked_sender == ("127.0.0.1", 4000, 0)
    assert core.stats.epoch_changes == 1


def test_candidate_forward_gap_counts_missing_in_new_epoch():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 1000, timestamp_ns=0)
    accept_packet(core, 900, timestamp_ns=600_000_000)
    result = accept_packet(core, 904, timestamp_ns=610_000_000)
    assert result == AcceptResult(
        accepted=True,
        classification=AcceptClassification.EPOCH_COMMITTED,
        epoch_changed=True,
        missing_frames=3,
    )
    assert core.newest_frame_index == 904
    assert core.stats.accepted == 3
    assert core.stats.inferred_missing_frames == 3


def test_time_code_modes_wrap_and_regression_evidence():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 10, timestamp_ns=0, time_code=0)
    accept_packet(core, 11, timestamp_ns=1, time_code=0)
    accept_packet(core, 12, timestamp_ns=2, time_code=0xFFFFFFFE)
    accept_packet(core, 13, timestamp_ns=3, time_code=0xFFFFFFFF)
    assert core.time_code_mode is TimeCodeMode.ADVANCING
    wrapped = accept_packet(core, 14, timestamp_ns=4, time_code=0)
    assert wrapped.classification is AcceptClassification.FORWARD
    half_range = accept_packet(
        core, 15, timestamp_ns=5, time_code=0x80000000
    )
    assert (
        half_range.classification
        is AcceptClassification.CANDIDATE_STARTED
    )

    # An active-compatible sample still wins over the pending candidate.
    resumed = accept_packet(core, 15, timestamp_ns=6, time_code=1)
    assert resumed.classification is AcceptClassification.FORWARD
    assert core.source_epoch == 91


def test_constant_to_advancing_allows_later_regression_commit():
    core, _ = make_core(epoch_draws=(91, 92))
    for counter, time_code in ((100, 0), (101, 0), (102, 50), (103, 51)):
        accept_packet(
            core, counter, timestamp_ns=counter, time_code=time_code
        )
    assert core.time_code_mode is TimeCodeMode.ADVANCING
    assert core.source_epoch == 91
    accept_packet(core, 104, timestamp_ns=104, time_code=0)
    committed = accept_packet(core, 105, timestamp_ns=105, time_code=1)
    assert committed.classification is AcceptClassification.EPOCH_COMMITTED
    assert core.source_epoch == 92


def test_advancing_time_code_duplicate_is_active_compatible():
    core, _ = make_core(epoch_draws=(91, 92))
    for counter, time_code in ((10, 40), (11, 41), (12, 41)):
        result = accept_packet(
            core, counter, timestamp_ns=counter, time_code=time_code
        )
    assert core.time_code_mode is TimeCodeMode.ADVANCING
    assert result.classification is AcceptClassification.FORWARD
    assert core.candidate_frame_count == 0


def test_counter_regression_wins_over_forward_time_code():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 1000, timestamp_ns=0, time_code=100)
    accept_packet(core, 1001, timestamp_ns=1, time_code=101)
    result = accept_packet(core, 900, timestamp_ns=2, time_code=102)
    assert result.classification is AcceptClassification.CANDIDATE_STARTED
    assert result.counter_kind is CounterKind.BACKWARD
    assert core.newest_frame_index == 1001
    assert core.newest_producer_monotonic_ns == 1


def test_changed_sender_requires_strictly_stale_active_producer():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(
        core, 1, timestamp_ns=0, sender=("127.0.0.1", 4000)
    )
    at_boundary = accept_packet(
        core,
        1,
        timestamp_ns=500_000_000,
        sender=("127.0.0.1", 5000),
    )
    assert (
        at_boundary.classification
        is AcceptClassification.SENDER_INELIGIBLE
    )
    first = accept_packet(
        core,
        1,
        timestamp_ns=500_000_001,
        sender=("127.0.0.1", 5000),
    )
    second = accept_packet(
        core,
        2,
        timestamp_ns=510_000_000,
        sender=("127.0.0.1", 5000),
    )
    assert first.classification is AcceptClassification.CANDIDATE_STARTED
    assert second.classification is AcceptClassification.EPOCH_COMMITTED
    assert core.locked_sender == ("127.0.0.1", 5000, 0)


def test_eligible_changed_sender_replaces_incoherent_candidate_sender():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 10, timestamp_ns=0)
    accept_packet(
        core,
        1,
        timestamp_ns=600_000_000,
        sender=("127.0.0.1", 5000),
    )
    replacement = accept_packet(
        core,
        20,
        timestamp_ns=610_000_000,
        sender=("127.0.0.1", 6000),
    )
    assert (
        replacement.classification
        is AcceptClassification.CANDIDATE_REPLACED
    )
    assert core.candidate_sender == ("127.0.0.1", 6000, 0)
    committed = accept_packet(
        core,
        21,
        timestamp_ns=620_000_000,
        sender=("127.0.0.1", 6000),
    )
    assert committed.classification is AcceptClassification.EPOCH_COMMITTED
    assert core.locked_sender == ("127.0.0.1", 6000, 0)


def test_fully_forward_same_sender_restart_is_unobservable():
    first, _ = make_core(epoch_draws=(91,))
    second, _ = make_core(epoch_draws=(92,))
    accept_packet(first, 700, timestamp_ns=0, time_code=700)
    accept_packet(second, 701, timestamp_ns=0, time_code=701)
    assert (first.source_epoch, second.source_epoch) == (91, 92)

    before = first.source_epoch
    result = accept_packet(
        first, 701, timestamp_ns=9_000_000_000, time_code=701
    )
    assert result.classification is AcceptClassification.FORWARD
    assert first.source_epoch == before  # MXTP02 exposes no restart evidence.


def test_failed_initial_conversion_changes_only_failure_stat(monkeypatch):
    converter = XsensMotionConverter()
    core, _ = make_core(epoch_draws=(91, 92), converter=converter)
    before = core.stats
    monkeypatch.setattr(
        converter,
        "convert",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            RuntimeError("FK failed")
        ),
    )
    result = accept_packet(core, 100, timestamp_ns=20)
    assert result == AcceptResult(
        accepted=False,
        classification=AcceptClassification.CONVERSION_REJECTED,
    )
    assert core.source_epoch == 91
    assert core.locked_sender is None
    assert core.newest_frame_index == -1
    assert core.newest_producer_monotonic_ns == 0
    assert core.stats == SourceCoreStats(conversion_failures=1)
    assert before == SourceCoreStats()


def test_failed_active_conversion_preserves_pending_candidate(monkeypatch):
    converter = XsensMotionConverter()
    core, _ = make_core(epoch_draws=(91, 92), converter=converter)
    accept_packet(core, 1000, timestamp_ns=0)
    accept_packet(core, 900, timestamp_ns=10)
    before = (
        core.source_epoch,
        core.locked_sender,
        core.newest_frame_index,
        core.newest_producer_monotonic_ns,
        core.candidate_frame_count,
        core.stats,
    )
    monkeypatch.setattr(
        converter,
        "convert",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            RuntimeError("FK failed")
        ),
    )
    result = accept_packet(core, 1001, timestamp_ns=300_000_011)
    assert result.classification is AcceptClassification.CONVERSION_REJECTED
    assert (
        core.source_epoch,
        core.locked_sender,
        core.newest_frame_index,
        core.newest_producer_monotonic_ns,
        core.candidate_frame_count,
    ) == before[:-1]
    assert core.stats == SourceCoreStats(
        accepted=1,
        backwards=1,
        reset_candidates=1,
        conversion_failures=1,
    )


def test_failed_second_candidate_conversion_does_not_partially_commit(
    monkeypatch,
):
    converter = XsensMotionConverter()
    core, _ = make_core(epoch_draws=(91, 92), converter=converter)
    accept_packet(core, 1000, timestamp_ns=0)
    accept_packet(core, 900, timestamp_ns=600_000_000)
    before = (
        core.source_epoch,
        core.locked_sender,
        core.newest_frame_index,
        core.newest_producer_monotonic_ns,
    )
    monkeypatch.setattr(
        converter,
        "convert_many",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            RuntimeError("FK failed")
        ),
    )
    result = accept_packet(core, 901, timestamp_ns=610_000_000)
    assert result.classification is AcceptClassification.CONVERSION_REJECTED
    assert (
        core.source_epoch,
        core.locked_sender,
        core.newest_frame_index,
        core.newest_producer_monotonic_ns,
    ) == before
    assert core.candidate_frame_count == 1
    assert core.stats == SourceCoreStats(
        accepted=1,
        backwards=1,
        reset_candidates=1,
        conversion_failures=1,
    )


def test_epoch_is_drawn_and_converter_reset_before_atomic_core_commit():
    class ObservingConverter(XsensMotionConverter):
        def __init__(self):
            super().__init__()
            self.core = None
            self.batch_observation = None

        def convert_many(self, items, *, reset_epoch=False):
            materialized = tuple(items)
            self.batch_observation = (
                reset_epoch,
                self.core.source_epoch,
                self.core.locked_sender,
                self.core.newest_frame_index,
                [
                    (frame_index, packet.receive_timestamp_ns)
                    for packet, frame_index in materialized
                ],
            )
            return super().convert_many(
                materialized, reset_epoch=reset_epoch
            )

    converter = ObservingConverter()
    core, _ = make_core(
        epoch_draws=(91, 0, 91, 92), converter=converter
    )
    converter.core = core
    accept_packet(core, 1000, timestamp_ns=0)
    negative = -np.tile(
        np.array([1, 0, 0, 0], dtype=np.float32), (23, 1)
    )
    core.accept(
        make_packet(
            sample_counter=900,
            receive_timestamp_ns=600_000_000,
            quaternions_wxyz=negative,
        )
    )
    result = core.accept(
        make_packet(
            sample_counter=901,
            receive_timestamp_ns=610_000_000,
            quaternions_wxyz=negative,
        )
    )
    assert result.classification is AcceptClassification.EPOCH_COMMITTED
    assert converter.batch_observation == (
        True,
        91,
        ("127.0.0.1", 4000, 0),
        1000,
        [(900, 600_000_000), (901, 610_000_000)],
    )
    assert core.source_epoch == 92
    assert np.all(converter.previous_raw_quats_xyzw[:, 3] < 0)


def test_source_properties_and_stats_are_read_only():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 10, timestamp_ns=1)
    for name, value in (
        ("source_epoch", 7),
        ("newest_frame_index", 7),
        ("newest_producer_monotonic_ns", 7),
        ("time_code_mode", TimeCodeMode.ADVANCING),
        ("candidate_frame_count", 7),
        ("candidate_sender", ("127.0.0.1", 7, 0)),
        ("locked_sender", ("127.0.0.1", 7, 0)),
        ("stats", SourceCoreStats()),
    ):
        with pytest.raises(AttributeError):
            setattr(core, name, value)
    with pytest.raises(FrozenInstanceError):
        core.stats.accepted = 99
