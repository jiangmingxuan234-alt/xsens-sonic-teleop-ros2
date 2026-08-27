"""Counter unwrapping and observable source-session tracking for Xsens."""

from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass, replace
from enum import Enum
import math
import operator
from threading import RLock

import numpy as np

from .converter import ConvertedXsensFrame, XsensMotionConverter
from .protocol import XsensPacket


_UINT32_MODULUS = 1 << 32
_UINT32_HALF_RANGE = 1 << 31
_MAX_SOURCE_EPOCH = (1 << 63) - 1
_NANOSECONDS_PER_SECOND = 1_000_000_000


class CounterKind(str, Enum):
    DUPLICATE = "duplicate"
    FORWARD = "forward"
    AMBIGUOUS = "ambiguous"
    BACKWARD = "backward"


class TimeCodeMode(str, Enum):
    UNKNOWN_OR_CONSTANT = "unknown_or_constant"
    ADVANCING = "advancing"


class AcceptClassification(str, Enum):
    INITIAL = "initial"
    FORWARD = "forward"
    DUPLICATE = "duplicate"
    CANDIDATE_STARTED = "candidate_started"
    CANDIDATE_REPLACED = "candidate_replaced"
    SENDER_INELIGIBLE = "sender_ineligible"
    EPOCH_COMMITTED = "epoch_committed"
    CONVERSION_REJECTED = "conversion_rejected"


@dataclass(frozen=True)
class CounterDelta:
    kind: CounterKind
    modular_delta: int
    missing_frames: int


@dataclass(frozen=True)
class AcceptResult:
    accepted: bool
    classification: AcceptClassification
    epoch_changed: bool = False
    missing_frames: int = 0
    counter_kind: CounterKind | None = None


@dataclass(frozen=True)
class SourceCoreStats:
    accepted: int = 0
    duplicates: int = 0
    ambiguous: int = 0
    backwards: int = 0
    sender_ineligible: int = 0
    reset_candidates: int = 0
    epoch_changes: int = 0
    inferred_missing_frames: int = 0
    conversion_failures: int = 0


def _integer_value(name: str, value) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer")
    try:
        return operator.index(value)
    except TypeError as error:
        raise ValueError(f"{name} must be an integer") from error


def _uint32_value(name: str, value) -> int:
    integer = _integer_value(name, value)
    if not 0 <= integer < _UINT32_MODULUS:
        raise ValueError(f"{name} must be a uint32 value")
    return integer


def classify_uint32_delta(previous: int, current: int) -> CounterDelta:
    """Classify a pair of uint32 counters using modular half-range order."""
    previous_value = _uint32_value("previous uint32 counter", previous)
    current_value = _uint32_value("current uint32 counter", current)
    delta = (current_value - previous_value) & 0xFFFFFFFF
    if delta == 0:
        return CounterDelta(CounterKind.DUPLICATE, 0, 0)
    if delta < _UINT32_HALF_RANGE:
        return CounterDelta(CounterKind.FORWARD, delta, delta - 1)
    if delta == _UINT32_HALF_RANGE:
        return CounterDelta(CounterKind.AMBIGUOUS, delta, 0)
    return CounterDelta(CounterKind.BACKWARD, delta, 0)


def draw_new_epoch(
    epoch_factory: Callable[[], int], *, current_epoch: int = 0
) -> int:
    """Draw a nonzero signed-int64 epoch distinct from the current one."""
    current = _integer_value("current_epoch", current_epoch)
    if not 0 <= current <= _MAX_SOURCE_EPOCH:
        raise ValueError("current_epoch must be in the range 0..2**63-1")
    while True:
        candidate = epoch_factory()
        if isinstance(candidate, bool):
            continue
        try:
            value = operator.index(candidate)
        except TypeError:
            continue
        if 1 <= value <= _MAX_SOURCE_EPOCH and value != current:
            return value


def _positive_integer(name: str, value, *, minimum: int = 1) -> int:
    integer = _integer_value(name, value)
    if integer < minimum:
        raise ValueError(f"{name} must be at least {minimum}")
    return integer


def _finite_float(
    name: str,
    value,
    *,
    minimum: float,
    maximum: float | None = None,
    inclusive_minimum: bool = True,
) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite") from error
    below_minimum = (
        number < minimum if inclusive_minimum else number <= minimum
    )
    if (
        not math.isfinite(number)
        or below_minimum
        or (maximum is not None and number > maximum)
    ):
        range_description = (
            f"greater than {minimum}"
            if not inclusive_minimum
            else f"at least {minimum}"
        )
        raise ValueError(f"{name} must be finite and {range_description}")
    return number


def _sender_key(packet: XsensPacket) -> tuple[str, int, int]:
    return (
        str(packet.sender_address[0]),
        int(packet.sender_address[1]),
        int(packet.header.character_id),
    )


def _next_time_code_state(
    mode: TimeCodeMode,
    probe: int | None,
    trusted: int | None,
    current: int,
) -> tuple[TimeCodeMode, int | None, int | None]:
    if mode is TimeCodeMode.ADVANCING:
        delta = classify_uint32_delta(trusted, current)
        if delta.kind is CounterKind.FORWARD:
            trusted = current
        return mode, probe, trusted

    if current == 0:
        return mode, probe, trusted
    if probe is None:
        return mode, current, trusted
    delta = classify_uint32_delta(probe, current)
    if delta.kind is CounterKind.FORWARD:
        return TimeCodeMode.ADVANCING, current, current
    return mode, current, trusted


def angular_deviation_degrees(samples_xyzw: np.ndarray) -> np.ndarray:
    """Measure each quaternion sample from its aligned segment mean."""
    aligned = np.asarray(samples_xyzw, dtype=np.float64).copy()
    aligned /= np.linalg.norm(aligned, axis=2, keepdims=True)
    aligned[
        np.einsum("fsc,sc->fs", aligned, aligned[0]) < 0.0
    ] *= -1.0
    mean = aligned.mean(axis=0)
    mean /= np.linalg.norm(mean, axis=1, keepdims=True)
    dots = np.abs(np.einsum("fsc,sc->fs", aligned, mean))
    return np.degrees(
        2.0 * np.arccos(np.clip(dots, 0.0, 1.0))
    )


class XsensSourceCore:
    """Classify parser-valid packets and commit observable source epochs."""

    def __init__(
        self,
        converter: XsensMotionConverter,
        *,
        clock_ns: Callable[[], int],
        epoch_factory: Callable[[], int],
        window_frames: int = 10,
        same_epoch_resume_frames: int = 10,
        ready_frames: int = 30,
        stale_seconds: float = 0.5,
        epoch_candidate_frames: int = 2,
        epoch_candidate_timeout_s: float = 0.25,
        max_pelvis_span_m: float = 0.15,
        max_segment_deviation_deg: float = 20.0,
    ) -> None:
        window_frames = _positive_integer("window_frames", window_frames)
        same_epoch_resume_frames = _positive_integer(
            "same_epoch_resume_frames", same_epoch_resume_frames
        )
        if same_epoch_resume_frames != window_frames:
            raise ValueError(
                "same_epoch_resume_frames must equal window_frames"
            )
        ready_frames = _positive_integer("ready_frames", ready_frames)
        epoch_candidate_frames = _positive_integer(
            "epoch_candidate_frames", epoch_candidate_frames, minimum=2
        )
        stale_seconds = _finite_float(
            "stale_seconds",
            stale_seconds,
            minimum=0.0,
            inclusive_minimum=False,
        )
        epoch_candidate_timeout_s = _finite_float(
            "epoch_candidate_timeout_s",
            epoch_candidate_timeout_s,
            minimum=0.0,
            inclusive_minimum=False,
        )
        max_pelvis_span_m = _finite_float(
            "max_pelvis_span_m", max_pelvis_span_m, minimum=0.0
        )
        max_segment_deviation_deg = _finite_float(
            "max_segment_deviation_deg",
            max_segment_deviation_deg,
            minimum=0.0,
            maximum=180.0,
        )
        if not callable(clock_ns):
            raise ValueError("clock_ns must be callable")
        if not callable(epoch_factory):
            raise ValueError("epoch_factory must be callable")

        self._lock = RLock()
        self._converter = converter
        self._clock_ns = clock_ns
        self._epoch_factory = epoch_factory
        self._window_frames = window_frames
        self._same_epoch_resume_frames = same_epoch_resume_frames
        self._ready_frames = ready_frames
        self._stale_ns = int(stale_seconds * _NANOSECONDS_PER_SECOND)
        self._epoch_candidate_frames = epoch_candidate_frames
        self._epoch_candidate_timeout_ns = int(
            epoch_candidate_timeout_s * _NANOSECONDS_PER_SECOND
        )
        self._max_pelvis_span_m = np.float32(max_pelvis_span_m)
        self._max_segment_deviation_deg = max_segment_deviation_deg

        self._source_epoch = draw_new_epoch(epoch_factory)
        self._locked_sender = None
        self._sample_counter = None
        self._newest_frame_index = -1
        self._newest_producer_monotonic_ns = 0
        self._time_code_mode = TimeCodeMode.UNKNOWN_OR_CONSTANT
        self._time_code_probe = None
        self._trusted_time_code = None
        self._candidate_packets: list[XsensPacket] = []
        self._candidate_sender = None
        self._candidate_first_timestamp_ns = None
        self._output_frames: deque[ConvertedXsensFrame] = deque(
            maxlen=window_frames
        )
        self._readiness_frames: deque[ConvertedXsensFrame] = deque(
            maxlen=ready_frames
        )
        self._stale_handled = False
        self._stats = SourceCoreStats()

    @property
    def stats(self) -> SourceCoreStats:
        with self._lock:
            return self._stats

    @property
    def source_epoch(self) -> int:
        with self._lock:
            return self._source_epoch

    @property
    def newest_frame_index(self) -> int:
        with self._lock:
            return self._newest_frame_index

    @property
    def newest_producer_monotonic_ns(self) -> int:
        with self._lock:
            return self._newest_producer_monotonic_ns

    @property
    def ready_frames(self) -> int:
        with self._lock:
            return len(self._readiness_frames)

    @property
    def reference_window_ready(self) -> bool:
        with self._lock:
            return len(self._output_frames) == self._window_frames

    def _readiness_is_stable(self) -> bool:
        pelvis_positions = np.stack(
            [
                frame.segment_positions_xrt[0]
                for frame in self._readiness_frames
            ]
        )
        pelvis_deltas = (
            pelvis_positions[:, None, :] - pelvis_positions[None, :, :]
        )
        pelvis_diameter = np.sqrt(
            np.max(np.einsum("fgi,fgi->fg", pelvis_deltas, pelvis_deltas))
        )
        if pelvis_diameter > self._max_pelvis_span_m:
            return False

        segment_quaternions = np.stack(
            [
                frame.segment_quat_xrt_xyzw
                for frame in self._readiness_frames
            ]
        )
        deviations = angular_deviation_degrees(segment_quaternions)
        segment_p95 = np.percentile(deviations, 95.0, axis=0)
        return bool(
            np.all(
                segment_p95
                <= self._max_segment_deviation_deg + 1e-5
            )
        )

    @property
    def ready(self) -> bool:
        with self._lock:
            return (
                len(self._readiness_frames) == self._ready_frames
                and len(self._output_frames) == self._window_frames
                and self._readiness_is_stable()
            )

    def current_pose_window(
        self,
    ) -> tuple[ConvertedXsensFrame, ...] | None:
        with self._lock:
            if len(self._output_frames) != self._window_frames:
                return None
            return tuple(self._output_frames)

    def check_stale(self, now_ns: int) -> bool:
        """Clear unarmed readiness once on the strict producer-age edge."""
        now = _integer_value("now_ns", now_ns)
        with self._lock:
            if (
                self._locked_sender is None
                or self._stale_handled
                or now - self._newest_producer_monotonic_ns
                <= self._stale_ns
            ):
                return False
            self._output_frames.clear()
            self._readiness_frames.clear()
            self._stale_handled = True
            return True

    @property
    def time_code_mode(self) -> TimeCodeMode:
        with self._lock:
            return self._time_code_mode

    @property
    def candidate_frame_count(self) -> int:
        with self._lock:
            return len(self._candidate_packets)

    @property
    def candidate_sender(self) -> tuple[str, int, int] | None:
        with self._lock:
            return self._candidate_sender

    @property
    def locked_sender(self) -> tuple[str, int, int] | None:
        with self._lock:
            return self._locked_sender

    def _bump_stats(self, **increments: int) -> None:
        values = {
            field: getattr(self._stats, field) + increment
            for field, increment in increments.items()
        }
        self._stats = replace(self._stats, **values)

    def _clear_candidate(self) -> None:
        self._candidate_packets = []
        self._candidate_sender = None
        self._candidate_first_timestamp_ns = None

    def _expire_candidate(self, now_ns: int) -> bool:
        if not self._candidate_is_expired(now_ns):
            return False
        self._clear_candidate()
        return True

    def _candidate_is_expired(self, now_ns: int) -> bool:
        if not self._candidate_packets:
            return False
        age_ns = now_ns - self._candidate_first_timestamp_ns
        return age_ns > self._epoch_candidate_timeout_ns

    def expire(self, now_ns: int) -> bool:
        """Expire a reset candidate after the strict candidate-age edge."""
        now = _integer_value("now_ns", now_ns)
        with self._lock:
            return self._expire_candidate(now)

    def _counter_anomaly_increment(
        self, counter_kind: CounterKind | None
    ) -> dict[str, int]:
        if counter_kind is CounterKind.AMBIGUOUS:
            return {"ambiguous": 1}
        if counter_kind is CounterKind.BACKWARD:
            return {"backwards": 1}
        return {}

    def _time_code_regressed(self, current: int) -> bool:
        if self._time_code_mode is not TimeCodeMode.ADVANCING:
            return False
        delta = classify_uint32_delta(self._trusted_time_code, current)
        return delta.kind in (CounterKind.AMBIGUOUS, CounterKind.BACKWARD)

    def _append_accepted_frame(self, frame: ConvertedXsensFrame) -> None:
        self._output_frames.append(frame)
        self._readiness_frames.append(frame)
        self._stale_handled = False

    def _conversion_rejected(self) -> AcceptResult:
        self._bump_stats(conversion_failures=1)
        return AcceptResult(
            accepted=False,
            classification=AcceptClassification.CONVERSION_REJECTED,
        )

    def _accept_initial(
        self,
        packet: XsensPacket,
        sender: tuple[str, int, int],
        counter: int,
        time_code: int,
    ) -> AcceptResult:
        try:
            frame = self._converter.convert(packet, frame_index=counter)
        except Exception:
            return self._conversion_rejected()

        mode, probe, trusted = _next_time_code_state(
            TimeCodeMode.UNKNOWN_OR_CONSTANT, None, None, time_code
        )
        self._locked_sender = sender
        self._sample_counter = counter
        self._newest_frame_index = counter
        self._newest_producer_monotonic_ns = int(
            packet.receive_timestamp_ns
        )
        self._time_code_mode = mode
        self._time_code_probe = probe
        self._trusted_time_code = trusted
        self._clear_candidate()
        self._append_accepted_frame(frame)
        self._bump_stats(accepted=1)
        return AcceptResult(
            accepted=True,
            classification=AcceptClassification.INITIAL,
        )

    def _accept_active_forward(
        self,
        packet: XsensPacket,
        counter: int,
        time_code: int,
        counter_delta: CounterDelta,
    ) -> AcceptResult:
        frame_index = self._newest_frame_index + counter_delta.modular_delta
        try:
            frame = self._converter.convert(
                packet, frame_index=frame_index
            )
        except Exception:
            return self._conversion_rejected()

        mode, probe, trusted = _next_time_code_state(
            self._time_code_mode,
            self._time_code_probe,
            self._trusted_time_code,
            time_code,
        )
        self._sample_counter = counter
        self._newest_frame_index = frame_index
        self._newest_producer_monotonic_ns = int(
            packet.receive_timestamp_ns
        )
        self._time_code_mode = mode
        self._time_code_probe = probe
        self._trusted_time_code = trusted
        self._clear_candidate()
        self._append_accepted_frame(frame)
        self._bump_stats(
            accepted=1,
            inferred_missing_frames=counter_delta.missing_frames,
        )
        return AcceptResult(
            accepted=True,
            classification=AcceptClassification.FORWARD,
            missing_frames=counter_delta.missing_frames,
            counter_kind=CounterKind.FORWARD,
        )

    def _start_or_replace_candidate(
        self,
        packet: XsensPacket,
        sender: tuple[str, int, int],
        counter_kind: CounterKind | None,
    ) -> AcceptResult:
        classification = (
            AcceptClassification.CANDIDATE_REPLACED
            if self._candidate_packets
            else AcceptClassification.CANDIDATE_STARTED
        )
        self._candidate_packets = [packet]
        self._candidate_sender = sender
        self._candidate_first_timestamp_ns = int(
            packet.receive_timestamp_ns
        )
        increments = {"reset_candidates": 1}
        increments.update(self._counter_anomaly_increment(counter_kind))
        self._bump_stats(**increments)
        return AcceptResult(
            accepted=False,
            classification=classification,
            counter_kind=counter_kind,
        )

    def _candidate_items(
        self, packets: Iterable[XsensPacket]
    ) -> tuple[tuple[tuple[XsensPacket, int], ...], int]:
        packets = tuple(packets)
        frame_index = _uint32_value(
            "candidate sample counter", packets[0].header.sample_counter
        )
        items = [(packets[0], frame_index)]
        missing_frames = 0
        previous_counter = packets[0].header.sample_counter
        for packet in packets[1:]:
            delta = classify_uint32_delta(
                previous_counter, packet.header.sample_counter
            )
            frame_index += delta.modular_delta
            missing_frames += delta.missing_frames
            items.append((packet, frame_index))
            previous_counter = packet.header.sample_counter
        return tuple(items), missing_frames

    def _commit_candidate(
        self,
        packets: tuple[XsensPacket, ...],
        sender: tuple[str, int, int],
    ) -> AcceptResult:
        items, missing_frames = self._candidate_items(packets)
        new_epoch = draw_new_epoch(
            self._epoch_factory, current_epoch=self._source_epoch
        )
        try:
            frames = self._converter.convert_many(
                items, reset_epoch=True
            )
        except Exception:
            return self._conversion_rejected()

        mode = TimeCodeMode.UNKNOWN_OR_CONSTANT
        probe = None
        trusted = None
        for packet in packets:
            time_code = _uint32_value(
                "packet time code", packet.header.time_code
            )
            mode, probe, trusted = _next_time_code_state(
                mode, probe, trusted, time_code
            )

        final_packet = packets[-1]
        final_frame_index = items[-1][1]
        self._source_epoch = new_epoch
        self._locked_sender = sender
        self._sample_counter = _uint32_value(
            "packet sample counter", final_packet.header.sample_counter
        )
        self._newest_frame_index = final_frame_index
        self._newest_producer_monotonic_ns = int(
            final_packet.receive_timestamp_ns
        )
        self._time_code_mode = mode
        self._time_code_probe = probe
        self._trusted_time_code = trusted
        self._clear_candidate()
        self._output_frames.clear()
        self._readiness_frames.clear()
        for frame in frames:
            self._append_accepted_frame(frame)
        self._bump_stats(
            accepted=len(frames),
            epoch_changes=1,
            inferred_missing_frames=missing_frames,
        )
        return AcceptResult(
            accepted=True,
            classification=AcceptClassification.EPOCH_COMMITTED,
            epoch_changed=True,
            missing_frames=missing_frames,
        )

    def _offer_candidate(
        self,
        packet: XsensPacket,
        sender: tuple[str, int, int],
        counter_kind: CounterKind | None,
    ) -> AcceptResult:
        if not self._candidate_packets or self._candidate_sender != sender:
            return self._start_or_replace_candidate(
                packet, sender, counter_kind
            )

        candidate_delta = classify_uint32_delta(
            self._candidate_packets[-1].header.sample_counter,
            packet.header.sample_counter,
        )
        if candidate_delta.kind is CounterKind.DUPLICATE:
            self._bump_stats(duplicates=1)
            return AcceptResult(
                accepted=False,
                classification=AcceptClassification.DUPLICATE,
                counter_kind=CounterKind.DUPLICATE,
            )
        if candidate_delta.kind is not CounterKind.FORWARD:
            return self._start_or_replace_candidate(
                packet, sender, counter_kind
            )

        packets = tuple((*self._candidate_packets, packet))
        if len(packets) < self._epoch_candidate_frames:
            self._candidate_packets.append(packet)
            return AcceptResult(
                accepted=False,
                classification=AcceptClassification.CANDIDATE_STARTED,
                counter_kind=counter_kind,
            )
        return self._commit_candidate(packets, sender)

    def accept(self, packet: XsensPacket) -> AcceptResult:
        """Classify and transactionally accept one parser-valid packet."""
        counter = _uint32_value(
            "packet sample counter", packet.header.sample_counter
        )
        time_code = _uint32_value(
            "packet time code", packet.header.time_code
        )
        sender = _sender_key(packet)
        timestamp_ns = _integer_value(
            "packet receive_timestamp_ns", packet.receive_timestamp_ns
        )

        with self._lock:
            if self._locked_sender is None:
                return self._accept_initial(
                    packet, sender, counter, time_code
                )

            candidate_expired = self._candidate_is_expired(timestamp_ns)
            if sender == self._locked_sender:
                counter_delta = classify_uint32_delta(
                    self._sample_counter, counter
                )
                if (
                    counter_delta.kind is CounterKind.FORWARD
                    and not self._time_code_regressed(time_code)
                ):
                    return self._accept_active_forward(
                        packet, counter, time_code, counter_delta
                    )
                if counter_delta.kind is CounterKind.DUPLICATE:
                    self._clear_candidate()
                    self._bump_stats(duplicates=1)
                    return AcceptResult(
                        accepted=False,
                        classification=AcceptClassification.DUPLICATE,
                        counter_kind=CounterKind.DUPLICATE,
                    )
                if candidate_expired:
                    self._clear_candidate()
                return self._offer_candidate(
                    packet, sender, counter_delta.kind
                )

            active_age_ns = (
                timestamp_ns - self._newest_producer_monotonic_ns
            )
            if active_age_ns <= self._stale_ns:
                if candidate_expired:
                    self._clear_candidate()
                self._bump_stats(sender_ineligible=1)
                return AcceptResult(
                    accepted=False,
                    classification=AcceptClassification.SENDER_INELIGIBLE,
                )
            if candidate_expired:
                self._clear_candidate()
            return self._offer_candidate(packet, sender, None)


__all__ = [
    "AcceptClassification",
    "AcceptResult",
    "CounterDelta",
    "CounterKind",
    "SourceCoreStats",
    "TimeCodeMode",
    "XsensSourceCore",
    "angular_deviation_degrees",
    "classify_uint32_delta",
    "draw_new_epoch",
]
