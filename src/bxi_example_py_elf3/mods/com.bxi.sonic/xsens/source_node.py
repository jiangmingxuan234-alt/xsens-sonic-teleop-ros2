"""State-scoped ROS, UDP, and ZMQ boundary for the Xsens source."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import math
import operator
import secrets
from threading import RLock
import time

import numpy as np
import zmq
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Int64MultiArray

from bxi_example_py_elf3.framework.mod_api import NodeBuildContext

from .converter import XsensMotionConverter
from .protocol import XsensProtocolError, parse_mxtp02_packet
from .source_core import (
    ArmCommand,
    CounterKind,
    XsensSourceCore,
)
from .udp_receiver import XsensUdpReceiver

if __package__ == "xsens":
    from pico.zmq_messages import pack_pose_message
else:
    from ..pico.zmq_messages import pack_pose_message


SOURCE_DEFAULTS: dict[str, object] = {
    "udp_bind_host": "0.0.0.0",
    "udp_port": 9763,
    "allowed_sender": "127.0.0.1",
    "pose_host": "127.0.0.1",
    "pose_port": 5559,
    "pose_topic": "pose",
    "status_topic": "xsens_status",
    "arm_command_topic": "sonic/xsens_arm_command",
    "status_rate_hz": 50.0,
    "input_rate_hz": 60.0,
    "publish_rate_hz": 50.0,
    "window_frames": 10,
    "same_epoch_resume_frames": 10,
    "ready_frames": 30,
    "stale_seconds": 0.5,
    "epoch_candidate_frames": 2,
    "epoch_candidate_timeout_s": 0.25,
    "max_pelvis_span_m": 0.15,
    "max_segment_deviation_deg": 20.0,
}

DIAGNOSTIC_SUMMARY_SECONDS = 5.0
_DIAGNOSTIC_SUMMARY_NS = int(
    DIAGNOSTIC_SUMMARY_SECONDS * 1_000_000_000
)
_SIGNED_INT64_MIN = -(1 << 63)
_SIGNED_INT64_MAX = (1 << 63) - 1


def _strict_fixed_number(
    params: Mapping[str, object], name: str, expected: float
) -> None:
    value = params[name]
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or float(value) != expected
    ):
        raise ValueError(f"{name} must be exactly {expected}")


def _finite_number(
    params: Mapping[str, object],
    name: str,
    *,
    minimum: float,
    maximum: float | None = None,
    minimum_inclusive: bool = True,
) -> None:
    value = params[name]
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite")
    below = value < minimum if minimum_inclusive else value <= minimum
    if below or (maximum is not None and value > maximum):
        raise ValueError(f"{name} is outside its safe range")


def validate_source_params(
    raw: Mapping[str, object],
) -> dict[str, object]:
    """Merge defaults and reject unsafe Xsens source configuration."""
    unknown = set(raw) - set(SOURCE_DEFAULTS)
    if unknown:
        raise ValueError(f"unknown Xsens source params: {sorted(unknown)}")
    params = {
        name: raw.get(name, default)
        for name, default in SOURCE_DEFAULTS.items()
    }

    for name in (
        "udp_bind_host",
        "pose_topic",
        "status_topic",
        "arm_command_topic",
    ):
        if not isinstance(params[name], str) or not params[name]:
            raise ValueError(f"{name} must be a non-empty string")
    if not isinstance(params["allowed_sender"], str):
        raise ValueError("allowed_sender must be a string")
    if params["pose_host"] != "127.0.0.1":
        raise ValueError("pose_host must be 127.0.0.1")

    for name in ("udp_port", "pose_port"):
        value = params[name]
        if type(value) is not int or not 1 <= value <= 65535:
            raise ValueError(f"{name} must be an integer from 1 to 65535")

    _strict_fixed_number(params, "status_rate_hz", 50.0)
    _strict_fixed_number(params, "input_rate_hz", 60.0)
    _strict_fixed_number(params, "publish_rate_hz", 50.0)

    for name, expected in (
        ("window_frames", 10),
        ("same_epoch_resume_frames", 10),
        ("ready_frames", 30),
    ):
        value = params[name]
        if type(value) is not int or value != expected:
            raise ValueError(f"{name} must be exactly {expected}")
    candidate_frames = params["epoch_candidate_frames"]
    if type(candidate_frames) is not int or candidate_frames < 2:
        raise ValueError(
            "epoch_candidate_frames must be an integer at least 2"
        )

    _finite_number(
        params,
        "stale_seconds",
        minimum=0.0,
        minimum_inclusive=False,
    )
    _finite_number(
        params,
        "epoch_candidate_timeout_s",
        minimum=0.0,
        minimum_inclusive=False,
    )
    _finite_number(params, "max_pelvis_span_m", minimum=0.0)
    _finite_number(
        params,
        "max_segment_deviation_deg",
        minimum=0.0,
        maximum=180.0,
    )
    return params


def random_epoch_candidate() -> int:
    """Draw one raw 63-bit candidate; the source core owns retry rules."""
    return secrets.randbits(63)


class XsensPoseStatusPublisher:
    """Own one PUB socket and, optionally, the ZMQ context behind it."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        zmq_context: zmq.Context | None = None,
    ) -> None:
        self._owns_context = zmq_context is None
        self._context = (
            zmq_context if zmq_context is not None else zmq.Context()
        )
        self._socket = None
        self._closed = False
        self._drop_count = 0
        self._lock = RLock()
        try:
            self._socket = self._context.socket(zmq.PUB)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.bind(f"tcp://{host}:{port}")
        except Exception:
            if self._socket is not None:
                try:
                    self._socket.close(linger=0)
                except Exception:
                    pass
            if self._owns_context:
                self._context.term()
            self._closed = True
            raise

    @property
    def drop_count(self) -> int:
        with self._lock:
            return self._drop_count

    def send(self, topic: str, fields: Mapping[str, np.ndarray]) -> bool:
        """Pack and attempt one nonblocking, single-frame publication."""
        message = pack_pose_message(dict(fields), topic=topic)
        with self._lock:
            if self._closed:
                return False
            try:
                self._socket.send(message, flags=zmq.NOBLOCK)
            except zmq.Again:
                self._drop_count += 1
                return False
            return True

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._socket.close(linger=0)
            finally:
                if self._owns_context:
                    self._context.term()


class XsensSourceNode(Node):
    """Run Xsens resources only for the lifetime of this ROS node."""

    def __init__(
        self,
        context: NodeBuildContext,
        *,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        epoch_factory: Callable[[], int] = random_epoch_candidate,
        receiver_factory: Callable[..., XsensUdpReceiver] = XsensUdpReceiver,
        converter_factory: Callable[[], XsensMotionConverter] = (
            XsensMotionConverter
        ),
        core_factory: Callable[..., XsensSourceCore] = XsensSourceCore,
        publisher_factory: Callable[..., XsensPoseStatusPublisher] = (
            XsensPoseStatusPublisher
        ),
    ) -> None:
        params = validate_source_params(context.params)
        super().__init__(
            context.node_name, namespace=context.namespace or None
        )

        self._clock_ns = clock_ns
        self._lock = RLock()
        self._receiver = None
        self._converter = None
        self._core = None
        self._publisher = None
        self._subscription = None
        self._timer = None
        self._closed = False
        self._destroy_result = True
        self._pose_topic = str(params["pose_topic"])
        self._status_topic = str(params["status_topic"])
        self._pose_pending = False
        self._last_published_frame_index = None
        self._malformed_packets = 0
        self._accepted_results = 0
        self._duplicate_results = 0
        self._ambiguous_results = 0
        self._backward_results = 0
        self._inferred_drops = 0
        self._last_accept_classification = None
        self._last_diagnostic_state = None
        self._last_diagnostic_summary_ns = None
        self._last_diagnostic_accepted = 0
        self._last_successful_status_ns = None

        try:
            allowed_sender = str(params["allowed_sender"])
            self._receiver = receiver_factory(
                bind_host=str(params["udp_bind_host"]),
                port=int(params["udp_port"]),
                allowed_sender_host=allowed_sender or None,
                clock_ns=clock_ns,
            )
            self._converter = converter_factory()
            self._core = core_factory(
                self._converter,
                clock_ns=clock_ns,
                epoch_factory=epoch_factory,
                window_frames=int(params["window_frames"]),
                same_epoch_resume_frames=int(
                    params["same_epoch_resume_frames"]
                ),
                ready_frames=int(params["ready_frames"]),
                stale_seconds=float(params["stale_seconds"]),
                epoch_candidate_frames=int(params["epoch_candidate_frames"]),
                epoch_candidate_timeout_s=float(
                    params["epoch_candidate_timeout_s"]
                ),
                max_pelvis_span_m=float(params["max_pelvis_span_m"]),
                max_segment_deviation_deg=float(
                    params["max_segment_deviation_deg"]
                ),
            )
            self._publisher = publisher_factory(
                str(params["pose_host"]), int(params["pose_port"])
            )
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._subscription = self.create_subscription(
                Int64MultiArray,
                str(params["arm_command_topic"]),
                self._arm_command_callback,
                qos,
            )
            self._timer = self.create_timer(
                1.0 / float(params["status_rate_hz"]), self._tick
            )
            now_ns = operator.index(self._clock_ns())
            with self._lock:
                status_fields, _ = self._publish_status(now_ns)
                self._maybe_log_diagnostics(now_ns, status_fields)
        except Exception:
            self._cleanup_failed_construction()
            raise

    @staticmethod
    def _signed_int64(value) -> int:
        if isinstance(value, (bool, np.bool_)):
            raise ValueError(
                "arm command values must be signed int64 integers"
            )
        try:
            integer = operator.index(value)
        except TypeError as error:
            raise ValueError(
                "arm command values must be signed int64 integers"
            ) from error
        if not _SIGNED_INT64_MIN <= integer <= _SIGNED_INT64_MAX:
            raise ValueError("arm command values must fit signed int64")
        return integer

    def _arm_command_callback(self, message: Int64MultiArray) -> None:
        with self._lock:
            if self._closed:
                return
            try:
                if (
                    list(message.layout.dim)
                    or message.layout.data_offset != 0
                    or len(message.data) != 3
                ):
                    raise ValueError("invalid Int64MultiArray layout")
                values = tuple(
                    self._signed_int64(value) for value in message.data
                )
            except (AttributeError, TypeError, ValueError):
                self._core.note_invalid_input()
                return

            try:
                result = self._core.handle_arm_command(ArmCommand(*values))
                if result.publish_immediately:
                    now_ns = operator.index(self._clock_ns())
                    status_fields, _ = self._publish_status(now_ns)
                    self._maybe_log_diagnostics(now_ns, status_fields)
            except Exception as error:
                self.get_logger().error(
                    f"Xsens arm command callback failed: {error}"
                )

    def _publish_status(self, now_ns: int):
        try:
            fields = self._core.build_status_fields(now_ns)
            sent = self._publisher.send(self._status_topic, fields)
            if sent:
                self._last_successful_status_ns = now_ns
            return fields, bool(sent)
        except Exception as error:
            self.get_logger().error(
                f"Xsens status publication failed: {error}"
            )
            return None, False

    def _observe_accept_result(self, result) -> None:
        self._last_accept_classification = result.classification
        if result.accepted:
            self._accepted_results += 1
        if result.counter_kind is CounterKind.DUPLICATE:
            self._duplicate_results += 1
        elif result.counter_kind is CounterKind.AMBIGUOUS:
            self._ambiguous_results += 1
        elif result.counter_kind is CounterKind.BACKWARD:
            self._backward_results += 1
        self._inferred_drops += int(result.missing_frames)

    def _drain_and_accept(self) -> None:
        try:
            datagrams = self._receiver.drain()
        except Exception as error:
            self.get_logger().error(f"Xsens UDP drain failed: {error}")
            return

        for datagram in datagrams:
            try:
                packet = parse_mxtp02_packet(
                    datagram.payload,
                    receive_timestamp_ns=datagram.receive_timestamp_ns,
                    sender_address=datagram.sender_address,
                )
            except XsensProtocolError:
                self._malformed_packets += 1
                self._core.note_invalid_input()
                continue

            try:
                self._core.check_stale(packet.receive_timestamp_ns)
                result = self._core.accept(packet)
            except Exception as error:
                self._core.note_invalid_input()
                self.get_logger().error(
                    f"Xsens packet acceptance failed: {error}"
                )
                continue
            self._observe_accept_result(result)
            if result.accepted:
                self._pose_pending = True

    @staticmethod
    def _status_scalar(fields, name, default=0):
        if fields is None or name not in fields:
            return default
        value = np.asarray(fields[name]).reshape(-1)
        return default if value.size == 0 else value[0].item()

    def _diagnostic_state(self, status_fields):
        return (
            int(self._status_scalar(status_fields, "source_epoch")),
            bool(self._status_scalar(status_fields, "ready")),
            bool(self._status_scalar(status_fields, "reference_window_ready")),
            bool(self._status_scalar(status_fields, "source_stale")),
            int(self._status_scalar(status_fields, "reason_code")),
            int(self._status_scalar(status_fields, "recovery_frames")),
            str(self._last_accept_classification),
        )

    def _maybe_log_diagnostics(self, now_ns: int, status_fields) -> None:
        if status_fields is None:
            return
        state = self._diagnostic_state(status_fields)
        changed = (
            self._last_diagnostic_state is not None
            and state != self._last_diagnostic_state
        )
        summary_due = (
            self._last_diagnostic_summary_ns is not None
            and now_ns - self._last_diagnostic_summary_ns
            >= _DIAGNOSTIC_SUMMARY_NS
        )
        first = self._last_diagnostic_state is None
        self._last_diagnostic_state = state
        if first:
            self._last_diagnostic_summary_ns = now_ns
            return
        if not changed and not summary_due:
            return

        core_stats = self._core.stats
        receiver_stats = self._receiver.stats
        elapsed_ns = max(
            1, now_ns - int(self._last_diagnostic_summary_ns or now_ns)
        )
        accepted = int(core_stats.accepted)
        measured_rate = (
            (accepted - self._last_diagnostic_accepted)
            * 1_000_000_000.0
            / elapsed_ns
        )
        producer_ns = int(
            self._status_scalar(
                status_fields, "producer_monotonic_ns", default=0
            )
        )
        producer_age_ms = (
            0.0 if producer_ns <= 0 else max(0, now_ns - producer_ns) / 1e6
        )
        status_age_ms = (
            0.0
            if self._last_successful_status_ns is None
            else max(0, now_ns - self._last_successful_status_ns) / 1e6
        )
        malformed = int(receiver_stats.invalid_size) + self._malformed_packets
        sender = self._core.locked_sender
        message = (
            "Xsens source summary: "
            f"sender={sender} rate_hz={measured_rate:.2f} "
            f"producer_age_ms={producer_age_ms:.1f} "
            f"status_age_ms={status_age_ms:.1f} "
            f"ready={state[1]} window_ready={state[2]} stale={state[3]} "
            f"recovery_frames={state[5]} reason={state[4]} "
            f"received={int(receiver_stats.received)} "
            f"accepted={accepted} malformed={malformed} "
            f"duplicate={int(core_stats.duplicates)} "
            f"ambiguous={int(core_stats.ambiguous)} "
            f"backward={int(core_stats.backwards)} "
            f"inferred_drop={int(core_stats.inferred_missing_frames)} "
            f"zmq_drop={int(self._publisher.drop_count)}"
        )
        if state[3]:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)
        if summary_due:
            self._last_diagnostic_summary_ns = now_ns
            self._last_diagnostic_accepted = accepted

    def _tick_locked(self, now_ns: int) -> None:
        try:
            self._core.expire(now_ns)
        except Exception as error:
            self.get_logger().error(f"Xsens candidate expiry failed: {error}")
        self._drain_and_accept()
        try:
            became_stale = bool(self._core.check_stale(now_ns))
        except Exception as error:
            became_stale = True
            self.get_logger().error(f"Xsens stale check failed: {error}")
        if became_stale:
            self._pose_pending = False

        status_fields, status_sent = self._publish_status(now_ns)
        if status_sent and self._pose_pending and not became_stale:
            try:
                pose_fields = self._core.build_pose_fields()
                if pose_fields is None:
                    self._pose_pending = False
                else:
                    newest = int(np.asarray(pose_fields["frame_index"])[-1])
                    if newest == self._last_published_frame_index:
                        self._pose_pending = False
                    elif self._publisher.send(self._pose_topic, pose_fields):
                        self._last_published_frame_index = newest
                        self._pose_pending = False
            except Exception as error:
                self.get_logger().error(
                    f"Xsens pose publication failed: {error}"
                )
        self._maybe_log_diagnostics(now_ns, status_fields)

    def _tick(self) -> None:
        with self._lock:
            if self._closed:
                return
            try:
                now_ns = operator.index(self._clock_ns())
                self._tick_locked(now_ns)
            except Exception as error:
                self.get_logger().error(f"Xsens source tick failed: {error}")

    def _close_with_warning(self, name: str, close_resource) -> None:
        try:
            close_resource()
        except Exception as error:
            self.get_logger().warning(f"failed to close {name}: {error}")

    def _cleanup_resources(self) -> None:
        if self._timer is not None:
            timer, self._timer = self._timer, None
            self._close_with_warning(
                "Xsens timer", lambda: self.destroy_timer(timer)
            )
        if self._subscription is not None:
            subscription, self._subscription = self._subscription, None
            self._close_with_warning(
                "Xsens arm subscription",
                lambda: self.destroy_subscription(subscription),
            )
        if self._publisher is not None:
            publisher, self._publisher = self._publisher, None
            self._close_with_warning("Xsens publisher", publisher.close)
        if self._receiver is not None:
            receiver, self._receiver = self._receiver, None
            self._close_with_warning("Xsens UDP receiver", receiver.close)

    def _cleanup_failed_construction(self) -> None:
        self._closed = True
        self._cleanup_resources()
        self._destroy_result = super().destroy_node()

    def destroy_node(self):
        with self._lock:
            if self._closed:
                return self._destroy_result
            self._closed = True
            self._cleanup_resources()
            self._destroy_result = super().destroy_node()
            return self._destroy_result


def create_node(context: NodeBuildContext) -> XsensSourceNode:
    return XsensSourceNode(context)


__all__ = [
    "DIAGNOSTIC_SUMMARY_SECONDS",
    "SOURCE_DEFAULTS",
    "XsensPoseStatusPublisher",
    "XsensSourceNode",
    "create_node",
    "random_epoch_candidate",
    "validate_source_params",
]
