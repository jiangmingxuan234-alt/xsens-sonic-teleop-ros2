from __future__ import annotations

from collections import deque
from dataclasses import replace
import socket
import struct
from threading import Event, Lock
from typing import TYPE_CHECKING, Mapping, Sequence

import numpy as np

from bxi_example_py_elf3.framework.inference import InferenceFrame
from bxi_example_py_elf3.framework.joints import JointStateView
from pico.zmq_messages import pack_pose_message
from policy import (
    MODEL_INPUT_DIM,
    SMPL_ROOT_ORI_START,
    SMPL_TOKENIZER_DIM,
    SONIC_PARAMETERS,
    SmplReferenceFrame,
    SonicTeleopPolicy,
)


if TYPE_CHECKING:
    from xsens.converter import XsensMotionConverter
    from xsens.source_core import XsensSourceCore


SOURCE_PERIOD_NS = 1_000_000_000 // 60


class FakeClock:
    def __init__(self, now_ns: int = 0) -> None:
        self.now_ns = int(now_ns)

    def monotonic_ns(self) -> int:
        return self.now_ns

    def monotonic(self) -> float:
        return self.now_ns / 1_000_000_000.0

    def advance_ns(self, delta: int) -> None:
        if delta < 0:
            raise ValueError("delta must be non-negative")
        self.now_ns += int(delta)


class FakeDatagramSocket:
    def __init__(self, datagrams=()) -> None:
        self.datagrams = deque(datagrams)
        self.bound = None
        self.blocking = None
        self.closed = False
        self.close_calls = 0
        self.recv_sizes = []

    def bind(self, address) -> None:
        self.bound = address

    def setblocking(self, value: bool) -> None:
        self.blocking = bool(value)

    def recvfrom(self, size: int):
        self.recv_sizes.append(size)
        if not self.datagrams:
            raise BlockingIOError
        payload, sender = self.datagrams.popleft()
        return payload, sender

    def close(self) -> None:
        self.close_calls += 1
        self.closed = True


class FakeZmqSocket:
    def __init__(self) -> None:
        self.options = []
        self.bound = None
        self.messages = []
        self.closed = False
        self.close_calls = 0
        self.bind_error = None
        self.send_error = None

    def setsockopt(self, option, value) -> None:
        self.options.append((option, value))

    def bind(self, endpoint: str) -> None:
        if self.bind_error is not None:
            raise self.bind_error
        self.bound = endpoint

    def send(self, message: bytes, flags=0) -> None:
        if self.send_error is not None:
            error, self.send_error = self.send_error, None
            raise error
        self.messages.append((bytes(message), flags))

    def close(self, linger=None) -> None:
        self.close_calls += 1
        self.closed = True


class FakeZmqContext:
    def __init__(self, socket=None) -> None:
        self.fake_socket = socket or FakeZmqSocket()
        self.socket_types = []
        self.term_calls = 0

    def socket(self, socket_type):
        self.socket_types.append(socket_type)
        return self.fake_socket

    def term(self) -> None:
        self.term_calls += 1


def _reserve_port(sock_type: int) -> int:
    with socket.socket(socket.AF_INET, sock_type) as reservation:
        reservation.bind(("127.0.0.1", 0))
        return int(reservation.getsockname()[1])


def reserve_udp_port() -> int:
    return _reserve_port(socket.SOCK_DGRAM)


def reserve_tcp_port() -> int:
    return _reserve_port(socket.SOCK_STREAM)


def build_mxtp02_packet(
    *,
    sample_counter: int = 1,
    time_code: int = 100,
    identifier: bytes = b"MXTP02",
    datagram_counter: int = 0,
    item_count: int = 23,
    character_id: int = 0,
    body_segment_count: int = 23,
    prop_count: int = 0,
    finger_segment_count: int = 0,
    reserved: int = 0,
    payload_size: int = 736,
    segment_order: tuple[int, ...] = tuple(range(1, 24)),
    positions: np.ndarray | None = None,
    quaternions_wxyz: np.ndarray | None = None,
) -> bytes:
    pos = (
        np.zeros((23, 3), dtype=np.float32)
        if positions is None
        else np.asarray(positions, dtype=np.float32)
    )
    quat = (
        np.tile(np.array([1, 0, 0, 0], dtype=np.float32), (23, 1))
        if quaternions_wxyz is None
        else np.asarray(quaternions_wxyz, dtype=np.float32)
    )
    header = struct.pack(
        ">6sIBBIBBBBHH",
        identifier,
        sample_counter,
        datagram_counter,
        item_count,
        time_code,
        character_id,
        body_segment_count,
        prop_count,
        finger_segment_count,
        reserved,
        payload_size,
    )
    rows = []
    for row_index, segment_id in enumerate(segment_order):
        source_index = (
            segment_id - 1 if 1 <= segment_id <= 23 else row_index % 23
        )
        rows.append(
            struct.pack(
                ">I7f", segment_id, *pos[source_index], *quat[source_index]
            )
        )
    return header + b"".join(rows)


def make_packet(
    sample_counter=1,
    time_code=100,
    sender=("127.0.0.1", 4000),
    receive_timestamp_ns=0,
    positions=None,
    quaternions_wxyz=None,
):
    from xsens.protocol import parse_mxtp02_packet

    payload = build_mxtp02_packet(
        sample_counter=sample_counter,
        time_code=time_code,
        positions=positions,
        quaternions_wxyz=quaternions_wxyz,
    )
    return parse_mxtp02_packet(
        payload,
        receive_timestamp_ns=receive_timestamp_ns,
        sender_address=sender,
    )


def make_core(
    *,
    epoch_draws: tuple[int, ...] = (91, 92, 93),
    converter: XsensMotionConverter | None = None,
    now_ns: int = 0,
    window_frames: int = 10,
    same_epoch_resume_frames: int = 10,
    ready_frames: int = 30,
    stale_seconds: float = 0.5,
    epoch_candidate_frames: int = 2,
    epoch_candidate_timeout_s: float = 0.25,
    max_pelvis_span_m: float = 0.15,
    max_segment_deviation_deg: float = 20.0,
) -> tuple[XsensSourceCore, FakeClock]:
    from xsens.converter import XsensMotionConverter
    from xsens.source_core import XsensSourceCore

    clock = FakeClock(now_ns)
    draws = iter(epoch_draws)
    core = XsensSourceCore(
        converter or XsensMotionConverter(),
        clock_ns=clock.monotonic_ns,
        epoch_factory=lambda: next(draws),
        window_frames=window_frames,
        same_epoch_resume_frames=same_epoch_resume_frames,
        ready_frames=ready_frames,
        stale_seconds=stale_seconds,
        epoch_candidate_frames=epoch_candidate_frames,
        epoch_candidate_timeout_s=epoch_candidate_timeout_s,
        max_pelvis_span_m=max_pelvis_span_m,
        max_segment_deviation_deg=max_segment_deviation_deg,
    )
    return core, clock


def accept_packet(
    core,
    counter,
    *,
    timestamp_ns,
    time_code=0,
    sender=("127.0.0.1", 4000),
    character_id=0,
):
    packet = make_packet(
        sample_counter=counter,
        time_code=time_code,
        sender=sender,
        receive_timestamp_ns=timestamp_ns,
    )
    packet = replace(
        packet,
        header=replace(packet.header, character_id=character_id),
    )
    return core.accept(packet)


def feed_frame_sequence(
    core, clock, positions, quaternions, *, counter_start=None
):
    count = len(positions)
    assert len(quaternions) == count
    if counter_start is None:
        counter_start = (
            1
            if core.newest_frame_index < 0
            else (core.newest_frame_index + 1) & 0xFFFFFFFF
        )
    results = []
    for offset, (position, quat) in enumerate(
        zip(positions, quaternions)
    ):
        clock.advance_ns(SOURCE_PERIOD_NS)
        results.append(
            core.accept(
                make_packet(
                    sample_counter=(counter_start + offset) & 0xFFFFFFFF,
                    time_code=0,
                    receive_timestamp_ns=clock.now_ns,
                    positions=position,
                    quaternions_wxyz=quat,
                )
            )
        )
    return results


def feed_stable_frames(core, clock, count, epoch_counter_start=None):
    positions = np.zeros((count, 23, 3), dtype=np.float32)
    quats = np.tile(
        np.array([1, 0, 0, 0], dtype=np.float32), (count, 23, 1)
    )
    return feed_frame_sequence(
        core,
        clock,
        positions,
        quats,
        counter_start=epoch_counter_start,
    )


def ready_core(epoch):
    core, clock = make_core(epoch_draws=(epoch, epoch + 1))
    feed_stable_frames(core, clock, 30)
    assert core.source_epoch == epoch and core.ready
    return core, clock


def make_xsens_pose_fields(
    *,
    indices=None,
    source_epoch=71,
    calibration_ready=True,
    producer_monotonic_ns=1_000_000_000,
):
    frame_index = (
        np.arange(100, 110, dtype=np.int64)
        if indices is None
        else np.asarray(indices, dtype=np.int64)
    )
    rows = int(frame_index.size)
    roots = np.zeros((rows, 4), dtype=np.float32)
    roots[:, 0] = 1.0
    return {
        "frame_index": frame_index,
        "smpl_joints": np.zeros((rows, 24, 3), dtype=np.float32),
        "body_quat_w": roots,
        "joint_pos": np.zeros((rows, 29), dtype=np.float32),
        "stream_mode": np.array([1], dtype=np.int32),
        "calibration_ready": np.array(
            [calibration_ready], dtype=np.bool_
        ),
        "producer_monotonic_ns": np.array(
            [producer_monotonic_ns], dtype=np.int64
        ),
        "source_epoch": np.array([source_epoch], dtype=np.int64),
    }


def make_status(
    *,
    status_sequence=1,
    status_monotonic_ns=1_000_000_000,
    source_epoch=71,
    last_arm_command_id=0,
    last_arm_target_epoch=0,
    last_requested_arm_epoch=0,
    accepted_arm_epoch=0,
    producer_monotonic_ns=1_000_000_000,
    newest_frame_index=109,
    ready=True,
    reference_window_ready=True,
    source_stale=False,
    ready_frames=30,
    recovery_frames=0,
    reason_code=None,
):
    from xsens.source_core import XsensReason

    if reason_code is None:
        reason_code = XsensReason.READY
    return {
        "status_sequence": np.array(
            [status_sequence], dtype=np.int64
        ),
        "status_monotonic_ns": np.array(
            [status_monotonic_ns], dtype=np.int64
        ),
        "source_epoch": np.array([source_epoch], dtype=np.int64),
        "last_arm_command_id": np.array(
            [last_arm_command_id], dtype=np.int64
        ),
        "last_arm_target_epoch": np.array(
            [last_arm_target_epoch], dtype=np.int64
        ),
        "last_requested_arm_epoch": np.array(
            [last_requested_arm_epoch], dtype=np.int64
        ),
        "accepted_arm_epoch": np.array(
            [accepted_arm_epoch], dtype=np.int64
        ),
        "producer_monotonic_ns": np.array(
            [producer_monotonic_ns], dtype=np.int64
        ),
        "newest_frame_index": np.array(
            [newest_frame_index], dtype=np.int64
        ),
        "ready": np.array([ready], dtype=np.bool_),
        "reference_window_ready": np.array(
            [reference_window_ready], dtype=np.bool_
        ),
        "source_stale": np.array([source_stale], dtype=np.bool_),
        "ready_frames": np.array([ready_frames], dtype=np.int32),
        "recovery_frames": np.array(
            [recovery_frames], dtype=np.int32
        ),
        "reason_code": np.array([int(reason_code)], dtype=np.int32),
    }


def axis_angle_wxyz(axis: int, degrees: float) -> np.ndarray:
    value = np.zeros(4, dtype=np.float32)
    value[0] = np.cos(np.deg2rad(degrees) / 2.0)
    value[axis + 1] = np.sin(np.deg2rad(degrees) / 2.0)
    return value


def make_reference(
    *,
    epoch: int | None = 71,
    newest_frame: int | None = 109,
    source_ready: bool = True,
    producer_monotonic_ns: int | None = 1_000_000_000,
    row_marker: float | Sequence[float] = 0.0,
    root_yaw_rad: float = 0.0,
    with_anchor: bool = True,
) -> SmplReferenceFrame:
    marker = np.asarray(row_marker, dtype=np.float32)
    if marker.ndim == 0:
        marker = np.repeat(marker.reshape(1), 10)
    marker = marker.reshape(10)
    term = np.repeat(marker[:, None], 72, axis=1).astype(np.float32)
    wrist = np.repeat(marker[:, None], 6, axis=1).astype(np.float32)
    root = np.zeros((10, 4), dtype=np.float32)
    root[:, 0] = np.cos(root_yaw_rad / 2.0)
    root[:, 3] = np.sin(root_yaw_rad / 2.0)
    anchor = root.copy() if with_anchor else None
    return SmplReferenceFrame(
        term1_local=term,
        root_quat=root,
        wrist=wrist,
        anchor_quat=anchor,
        frame_index=-1 if newest_frame is None else int(newest_frame) - 9,
        sequence=1,
        source_ready=bool(source_ready),
        producer_monotonic_ns=producer_monotonic_ns,
        source_epoch=epoch,
        source_newest_frame_index=newest_frame,
        received_monotonic=0.0,
    )


def make_inference_frame(qpos_offset: float = 0.0) -> InferenceFrame:
    position = (
        SONIC_PARAMETERS.default_position.astype(np.float32)
        + np.float32(qpos_offset)
    )
    joints = JointStateView(
        SONIC_PARAMETERS.layout,
        position.copy(),
        np.zeros(29, dtype=np.float32),
    )
    return InferenceFrame(
        joints=joints,
        quat_wxyz=np.array([1, 0, 0, 0], dtype=np.float32),
        angular_velocity=np.zeros(3, dtype=np.float32),
    )


class DeterministicBackend:
    def run(self, inputs: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
        observation = np.asarray(inputs["obs_dict"], dtype=np.float32)
        human = float(np.mean(observation[0, :720]))
        root = float(observation[0, SMPL_ROOT_ORI_START + 1])
        proprio = float(np.mean(observation[0, SMPL_TOKENIZER_DIM:]))
        signal = 0.01 * human + 0.25 * root + 0.01 * proprio
        return {
            "action": np.full(
                (1, 29),
                np.tanh(signal) * np.float32(0.1),
                dtype=np.float32,
            )
        }

    def close(self) -> None:
        pass


class TestableSonicPolicy(SonicTeleopPolicy):
    __test__ = False

    @property
    def pending_yaw_offset(self) -> float | None:
        return self._pending_yaw_offset

    @pending_yaw_offset.setter
    def pending_yaw_offset(self, value: float | None) -> None:
        self._pending_yaw_offset = value

    def _load_stream_reference(self) -> None:
        rows = 3520
        self.ref_term1 = np.zeros((rows, 72), np.float32)
        self.ref_root_quat = np.zeros((rows, 4), np.float32)
        self.ref_root_quat[:, 0] = 1.0
        self.ref_wrist = np.zeros((rows, 6), np.float32)
        self.ref_anchor_quat = None

    def _init_backend(self, backend: str) -> None:
        self._backend = DeterministicBackend()
        self.input_buffer = np.zeros((1, MODEL_INPUT_DIM), np.float32)
        self._inputs = {"obs_dict": self.input_buffer}

    def _init_zmq(self) -> None:
        self._message_lock = Lock()
        self._reference_messages = deque(maxlen=64)
        self._status_messages = deque(maxlen=64)
        self._zmq_stop = Event()
        self._zmq_thread = None

    def inject_status(
        self,
        fields: Mapping[str, np.ndarray],
        received_mono: float | None = None,
    ) -> None:
        received = (
            self._monotonic() if received_mono is None else received_mono
        )
        message = pack_pose_message(fields, topic=self.xsens_status_zmq_topic)
        with self._message_lock:
            self._status_messages.append((message, float(received)))

    def inject_reference(
        self,
        reference: SmplReferenceFrame,
        received_mono: float | None = None,
    ) -> None:
        received = (
            self._monotonic() if received_mono is None else received_mono
        )
        fields = {
            "term1_local": reference.term1_local,
            "root_quat": reference.root_quat,
            "wrist": reference.wrist,
            "frame_index": np.array([reference.frame_index], np.int64),
            "source_ready": np.array([reference.source_ready], np.bool_),
        }
        if reference.producer_monotonic_ns is not None:
            fields["producer_monotonic_ns"] = np.array(
                [reference.producer_monotonic_ns], np.int64
            )
        if reference.source_epoch is not None:
            fields["source_epoch"] = np.array(
                [reference.source_epoch], np.int64
            )
        if reference.source_newest_frame_index is not None:
            fields["source_newest_frame_index"] = np.array(
                [reference.source_newest_frame_index], np.int64
            )
        if reference.anchor_quat is not None:
            fields["anchor_quat"] = reference.anchor_quat
        message = pack_pose_message(fields, topic=self.smpl_ref_zmq_topic)
        with self._message_lock:
            self._reference_messages.append((message, float(received)))


class CaptureLogger:
    def __init__(self) -> None:
        self.infos = []
        self.warnings = []
        self.events = []

    def info(self, message) -> None:
        self.infos.append(str(message))
        self.events.append(("info", str(message)))

    def warning(self, message) -> None:
        self.warnings.append(str(message))
        self.events.append(("warning", str(message)))


class PolicyHarness:
    def __init__(self) -> None:
        self.clock = FakeClock(1_000_000_000)
        self.policy = TestableSonicPolicy(
            "unused.onnx",
            "unused.npz",
            use_smpl_ref_zmq=True,
            monotonic=self.clock.monotonic,
            monotonic_ns=self.clock.monotonic_ns,
        )
        self.policy.test_clock = self.clock
        self.policy.bind_logger(CaptureLogger())
        self.policy.configure_runtime(
            yaw_bias_rad=np.pi / 2,
            live_ref_timeout_s=0.5,
            idle_frame_start=3509,
            source_blend_duration_s=0.4,
            source_kind="xsens",
            status_timeout_s=0.2,
        )
        self.policy.reset(make_inference_frame())

    def close(self) -> None:
        self.policy.close()
