"""Bridge official PICO manager ``pose`` stream to ELF3 ``smpl_ref`` stream.

The official ``gear_sonic/scripts/pico_manager_thread_server.py --manager`` sends
packed ZMQ messages on topic ``pose``.  For the ELF3 native _smpl.onnx deploy we
publish the same long-lived reference contract used by ``smpl_ref_bridge.py``:

    term1_local : float32 [10,72]   SMPL joints local, flattened per frame
    root_quat   : float32 [10,4]    SMPL root quaternion, wxyz
    wrist       : float32 [10,6]    ELF3 native wrist x/y/z, left then right

The bridge is intentionally only an adapter: PICO/SMPL normalization stays in
the official PICO manager, while the downstream SONIC policy consumes the
normalized reference tensors.  Live PICO chunks are merged with the same
sliding-window semantics as the official C++ StreamedMotionMerger, so the
published 10-frame smpl_ref window is a true future window instead of a tiled
latest frame.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
import json
import math
import sys
import time
from typing import Any, Protocol

import numpy as np
import zmq

from rclpy.node import Node
from std_msgs.msg import Float32

from bxi_example_py_elf3.framework.mod_api import NodeBuildContext
from xsens.source_core import XsensReason

from .zmq_messages import pack_pose_message
from .runtime_config import (
    PICO_HOST,
    PICO_PORT,
    PICO_STALE_SECONDS,
    PICO_TOPIC,
    SMPL_REF_HOST,
    SMPL_REF_PORT,
    SMPL_REF_TOPIC,
)


HEADER_SIZE = 1280
DTYPE_MAP = {
    "f32": np.dtype("<f4"),
    "f64": np.dtype("<f8"),
    "i32": np.dtype("<i4"),
    "i64": np.dtype("<i8"),
    "u8": np.dtype("u1"),
    "bool": np.dtype("?"),
}

# The packaged PICO manager writes directly to the ELF3 29-DoF joint order.
# Export the six wrist joints as:
#   l_wrist_x, l_wrist_y, l_wrist_z, r_wrist_x, r_wrist_y, r_wrist_z.
ELF3_NATIVE_WRIST_IDX = [19, 20, 21, 26, 27, 28]
WINDOW = 10
HISTORY_FRAMES = 5
MAX_GAP_FRAMES = 200
DEFAULT_RATE_HZ = 50.0
POSE_STREAM_MODE = 1
READY_CONSECUTIVE_MESSAGES = 3
MAX_PRODUCER_AGE_NS = 500_000_000


PICO_BUTTON_FIELDS = (
    "left_trigger",
    "right_trigger",
    "left_grip",
    "right_grip",
)

BRIDGE_DEFAULTS: dict[str, object] = {
    "pico_host": PICO_HOST,
    "pico_port": PICO_PORT,
    "out_host": SMPL_REF_HOST,
    "out_port": SMPL_REF_PORT,
    "source_kind": "legacy",
    "input_pose_topic": PICO_TOPIC,
    "input_status_topic": "xsens_status",
    "output_reference_topic": SMPL_REF_TOPIC,
    "output_status_topic": "xsens_status",
    "authoritative_input_window": False,
    "readiness_debounce_messages": 3,
    "rate_hz": DEFAULT_RATE_HZ,
    "history_frames": HISTORY_FRAMES,
    "max_gap_frames": MAX_GAP_FRAMES,
    "catch_up_enabled": True,
    "stale_warning_seconds": PICO_STALE_SECONDS,
}
BRIDGE_ALIASES = {
    "pico_topic": "input_pose_topic",
    "out_topic": "output_reference_topic",
}


def _field_scalar(fields: dict[str, np.ndarray], name: str) -> float | None:
    value = fields.get(name)
    if value is None:
        return None
    try:
        arr = np.asarray(value, dtype=np.float32).reshape(-1)
    except (TypeError, ValueError):
        return None
    if arr.size == 0:
        return None
    scalar = float(arr[0])
    return scalar if np.isfinite(scalar) else None


@dataclass
class IncomingChunk:
    frame_indices: np.ndarray
    term1_local: np.ndarray
    root_quat: np.ndarray
    wrist: np.ndarray


@dataclass
class MergeResult:
    did_catchup_reset: bool = False
    frame_offset_adjustment: int = 0
    frame_step: int = 1


class PicoSourceReadinessGate:
    """Require calibrated POSE metadata and progressing, finite raw frames."""

    def __init__(self, required_consecutive: int = READY_CONSECUTIVE_MESSAGES):
        self.required_consecutive = max(1, int(required_consecutive))
        self.reset()

    def reset(self) -> None:
        self.streak = 0
        self.last_frame_index: int | None = None
        self.last_message_mono: float | None = None
        self.last_ready_mono: float | None = None

    def observe(
        self,
        fields: dict[str, np.ndarray],
        now_mono: float,
        stale_seconds: float,
    ) -> bool:
        if (
            self.last_message_mono is not None
            and now_mono - self.last_message_mono > stale_seconds
        ):
            self.reset()
        self.last_message_mono = now_mono

        frame_index: int | None = None
        try:
            mode = int(np.asarray(fields["stream_mode"]).reshape(-1)[-1])
            calibrated = bool(np.asarray(fields["calibration_ready"]).reshape(-1)[-1])
            frame_index = int(np.asarray(fields["frame_index"]).reshape(-1)[-1])
            finite = all(
                np.asarray(fields[name]).size > 0
                and np.isfinite(np.asarray(fields[name])).all()
                for name in ("smpl_joints", "body_quat_w", "joint_pos")
            )
        except (KeyError, TypeError, ValueError, IndexError):
            mode, calibrated, finite = -1, False, False

        source_valid = mode == POSE_STREAM_MODE and calibrated and finite
        if (
            source_valid
            and frame_index is not None
            and self.last_frame_index is not None
            and frame_index < self.last_frame_index
        ):
            # PICO restarts its frame counter when a new POSE session starts.
            # Treat the first lower-index packet as frame one of that session;
            # otherwise the gate would wait for the counter to overtake the
            # previous session before live references could become ready again.
            self.streak = 0
            self.last_frame_index = None
            self.last_ready_mono = None

        progressing = frame_index is not None and (
            self.last_frame_index is None or frame_index > self.last_frame_index
        )
        if frame_index is not None and (
            self.last_frame_index is None or frame_index > self.last_frame_index
        ):
            self.last_frame_index = frame_index
        if source_valid and progressing:
            self.streak += 1
        else:
            self.streak = 0
            self.last_ready_mono = None
        ready = self.streak >= self.required_consecutive
        if ready:
            self.last_ready_mono = now_mono
        return ready

    def is_fresh(self, now_mono: float, stale_seconds: float) -> bool:
        return (
            self.streak >= self.required_consecutive
            and self.last_message_mono is not None
            and self.last_ready_mono is not None
            and now_mono - self.last_ready_mono <= stale_seconds
        )


def _decode_packed_message(msg: bytes, topic: str) -> dict[str, np.ndarray] | None:
    prefix = topic.encode("utf-8")
    if not msg.startswith(prefix):
        return None
    payload = msg[len(prefix) :]
    if len(payload) < HEADER_SIZE:
        return None

    raw_header = payload[:HEADER_SIZE].split(b"\x00", 1)[0]
    if not raw_header:
        return None
    header: dict[str, Any] = json.loads(raw_header.decode("utf-8"))
    data = memoryview(payload[HEADER_SIZE:])

    out: dict[str, np.ndarray] = {}
    offset = 0
    for field in header.get("fields", []):
        name = field["name"]
        dtype = DTYPE_MAP.get(field["dtype"])
        if dtype is None:
            raise ValueError(f"unsupported dtype for field {name}: {field['dtype']}")
        shape = tuple(int(x) for x in field.get("shape", []))
        count = int(np.prod(shape)) if shape else 1
        nbytes = dtype.itemsize * count
        if offset + nbytes > len(data):
            raise ValueError(f"field {name} exceeds payload bounds")
        arr = np.frombuffer(data[offset : offset + nbytes], dtype=dtype, count=count)
        out[name] = arr.reshape(shape).copy()
        offset += nbytes
    return out


class PackedMessageInput(Protocol):
    def drain(self) -> tuple[bytes, ...]:
        ...

    def close(self) -> None:
        ...


class PackedTopicOutput(Protocol):
    def send(self, topic: str, fields: Mapping[str, np.ndarray]) -> bool:
        ...

    def close(self) -> None:
        ...


class ZmqPackedMessageInput:
    """One ordered packed-topic input connection owned by the bridge."""

    def __init__(self, host: str, port: int, topics) -> None:
        self._closed = False
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        try:
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.setsockopt(zmq.RCVHWM, 64)
            for topic in topics:
                self._socket.setsockopt_string(zmq.SUBSCRIBE, topic)
            self._socket.connect(f"tcp://{host}:{port}")
        except Exception:
            self._socket.close(linger=0)
            self._context.term()
            self._closed = True
            raise

    def drain(self) -> tuple[bytes, ...]:
        messages = []
        while True:
            try:
                messages.append(self._socket.recv(flags=zmq.NOBLOCK))
            except zmq.Again:
                return tuple(messages)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._socket.close(linger=0)
        self._context.term()


class ZmqPackedTopicOutput:
    """One packed-topic output connection shared by status and reference."""

    def __init__(self, host: str, port: int) -> None:
        self._closed = False
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        try:
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.setsockopt(zmq.SNDHWM, 64)
            self._socket.bind(f"tcp://{host}:{port}")
        except Exception:
            self._socket.close(linger=0)
            self._context.term()
            self._closed = True
            raise

    def send(self, topic: str, fields: Mapping[str, np.ndarray]) -> bool:
        message = pack_pose_message(fields, topic=topic, version=4)
        try:
            self._socket.send(message, flags=zmq.NOBLOCK)
        except zmq.Again:
            return False
        return True

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._socket.close(linger=0)
        self._context.term()


_XSENS_POSE_SCHEMA = {
    "frame_index": (np.dtype(np.int64), (10,)),
    "smpl_joints": (np.dtype(np.float32), (10, 24, 3)),
    "body_quat_w": (np.dtype(np.float32), (10, 4)),
    "joint_pos": (np.dtype(np.float32), (10, 29)),
    "stream_mode": (np.dtype(np.int32), (1,)),
    "calibration_ready": (np.dtype(np.bool_), (1,)),
    "producer_monotonic_ns": (np.dtype(np.int64), (1,)),
    "source_epoch": (np.dtype(np.int64), (1,)),
}
_XSENS_STATUS_SCHEMA = {
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


def _validate_array_schema(
    fields: Mapping[str, np.ndarray],
    schema: Mapping[str, tuple[np.dtype, tuple[int, ...]]],
) -> None:
    missing = sorted(set(schema) - set(fields))
    if missing:
        raise ValueError(f"missing field {missing[0]}")
    unexpected = sorted(set(fields) - set(schema))
    if unexpected:
        raise ValueError(f"unexpected field {unexpected[0]}")
    for name, (dtype, shape) in schema.items():
        value = fields[name]
        if not isinstance(value, np.ndarray):
            raise ValueError(f"{name} must be an np.ndarray")
        if value.dtype != dtype:
            raise ValueError(f"{name} has dtype {value.dtype}; expected {dtype}")
        if value.shape != shape:
            raise ValueError(f"{name} has shape {value.shape}; expected {shape}")


def _build_authoritative_xsens_smpl_ref(
    fields: Mapping[str, np.ndarray],
) -> dict[str, np.ndarray]:
    _validate_array_schema(fields, _XSENS_POSE_SCHEMA)
    for name in ("smpl_joints", "body_quat_w", "joint_pos"):
        if not np.isfinite(fields[name]).all():
            raise ValueError(f"{name} must contain only finite values")
    if np.any(np.linalg.norm(fields["body_quat_w"], axis=1) <= 0):
        raise ValueError("body_quat_w must contain nonzero root quaternions")
    if int(fields["stream_mode"][0]) != POSE_STREAM_MODE:
        raise ValueError("stream_mode must be 1")
    if int(fields["producer_monotonic_ns"][0]) <= 0:
        raise ValueError("producer_monotonic_ns must be positive")
    if int(fields["source_epoch"][0]) <= 0:
        raise ValueError("source_epoch must be positive")
    if np.any(fields["frame_index"][1:] <= fields["frame_index"][:-1]):
        raise ValueError("frame_index must be strictly increasing")

    return {
        "term1_local": np.ascontiguousarray(
            fields["smpl_joints"].reshape(10, 72), dtype=np.float32
        ),
        "root_quat": np.ascontiguousarray(
            fields["body_quat_w"], dtype=np.float32
        ),
        "wrist": np.ascontiguousarray(
            fields["joint_pos"][:, ELF3_NATIVE_WRIST_IDX], dtype=np.float32
        ),
        "frame_index": np.array([fields["frame_index"][0]], dtype=np.int64),
        "source_ready": np.array(
            [fields["calibration_ready"][0]], dtype=np.bool_
        ),
        "producer_monotonic_ns": np.array(
            fields["producer_monotonic_ns"], copy=True
        ),
        "source_epoch": np.array(fields["source_epoch"], copy=True),
        "source_newest_frame_index": np.array(
            [fields["frame_index"][9]], dtype=np.int64
        ),
    }


def _validate_xsens_status_fields(
    fields: Mapping[str, np.ndarray],
) -> dict[str, np.ndarray]:
    _validate_array_schema(fields, _XSENS_STATUS_SCHEMA)
    values = {name: value[0].item() for name, value in fields.items()}

    if values["status_sequence"] < 1:
        raise ValueError("status_sequence must be positive")
    if values["status_monotonic_ns"] < 0:
        raise ValueError("status_monotonic_ns must be nonnegative")
    for name in (
        "source_epoch",
        "last_arm_command_id",
        "last_arm_target_epoch",
        "last_requested_arm_epoch",
        "accepted_arm_epoch",
        "producer_monotonic_ns",
    ):
        if values[name] < 0:
            raise ValueError(f"{name} must be nonnegative")
    if values["newest_frame_index"] < -1:
        raise ValueError("newest_frame_index must be at least -1")

    command_id = values["last_arm_command_id"]
    target_epoch = values["last_arm_target_epoch"]
    requested_epoch = values["last_requested_arm_epoch"]
    if command_id == 0:
        if target_epoch != 0 or requested_epoch != 0:
            raise ValueError("arm target/request requires a command receipt")
    elif target_epoch <= 0 or requested_epoch not in (0, target_epoch):
        raise ValueError("arm receipt target/request relationship is invalid")

    accepted_epoch = values["accepted_arm_epoch"]
    source_epoch = values["source_epoch"]
    if accepted_epoch not in (0, source_epoch):
        raise ValueError("accepted_arm_epoch must equal source_epoch")
    if accepted_epoch > 0 and (source_epoch <= 0 or command_id <= 0):
        raise ValueError("accepted_arm_epoch requires a positive receipt")

    producer_ns = values["producer_monotonic_ns"]
    newest_frame = values["newest_frame_index"]
    no_data = producer_ns == 0 and newest_frame == -1
    if (producer_ns == 0) != (newest_frame == -1):
        raise ValueError("producer_monotonic_ns/newest_frame_index sentinel mismatch")
    if no_data:
        if (
            source_epoch != 0
            or values["ready"]
            or values["reference_window_ready"]
        ):
            raise ValueError("no-data status has invalid epoch or readiness")
    elif producer_ns <= 0 or newest_frame < 0 or source_epoch <= 0:
        raise ValueError("data-present status requires positive source metadata")

    if values["source_stale"] and (
        values["ready"] or values["reference_window_ready"]
    ):
        raise ValueError("source_stale requires readiness to be false")
    if values["ready"] and not values["reference_window_ready"]:
        raise ValueError("ready requires reference_window_ready")
    if (values["ready"] or values["reference_window_ready"]) and no_data:
        raise ValueError("readiness requires data-present status")
    if not 0 <= values["ready_frames"] <= 30:
        raise ValueError("ready_frames must be in 0..30")
    if not 0 <= values["recovery_frames"] <= 10:
        raise ValueError("recovery_frames must be in 0..10")
    valid_reasons = {int(reason) for reason in XsensReason}
    if values["reason_code"] not in valid_reasons:
        raise ValueError("reason_code is not an XsensReason")

    return {name: np.array(value, copy=True) for name, value in fields.items()}


def _as_frame_matrix(arr: np.ndarray, width: int, name: str) -> np.ndarray:
    arr = np.asarray(arr, dtype=np.float32)
    if arr.ndim == 1:
        arr = arr.reshape(1, width)
    elif arr.ndim > 2:
        arr = arr.reshape(arr.shape[0], -1)
    if arr.shape[1] != width:
        raise ValueError(f"{name} has shape {arr.shape}; expected (*,{width})")
    return np.ascontiguousarray(arr, dtype=np.float32)


def _extract_wrist_frames(joint_pos: np.ndarray) -> np.ndarray:
    jp = np.asarray(joint_pos, dtype=np.float32)
    if jp.ndim == 1:
        jp = jp.reshape(1, -1)
    if jp.shape[1] < 29:
        raise ValueError(
            f"joint_pos has shape {jp.shape}; expected at least 29 columns"
        )

    return np.ascontiguousarray(jp[:, ELF3_NATIVE_WRIST_IDX], dtype=np.float32)


def _parse_incoming_chunk(fields: dict[str, np.ndarray]) -> IncomingChunk:
    missing = [
        k
        for k in ("frame_index", "smpl_joints", "body_quat_w", "joint_pos")
        if k not in fields
    ]
    if missing:
        raise ValueError(f"PICO pose message missing required fields: {missing}")

    smpl_joints = np.asarray(fields["smpl_joints"], dtype=np.float32)
    if smpl_joints.ndim == 2 and smpl_joints.shape[1] == 72:
        term1 = _as_frame_matrix(smpl_joints, 72, "smpl_joints")
    elif smpl_joints.ndim == 3 and smpl_joints.shape[1:] == (24, 3):
        term1 = _as_frame_matrix(
            smpl_joints.reshape(smpl_joints.shape[0], 72),
            72,
            "smpl_joints",
        )
    else:
        raise ValueError(
            f"smpl_joints has shape {smpl_joints.shape}; expected (N,24,3)"
        )

    root_quat = _as_frame_matrix(fields["body_quat_w"], 4, "body_quat_w")
    wrist = _extract_wrist_frames(fields["joint_pos"])
    frame_indices = np.asarray(fields["frame_index"], dtype=np.int64).reshape(-1)

    n = term1.shape[0]
    if root_quat.shape[0] != n or wrist.shape[0] != n or frame_indices.shape[0] != n:
        raise ValueError(
            "PICO pose frame count mismatch: "
            f"frame_index={frame_indices.shape[0]} term1={term1.shape[0]} "
            f"root={root_quat.shape[0]} wrist={wrist.shape[0]}"
        )
    if n <= 0:
        raise ValueError("PICO pose message has zero frames")
    if n > 1 and np.any(np.diff(frame_indices) <= 0):
        raise ValueError(
            f"frame_index must be strictly increasing: {frame_indices.tolist()}"
        )

    return IncomingChunk(
        frame_indices=np.ascontiguousarray(frame_indices, dtype=np.int64),
        term1_local=term1,
        root_quat=root_quat,
        wrist=wrist,
    )


class StreamedSmplRefMerger:
    """Python port of C++ StreamedMotionMerger for live PICO SMPL refs."""

    def __init__(
        self,
        history_frames: int = HISTORY_FRAMES,
        max_gap_frames: int = MAX_GAP_FRAMES,
        catch_up_enabled: bool = True,
    ):
        self.history_frames = int(history_frames)
        self.max_gap_frames = int(max_gap_frames)
        self.catch_up_enabled = bool(catch_up_enabled)
        self.reset()

    def reset(self) -> None:
        self.term1_local = np.zeros((0, 72), dtype=np.float32)
        self.root_quat = np.zeros((0, 4), dtype=np.float32)
        self.wrist = np.zeros((0, 6), dtype=np.float32)
        self.stream_window_start = 0
        self.current_frame = 0
        self.frame_step = 1
        self.total_merges = 0
        self.catchup_count = 0

    @property
    def timesteps(self) -> int:
        return int(self.term1_local.shape[0])

    def _calculate_frame_step(self, frame_indices: np.ndarray) -> int:
        if frame_indices.shape[0] < 2:
            return 1
        step = abs(int(frame_indices[1]) - int(frame_indices[0]))
        return step if step > 0 else 1

    def _calculate_sliding_window(
        self,
        incoming_frame_start: int,
        incoming_frame_end: int,
        frame_step: int,
    ) -> tuple[int, int, bool]:
        if self.timesteps <= 0:
            return incoming_frame_start, 0, True

        global_playback_frame = self.stream_window_start + frame_step * max(
            0, self.current_frame - self.history_frames
        )
        max_gap_frames = (
            self.max_gap_frames + self.history_frames
            if self.catch_up_enabled
            else sys.maxsize
        )
        stream_window_end = self.stream_window_start + frame_step * (self.timesteps - 1)

        if incoming_frame_start <= self.stream_window_start:
            return incoming_frame_start, 0, True
        if incoming_frame_end <= stream_window_end:
            return incoming_frame_start, 0, True

        desired_window_start = global_playback_frame
        tentative_window_start = min(desired_window_start, incoming_frame_start)
        delta_to_incoming = incoming_frame_start - tentative_window_start
        tentative_merge_dst = delta_to_incoming // frame_step if frame_step > 0 else 0
        large_gap_from_old = incoming_frame_start > stream_window_end + frame_step

        if tentative_merge_dst > max_gap_frames or large_gap_from_old:
            return incoming_frame_start, 0, True
        return tentative_window_start, tentative_merge_dst, False

    def merge(self, chunk: IncomingChunk) -> MergeResult:
        frame_step = self._calculate_frame_step(chunk.frame_indices)
        incoming_frame_start = int(chunk.frame_indices[0])
        incoming_frame_end = int(chunk.frame_indices[-1])

        new_window_start, merge_dst_frame, did_catchup = self._calculate_sliding_window(
            incoming_frame_start,
            incoming_frame_end,
            frame_step,
        )

        new_len = merge_dst_frame + int(chunk.frame_indices.shape[0])
        new_term1 = np.zeros((new_len, 72), dtype=np.float32)
        new_root = np.zeros((new_len, 4), dtype=np.float32)
        new_wrist = np.zeros((new_len, 6), dtype=np.float32)

        old_window_start = self.stream_window_start
        if merge_dst_frame > 0 and self.timesteps > 0:
            old_window_end = old_window_start + frame_step * self.timesteps
            need_start_global = new_window_start
            need_end_global = incoming_frame_start
            overlap_start_global = max(need_start_global, old_window_start)
            overlap_end_global = min(need_end_global, old_window_end)
            if overlap_start_global < overlap_end_global:
                start_offset_old = overlap_start_global - old_window_start
                start_offset_new = overlap_start_global - new_window_start
                overlap_span = overlap_end_global - overlap_start_global
                copy_src_idx = start_offset_old // frame_step if frame_step > 0 else 0
                copy_dst_idx = start_offset_new // frame_step if frame_step > 0 else 0
                copy_count = overlap_span // frame_step if frame_step > 0 else 0
                if copy_count > 0:
                    new_term1[
                        copy_dst_idx : copy_dst_idx + copy_count
                    ] = self.term1_local[copy_src_idx : copy_src_idx + copy_count]
                    new_root[copy_dst_idx : copy_dst_idx + copy_count] = self.root_quat[
                        copy_src_idx : copy_src_idx + copy_count
                    ]
                    new_wrist[copy_dst_idx : copy_dst_idx + copy_count] = self.wrist[
                        copy_src_idx : copy_src_idx + copy_count
                    ]

        n_in = int(chunk.frame_indices.shape[0])
        new_term1[merge_dst_frame : merge_dst_frame + n_in] = chunk.term1_local
        new_root[merge_dst_frame : merge_dst_frame + n_in] = chunk.root_quat
        new_wrist[merge_dst_frame : merge_dst_frame + n_in] = chunk.wrist

        window_shift_ticks = new_window_start - old_window_start
        window_shift = window_shift_ticks // frame_step if frame_step > 0 else 0

        self.term1_local = new_term1
        self.root_quat = new_root
        self.wrist = new_wrist
        self.stream_window_start = new_window_start
        self.frame_step = frame_step
        self.total_merges += 1

        if did_catchup:
            self.current_frame = 0
            self.catchup_count += 1
            frame_offset_adjustment = 0
        else:
            self.current_frame = max(0, self.current_frame - window_shift)
            frame_offset_adjustment = window_shift

        return MergeResult(
            did_catchup_reset=did_catchup,
            frame_offset_adjustment=frame_offset_adjustment,
            frame_step=frame_step,
        )

    def advance(self, playing: bool = True) -> None:
        if self.timesteps <= 0:
            return
        if playing and self.current_frame + 1 < self.timesteps:
            self.current_frame += 1
        self.current_frame = int(np.clip(self.current_frame, 0, self.timesteps - 1))

    def build_smpl_ref(self) -> dict[str, np.ndarray] | None:
        if self.timesteps <= 0:
            return None
        self.advance(playing=True)
        idx = np.minimum(
            self.current_frame + np.arange(WINDOW, dtype=np.int64),
            self.timesteps - 1,
        )
        current_global_frame = (
            self.stream_window_start + self.current_frame * self.frame_step
        )
        return {
            "term1_local": np.ascontiguousarray(
                self.term1_local[idx], dtype=np.float32
            ),
            "root_quat": np.ascontiguousarray(self.root_quat[idx], dtype=np.float32),
            "wrist": np.ascontiguousarray(self.wrist[idx], dtype=np.float32),
            "frame_index": np.asarray([current_global_frame], dtype=np.int64),
        }


def _build_live_smpl_ref_if_ready(
    source_gate: PicoSourceReadinessGate,
    merger: StreamedSmplRefMerger,
    now_mono: float,
    stale_seconds: float,
) -> dict[str, np.ndarray] | None:
    """Build one live output only while the calibrated POSE source is fresh."""
    if not source_gate.is_fresh(now_mono, stale_seconds):
        if merger.timesteps:
            merger.reset()
        return None

    smpl_ref = merger.build_smpl_ref()
    if smpl_ref is None:
        return None
    smpl_ref["source_ready"] = np.array([True], dtype=bool)
    smpl_ref["source_stream_mode"] = np.array([POSE_STREAM_MODE], dtype=np.int32)
    smpl_ref["source_calibration_ready"] = np.array([True], dtype=bool)
    return smpl_ref


def _validated_bridge_params(
    raw: Mapping[str, object],
) -> dict[str, object]:
    normalized = dict(raw)
    allowed = set(BRIDGE_DEFAULTS) | set(BRIDGE_ALIASES)
    unknown = set(normalized) - allowed
    if unknown:
        raise ValueError(f"unknown bridge params: {sorted(unknown)}")

    for alias, canonical in BRIDGE_ALIASES.items():
        if alias not in normalized:
            continue
        if canonical in normalized and normalized[alias] != normalized[canonical]:
            raise ValueError(
                f"conflicting bridge params: {alias} and {canonical}"
            )
        normalized.setdefault(canonical, normalized[alias])
        del normalized[alias]

    params = {
        name: normalized.get(name, default)
        for name, default in BRIDGE_DEFAULTS.items()
    }
    if params["source_kind"] not in ("legacy", "xsens"):
        raise ValueError("source_kind must be exactly 'legacy' or 'xsens'")
    for name in (
        "pico_host",
        "out_host",
        "input_pose_topic",
        "input_status_topic",
        "output_reference_topic",
        "output_status_topic",
    ):
        if not isinstance(params[name], str) or not params[name]:
            raise ValueError(f"{name} must be a non-empty string")
    for name in ("pico_port", "out_port"):
        value = params[name]
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 1 <= value <= 65535
        ):
            raise ValueError(f"{name} must be an integer from 1 to 65535")
    for name in ("history_frames", "max_gap_frames"):
        value = params[name]
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{name} must be a non-negative integer")
    for name in ("rate_hz", "stale_warning_seconds"):
        value = params[name]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"{name} must be greater than zero")
    for name in ("catch_up_enabled", "authoritative_input_window"):
        if type(params[name]) is not bool:
            raise ValueError(f"{name} must be a boolean")
    debounce = params["readiness_debounce_messages"]
    if isinstance(debounce, bool) or not isinstance(debounce, int) or debounce < 1:
        raise ValueError("readiness_debounce_messages must be a positive integer")
    if params["source_kind"] == "xsens" and (
        params["authoritative_input_window"] is not True or debounce != 1
    ):
        raise ValueError(
            "xsens requires authoritative_input_window=true and "
            "readiness_debounce_messages=1"
        )
    return params


def _validated_params(raw: Mapping[str, object]) -> dict[str, object]:
    return _validated_bridge_params(raw)


class SmplRefBridgeNode(Node):
    """Non-blocking ROS node owned by the framework's shared executor."""

    def __init__(
        self,
        context: NodeBuildContext,
        *,
        monotonic: Callable[[], float] = time.monotonic,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        input_transport: PackedMessageInput | None = None,
        output_transport: PackedTopicOutput | None = None,
    ):
        params = _validated_bridge_params(dict(context.params))
        super().__init__(context.node_name, namespace=context.namespace or None)

        self._monotonic = monotonic
        self._monotonic_ns = monotonic_ns
        self._source_kind = str(params["source_kind"])
        self._input_pose_topic = str(params["input_pose_topic"])
        self._input_status_topic = str(params["input_status_topic"])
        self._output_reference_topic = str(params["output_reference_topic"])
        self._output_status_topic = str(params["output_status_topic"])
        self._pico_topic = self._input_pose_topic
        self._out_topic = self._output_reference_topic
        self._stale_seconds = float(params["stale_warning_seconds"])
        self._merger = StreamedSmplRefMerger(
            history_frames=int(params["history_frames"]),
            max_gap_frames=int(params["max_gap_frames"]),
            catch_up_enabled=bool(params["catch_up_enabled"]),
        )
        self._source_gate = PicoSourceReadinessGate(
            required_consecutive=int(params["readiness_debounce_messages"])
        )
        self._button_publishers = {}
        self._invalid_button_fields: set[str] = set()
        self._pending_fields: dict[str, np.ndarray] | None = None
        self._pending_xsens_pose: dict[str, np.ndarray] | None = None
        self._pending_status: dict[str, np.ndarray] | None = None
        self._last_forwarded_status_epoch: int | None = None
        self._last_forwarded_status_frame: int = -1
        self._last_forwarded_status_sequence: int = 0
        self._last_xsens_epoch_seen: int | None = None
        self._last_xsens_status_sequence_seen: int = 0
        self._last_xsens_pose_epoch: int | None = None
        self._last_xsens_pose_newest_seen: int = -1
        self._status_send_failed_this_tick: bool = False
        self._last_received_mono: float | None = None
        self._received = 0
        self._skipped = 0
        self._stale_was_reported = False
        self._stream_state: str | None = None
        self._closed = False
        self._destroy_result = None
        self._timer = None
        self._input_transport: PackedMessageInput | None = None
        self._output_transport: PackedTopicOutput | None = None
        self._owns_input_transport = False
        self._owns_output_transport = False
        self._pico_endpoint = f"tcp://{params['pico_host']}:{params['pico_port']}"
        self._out_endpoint = f"tcp://{params['out_host']}:{params['out_port']}"

        try:
            if input_transport is None:
                topics = (
                    (self._input_pose_topic, self._input_status_topic)
                    if self._source_kind == "xsens"
                    else (self._input_pose_topic,)
                )
                self._input_transport = ZmqPackedMessageInput(
                    str(params["pico_host"]), int(params["pico_port"]), topics
                )
                self._owns_input_transport = True
            else:
                self._input_transport = input_transport

            if output_transport is None:
                self._output_transport = ZmqPackedTopicOutput(
                    str(params["out_host"]), int(params["out_port"])
                )
                self._owns_output_transport = True
            else:
                self._output_transport = output_transport

            for name in PICO_BUTTON_FIELDS:
                self._button_publishers[name] = self.create_publisher(
                    Float32, f"pico/{name}", 10
                )
            self._timer = self.create_timer(1.0 / float(params["rate_hz"]), self._tick)
        except Exception:
            self.destroy_node()
            raise

        self.get_logger().info(
            f"SONIC bridge SUB {self._pico_endpoint} "
            f"topic='{self._input_pose_topic}', "
            f"PUB {self._out_endpoint} topic='{self._output_reference_topic}', "
            f"source_kind='{self._source_kind}', "
            f"rate={float(params['rate_hz']):g}Hz"
        )

    def _publish_buttons(self, fields: dict[str, np.ndarray]) -> None:
        for name, publisher in self._button_publishers.items():
            value = _field_scalar(fields, name)
            if value is None:
                if name in fields and name not in self._invalid_button_fields:
                    self.get_logger().warning(
                        f"invalid PICO {name}; retaining the last valid input"
                    )
                    self._invalid_button_fields.add(name)
                continue
            if name in self._invalid_button_fields:
                self.get_logger().info(f"PICO {name} recovered")
                self._invalid_button_fields.remove(name)
            message = Float32()
            message.data = value
            publisher.publish(message)

    def _invalidate_forwarded_barrier(self) -> None:
        self._last_forwarded_status_epoch = None
        self._last_forwarded_status_frame = -1

    def _record_forwarded_status(
        self, fields: Mapping[str, np.ndarray]
    ) -> None:
        self._last_forwarded_status_sequence = int(fields["status_sequence"][0])
        if bool(fields["source_stale"][0]):
            self._invalidate_forwarded_barrier()
            return
        self._last_forwarded_status_epoch = int(fields["source_epoch"][0])
        self._last_forwarded_status_frame = int(fields["newest_frame_index"][0])

    def _handle_xsens_status(self, fields: dict[str, np.ndarray]) -> None:
        epoch = int(fields["source_epoch"][0])
        sequence = int(fields["status_sequence"][0])
        stale = bool(fields["source_stale"][0])

        if epoch != self._last_xsens_epoch_seen:
            self._last_xsens_epoch_seen = epoch
            self._last_xsens_status_sequence_seen = 0
            if (
                self._pending_status is not None
                and int(self._pending_status["source_epoch"][0]) != epoch
            ):
                self._pending_status = None
            self._invalidate_forwarded_barrier()
            self._last_forwarded_status_sequence = 0
            if (
                self._pending_xsens_pose is not None
                and int(self._pending_xsens_pose["source_epoch"][0]) != epoch
            ):
                self._pending_xsens_pose = None
            if self._last_xsens_pose_epoch != epoch:
                self._last_xsens_pose_epoch = None
                self._last_xsens_pose_newest_seen = -1

        if stale:
            self._pending_xsens_pose = None
            self._invalidate_forwarded_barrier()

        if sequence <= self._last_xsens_status_sequence_seen:
            return
        self._last_xsens_status_sequence_seen = sequence
        self._pending_status = {
            name: np.array(value, copy=True) for name, value in fields.items()
        }
        assert self._output_transport is not None
        if not self._output_transport.send(
            self._output_status_topic, self._pending_status
        ):
            self._status_send_failed_this_tick = True
            return
        forwarded = self._pending_status
        self._pending_status = None
        self._record_forwarded_status(forwarded)

    def _handle_xsens_pose(
        self, fields: dict[str, np.ndarray], now_ns: int
    ) -> None:
        reference = _build_authoritative_xsens_smpl_ref(fields)
        epoch = int(reference["source_epoch"][0])
        newest = int(reference["source_newest_frame_index"][0])
        producer_ns = int(reference["producer_monotonic_ns"][0])
        if self._last_xsens_epoch_seen is not None and (
            epoch != self._last_xsens_epoch_seen
        ):
            return
        if now_ns - producer_ns > MAX_PRODUCER_AGE_NS:
            return
        if epoch != self._last_xsens_pose_epoch:
            self._last_xsens_pose_epoch = epoch
            self._last_xsens_pose_newest_seen = -1
        if newest <= self._last_xsens_pose_newest_seen:
            return
        self._last_xsens_pose_newest_seen = newest
        self._pending_xsens_pose = reference

    def _handle_legacy_pose(
        self, fields: dict[str, np.ndarray], now_mono: float
    ) -> None:
        self._publish_buttons(fields)
        self._pending_fields = (
            fields
            if self._source_gate.observe(
                fields,
                now_mono,
                self._stale_seconds,
            )
            else None
        )
        self._last_received_mono = now_mono
        self._stale_was_reported = False
        self._received += 1

    def _drain_input(self, now_mono: float, now_ns: int) -> None:
        assert self._input_transport is not None
        pose_marker = self._input_pose_topic.encode("utf-8") + b"{"
        status_marker = self._input_status_topic.encode("utf-8") + b"{"
        for message in self._input_transport.drain():
            topic = None
            message_kind = None
            if self._source_kind == "xsens" and message.startswith(status_marker):
                topic = self._input_status_topic
                message_kind = "status"
            elif message.startswith(pose_marker):
                topic = self._input_pose_topic
                message_kind = "pose"
            if topic is None:
                continue
            try:
                fields = _decode_packed_message(message, topic)
                if fields is None:
                    raise ValueError(f"invalid packed {topic} message")
                if self._source_kind == "xsens":
                    if message_kind == "status":
                        self._handle_xsens_status(
                            _validate_xsens_status_fields(fields)
                        )
                    else:
                        self._handle_xsens_pose(fields, now_ns)
                else:
                    self._handle_legacy_pose(fields, now_mono)
            except (KeyError, TypeError, ValueError, IndexError) as exc:
                self._skipped += 1
                self.get_logger().warning(f"skipped malformed bridge packet: {exc}")
                continue

    def _tick_legacy(self, now_mono: float) -> None:
        if self._pending_fields is not None:
            try:
                self._merger.merge(_parse_incoming_chunk(self._pending_fields))
            except (KeyError, TypeError, ValueError, IndexError) as exc:
                self._skipped += 1
                self._source_gate.reset()
                self._merger.reset()
                self.get_logger().warning(f"skipped invalid PICO pose: {exc}")
            self._pending_fields = None

        smpl_ref = _build_live_smpl_ref_if_ready(
            self._source_gate,
            self._merger,
            now_mono,
            self._stale_seconds,
        )
        if smpl_ref is not None:
            assert self._output_transport is not None
            if not self._output_transport.send(
                self._output_reference_topic, smpl_ref
            ):
                self._skipped += 1

        input_age = (
            now_mono - self._last_received_mono
            if self._last_received_mono is not None
            else float("inf")
        )
        if (
            self._last_received_mono is not None
            and input_age > self._stale_seconds
            and not self._stale_was_reported
        ):
            self.get_logger().warning(
                "PICO pose input stale; "
                f"age_ms={input_age * 1000.0:.0f} received={self._received}. "
                "Live smpl_ref publication stopped; policy uses idle reference."
            )
            self._stale_was_reported = True

        current_state = (
            "streaming"
            if smpl_ref is not None
            else "stale" if self._stale_was_reported else "waiting"
        )
        if current_state == self._stream_state:
            return
        self._stream_state = current_state
        if current_state == "waiting":
            self.get_logger().info(
                "waiting for calibrated, fresh PICO pose frames; "
                f"received={self._received} skipped={self._skipped}"
            )
        elif current_state == "streaming" and smpl_ref is not None:
            self.get_logger().info(
                "PICO stream ready; "
                f"frame={int(smpl_ref['frame_index'][0])} "
                f"input_age_ms={input_age * 1000.0:.0f}"
            )

    def _tick(self) -> None:
        if self._closed:
            return
        now_mono = self._monotonic()
        now_ns = self._monotonic_ns()
        self._status_send_failed_this_tick = False
        self._drain_input(now_mono, now_ns)
        if self._status_send_failed_this_tick:
            return

        if self._pending_status is not None:
            pending = self._pending_status
            assert self._output_transport is not None
            if not self._output_transport.send(self._output_status_topic, pending):
                return
            self._pending_status = None
            self._record_forwarded_status(pending)

        if self._source_kind == "legacy":
            self._tick_legacy(now_mono)
            return

        pose = self._pending_xsens_pose
        if pose is None:
            return
        producer_ns = int(pose["producer_monotonic_ns"][0])
        if now_ns - producer_ns > MAX_PRODUCER_AGE_NS:
            self._pending_xsens_pose = None
            return
        if self._pending_status is not None:
            return
        epoch = int(pose["source_epoch"][0])
        newest = int(pose["source_newest_frame_index"][0])
        if (
            self._last_forwarded_status_epoch != epoch
            or self._last_forwarded_status_frame < newest
        ):
            return
        assert self._output_transport is not None
        if self._output_transport.send(self._output_reference_topic, pose):
            self._pending_xsens_pose = None
        else:
            self._skipped += 1

    def destroy_node(self):
        if self._closed:
            return self._destroy_result
        self._closed = True
        if self._timer is not None:
            timer, self._timer = self._timer, None
            try:
                self.destroy_timer(timer)
            except Exception as exc:
                self.get_logger().warning(f"failed to destroy timer: {exc}")
        for name, publisher in reversed(tuple(self._button_publishers.items())):
            try:
                self.destroy_publisher(publisher)
            except Exception as exc:
                self.get_logger().warning(
                    f"failed to destroy pico/{name} publisher: {exc}"
                )
        self._button_publishers.clear()
        if self._owns_output_transport and self._output_transport is not None:
            output, self._output_transport = self._output_transport, None
            try:
                output.close()
            except Exception as exc:
                self.get_logger().warning(f"failed to close output transport: {exc}")
        if self._owns_input_transport and self._input_transport is not None:
            input_transport, self._input_transport = self._input_transport, None
            try:
                input_transport.close()
            except Exception as exc:
                self.get_logger().warning(f"failed to close input transport: {exc}")
        self._destroy_result = super().destroy_node()
        return self._destroy_result


def create_node(context: NodeBuildContext) -> SmplRefBridgeNode:
    return SmplRefBridgeNode(context)


__all__ = [
    "BRIDGE_ALIASES",
    "BRIDGE_DEFAULTS",
    "IncomingChunk",
    "MAX_PRODUCER_AGE_NS",
    "PackedMessageInput",
    "PackedTopicOutput",
    "PicoSourceReadinessGate",
    "SmplRefBridgeNode",
    "StreamedSmplRefMerger",
    "ZmqPackedMessageInput",
    "ZmqPackedTopicOutput",
    "create_node",
]
