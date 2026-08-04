"""Pure ZeroLab pose-window and stale-stream state management."""

from collections import deque
import operator

import numpy as np

from .converter import ConvertedPoseFrame, ZeroLabMotionConverter


_POSE_FIELDS = {
    "smpl_body_pose": (21, 3),
    "smpl_joints": (24, 3),
    "body_quat_w": (4,),
    "joint_pos": (29,),
}


def _as_finite_float32(name, values, expected_shape):
    array = np.asarray(values)
    if (
        array.shape != expected_shape
        or not np.issubdtype(array.dtype, np.number)
        or np.issubdtype(array.dtype, np.complexfloating)
    ):
        raise ValueError(
            f"{name} must use a real numeric dtype with shape {expected_shape}"
        )
    result = np.ascontiguousarray(array, dtype=np.float32)
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must be finite with shape {expected_shape}")
    return result


class PoseChunkWindow:
    """Build a rolling fixed-size SONIC pose chunk from converted frames."""

    def __init__(self, window_frames: int = 10) -> None:
        if isinstance(window_frames, bool) or window_frames < 1:
            raise ValueError("window_frames must be at least 1")
        self._window_frames = int(window_frames)
        self._frames = deque(maxlen=self._window_frames)
        self._last_frame_index = None

    @property
    def ready(self) -> bool:
        return len(self._frames) == self._window_frames

    def clear(self) -> None:
        self._frames.clear()
        self._last_frame_index = None

    def append(self, frame: ConvertedPoseFrame):
        try:
            frame_index = operator.index(frame.frame_index)
        except TypeError as error:
            raise ValueError("frame_index must be an integer") from error
        if isinstance(frame.frame_index, bool):
            raise ValueError("frame_index must be an integer")
        if (
            self._last_frame_index is not None
            and frame_index <= self._last_frame_index
        ):
            self.clear()
            raise ValueError("frame_index values must be strictly increasing")

        validated = {
            name: _as_finite_float32(name, getattr(frame, name), shape)
            for name, shape in _POSE_FIELDS.items()
        }
        self._frames.append((frame_index, validated))
        self._last_frame_index = frame_index
        if not self.ready:
            return None

        frames = tuple(self._frames)
        return {
            "frame_index": np.asarray(
                [frame_index for frame_index, _ in frames], dtype=np.int64
            ),
            "smpl_joints": np.stack(
                [values["smpl_joints"] for _, values in frames]
            ).astype(np.float32),
            "body_quat_w": np.stack(
                [values["body_quat_w"] for _, values in frames]
            ).astype(np.float32),
            "joint_pos": np.stack(
                [values["joint_pos"] for _, values in frames]
            ).astype(np.float32),
            "stream_mode": np.array([1], dtype=np.int32),
            "calibration_ready": np.array([True], dtype=bool),
        }


class ZeroLabSourceCore:
    """Apply stale-stream semantics around ZeroLab conversion and chunking."""

    def __init__(
        self,
        converter: ZeroLabMotionConverter,
        window_frames: int = 10,
        stale_seconds: float = 0.5,
    ) -> None:
        if not np.isfinite(stale_seconds) or stale_seconds < 0.0:
            raise ValueError("stale_seconds must be finite and non-negative")
        self._converter = converter
        self._window = PoseChunkWindow(window_frames)
        self._stale_ns = int(float(stale_seconds) * 1_000_000_000)
        self._last_timestamp_ns = None
        self._stale_handled = False

    def _mark_stale(self) -> None:
        self._window.clear()
        self._converter.mark_stale()
        self._stale_handled = True

    def accept(self, packet):
        timestamp_ns = operator.index(packet.receive_timestamp_ns)
        if (
            self._last_timestamp_ns is not None
            and not self._stale_handled
            and timestamp_ns - self._last_timestamp_ns > self._stale_ns
        ):
            self._mark_stale()

        frame = self._converter.observe(packet)
        self._last_timestamp_ns = timestamp_ns
        self._stale_handled = False
        if frame is None:
            return None
        return self._window.append(frame)

    def check_stale(self, now_ns: int) -> bool:
        now_timestamp_ns = operator.index(now_ns)
        if (
            self._last_timestamp_ns is None
            or self._stale_handled
            or now_timestamp_ns - self._last_timestamp_ns <= self._stale_ns
        ):
            return False
        self._mark_stale()
        return True
