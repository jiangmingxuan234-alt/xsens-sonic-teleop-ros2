"""Quaternion calibration and ZeroLab-to-SMPL world-rotation mapping."""

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


BODY_JOINT_COUNT = 17
SMPL24_PARENTS = [
    -1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8,
    9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21,
]

_BODY_QUATERNION_SHAPE = (BODY_JOINT_COUNT, 4)


def _normalize_quaternions(quats, expected_shape):
    """Return finite, nonzero scalar-last quaternions as contiguous float32."""
    source = np.asarray(quats)
    if not np.issubdtype(source.dtype, np.number) or np.issubdtype(
        source.dtype, np.complexfloating
    ):
        raise ValueError("quaternion array must use a real numeric dtype")
    values = source.astype(np.float64, copy=False)
    if values.shape != expected_shape or not np.isfinite(values).all():
        raise ValueError(
            f"quaternion array must be finite with shape {expected_shape}"
        )
    norms = np.linalg.norm(values, axis=-1, keepdims=True)
    if np.any(norms < 1e-6):
        raise ValueError("quaternion norm is below 1e-6")
    return np.ascontiguousarray(values / norms, dtype=np.float32)


def align_quaternion_signs(current_xyzw, previous_xyzw=None):
    """Choose quaternion representatives closest to a previous frame."""
    current_values = np.asarray(current_xyzw)
    if current_values.ndim != 2 or current_values.shape[1] != 4:
        raise ValueError("quaternion array must have shape (N, 4)")
    current = _normalize_quaternions(current_values, current_values.shape)
    if previous_xyzw is None:
        return current
    previous = _normalize_quaternions(previous_xyzw, current.shape)
    flip = np.einsum("ij,ij->i", current, previous) < 0.0
    current[flip] *= -1.0
    return current


def apply_rest_alignment(raw_xyzw, rest_xyzw):
    """Express raw world rotations relative to calibrated rest rotations."""
    raw = _normalize_quaternions(raw_xyzw, _BODY_QUATERNION_SHAPE)
    rest = _normalize_quaternions(rest_xyzw, _BODY_QUATERNION_SHAPE)
    aligned = (
        Rotation.from_quat(raw) * Rotation.from_quat(rest).inv()
    ).as_quat()
    return np.ascontiguousarray(aligned, dtype=np.float32)


class TPoseCalibrator:
    """Collect a stable quaternion window and derive its T-pose rest frame."""

    def __init__(
        self,
        required_frames: int = 100,
        max_deviation_degrees: float = 5.0,
    ) -> None:
        if isinstance(required_frames, bool) or required_frames < 1:
            raise ValueError("required_frames must be at least 1")
        if (
            not np.isfinite(max_deviation_degrees)
            or max_deviation_degrees < 0.0
        ):
            raise ValueError(
                "max_deviation_degrees must be finite and non-negative"
            )
        self._required_frames = int(required_frames)
        self._max_deviation_degrees = float(max_deviation_degrees)
        self._window = []
        self._rest_quats_xyzw = None

    @property
    def frames_collected(self) -> int:
        return len(self._window)

    @property
    def is_calibrated(self) -> bool:
        return self._rest_quats_xyzw is not None

    @property
    def rest_quats_xyzw(self):
        if self._rest_quats_xyzw is None:
            return None
        return self._rest_quats_xyzw.copy()

    def reset(self) -> None:
        self._window = []
        self._rest_quats_xyzw = None

    def observe(self, frame_xyzw) -> bool:
        frame = _normalize_quaternions(frame_xyzw, _BODY_QUATERNION_SHAPE)
        if self.is_calibrated:
            return True

        previous = self._window[-1] if self._window else None
        frame = align_quaternion_signs(frame, previous)
        if self._window:
            mean = _normalize_quaternions(
                np.mean(self._window, axis=0, dtype=np.float64),
                _BODY_QUATERNION_SHAPE,
            )
            dots = np.sum(frame * mean, axis=1)
            angular_distances = np.degrees(
                2.0 * np.arccos(np.clip(np.abs(dots), 0.0, 1.0))
            )
            if np.any(angular_distances > self._max_deviation_degrees):
                self._window = [frame]
            else:
                self._window.append(frame)
        else:
            self._window.append(frame)

        if len(self._window) == self._required_frames:
            self._rest_quats_xyzw = _normalize_quaternions(
                np.mean(self._window, axis=0, dtype=np.float64),
                _BODY_QUATERNION_SHAPE,
            )
            return True
        return False


def _shortest_path_slerp(start_xyzw, end_xyzw, fraction):
    start = _normalize_quaternions(
        np.asarray(start_xyzw)[None, :], (1, 4)
    )[0]
    end = _normalize_quaternions(np.asarray(end_xyzw)[None, :], (1, 4))[0]
    if np.dot(start, end) < 0.0:
        end *= -1.0
    rotations = Rotation.from_quat(np.stack((start, end)))
    result = Slerp([0.0, 1.0], rotations)([fraction]).as_quat()[0]
    return np.ascontiguousarray(result, dtype=np.float32)


def synthesize_smpl_world_quats(aligned_body_xyzw):
    """Map 17 aligned ZeroLab world rotations to 24 SMPL world rotations."""
    body = _normalize_quaternions(aligned_body_xyzw, _BODY_QUATERNION_SHAPE)
    smpl = np.empty((24, 4), dtype=np.float32)

    smpl[0] = body[10]
    smpl[1] = body[11]
    smpl[2] = body[14]
    smpl[3] = _shortest_path_slerp(body[10], body[1], 1.0 / 3.0)
    smpl[4] = body[12]
    smpl[5] = body[15]
    smpl[6] = _shortest_path_slerp(body[10], body[1], 2.0 / 3.0)
    smpl[7] = body[13]
    smpl[8] = body[16]
    smpl[9] = body[1]
    smpl[10] = smpl[7]
    smpl[11] = smpl[8]
    smpl[12] = _shortest_path_slerp(body[1], body[0], 0.5)
    smpl[13] = body[2]
    smpl[14] = body[6]
    smpl[15] = body[0]
    smpl[16] = body[3]
    smpl[17] = body[7]
    smpl[18] = body[4]
    smpl[19] = body[8]
    smpl[20] = body[5]
    smpl[21] = body[9]
    smpl[22] = smpl[20]
    smpl[23] = smpl[21]
    return smpl
