"""Convert Xsens segment transforms into the SONIC pose contract."""

from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation, Slerp

from .protocol import SEGMENT_COUNT, XsensPacket

if __package__ == "xsens":
    from pico.gear_sonic.trl.utils.elf3_wrist import build_elf3_joint_pos
    from pico.gear_sonic.trl.utils.numpy_smpl import compute_from_body_poses
else:
    from ..pico.gear_sonic.trl.utils.elf3_wrist import build_elf3_joint_pos
    from ..pico.gear_sonic.trl.utils.numpy_smpl import compute_from_body_poses


SMPL24_PARENTS = [
    -1,
    0,
    0,
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    9,
    9,
    12,
    13,
    14,
    16,
    17,
    18,
    19,
    20,
    21,
]

XSENS_TO_XRT = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)

DIRECT_SMPL_SEGMENT_IDS = {
    0: 1,
    1: 20,
    2: 16,
    3: 2,
    4: 21,
    5: 17,
    7: 22,
    8: 18,
    9: 5,
    10: 23,
    11: 19,
    12: 6,
    13: 12,
    14: 8,
    15: 7,
    16: 13,
    17: 9,
    18: 14,
    19: 10,
    20: 15,
    21: 11,
}

_SEGMENT_POSITION_SHAPE = (SEGMENT_COUNT, 3)
_SEGMENT_QUATERNION_SHAPE = (SEGMENT_COUNT, 4)
_SMPL_WORLD_QUATERNION_SHAPE = (24, 4)


@dataclass(frozen=True)
class ConvertedXsensFrame:
    """One immutable Xsens frame converted for SONIC consumption."""

    frame_index: int
    receive_timestamp_ns: int
    segment_positions_xrt: NDArray[np.float32]
    segment_quat_xrt_xyzw: NDArray[np.float32]
    smpl_body_pose: NDArray[np.float32]
    smpl_joints: NDArray[np.float32]
    body_quat_w: NDArray[np.float32]
    joint_pos: NDArray[np.float32]


def _real_matrix(name, values, columns):
    source = np.asarray(values)
    if not np.issubdtype(source.dtype, np.number) or np.issubdtype(
        source.dtype, np.complexfloating
    ):
        raise ValueError(f"{name} must use a real numeric dtype")
    matrix = source.astype(np.float64, copy=False)
    if (
        matrix.ndim != 2
        or matrix.shape[1] != columns
        or not np.isfinite(matrix).all()
    ):
        raise ValueError(
            f"{name} must be a finite two-dimensional array with "
            f"{columns} columns"
        )
    return matrix


def _normalize_xyzw(name, values, expected_shape=None):
    quaternions = _real_matrix(name, values, 4)
    if expected_shape is not None and quaternions.shape != expected_shape:
        raise ValueError(f"{name} must have shape {expected_shape}")
    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    if np.any(norms < 1e-6):
        raise ValueError(f"{name} contains a quaternion norm below 1e-6")
    return np.ascontiguousarray(quaternions / norms, dtype=np.float32)


def _normalize_wxyz_as_xyzw(name, values, expected_shape=None):
    wxyz = _real_matrix(name, values, 4)
    if expected_shape is not None and wxyz.shape != expected_shape:
        raise ValueError(f"{name} must have shape {expected_shape}")
    return _normalize_xyzw(name, wxyz[:, [1, 2, 3, 0]], wxyz.shape)


def _xsens_xyzw_to_xrt(quats_xyzw):
    """Apply the quaternion form of ``C R C.T`` while retaining signs."""
    quaternions = _normalize_xyzw("Xsens quaternion array", quats_xyzw)
    converted = np.empty_like(quaternions)
    converted[:, 0] = quaternions[:, 0]
    converted[:, 1] = quaternions[:, 2]
    converted[:, 2] = -quaternions[:, 1]
    converted[:, 3] = quaternions[:, 3]
    return np.ascontiguousarray(converted, dtype=np.float32)


def xsens_positions_to_xrt(positions_xsens):
    """Map Xsens positions ``[x, y, z]`` to XRT ``[x, z, -y]``."""
    positions = _real_matrix("Xsens position array", positions_xsens, 3)
    converted = positions @ XSENS_TO_XRT.T
    return np.ascontiguousarray(converted, dtype=np.float32)


def xsens_world_quaternions_to_xrt(quats_wxyz):
    """Normalize WXYZ Xsens world quaternions and express them in XRT."""
    source_xyzw = _normalize_wxyz_as_xyzw(
        "Xsens quaternion array", quats_wxyz
    )
    return _xsens_xyzw_to_xrt(source_xyzw)


def align_quaternion_signs(current_xyzw, previous_xyzw=None):
    """Select normalized representatives nearest the preceding frame."""
    current = _normalize_xyzw("current quaternion array", current_xyzw)
    if previous_xyzw is None:
        return current
    previous = _normalize_xyzw(
        "previous quaternion array", previous_xyzw, current.shape
    )
    flip = np.einsum("ij,ij->i", current, previous) < 0.0
    current[flip] *= -1.0
    return current


def shortest_path_slerp(start_xyzw, end_xyzw, fraction):
    """Interpolate normalized scalar-last quaternions on the short arc."""
    if isinstance(fraction, bool) or not np.isscalar(fraction):
        raise ValueError("Slerp fraction must be a finite scalar in [0, 1]")
    amount = float(fraction)
    if not np.isfinite(amount) or not 0.0 <= amount <= 1.0:
        raise ValueError("Slerp fraction must be a finite scalar in [0, 1]")
    start = _normalize_xyzw(
        "Slerp start quaternion", np.asarray(start_xyzw)[None, :], (1, 4)
    )[0]
    end = _normalize_xyzw(
        "Slerp end quaternion", np.asarray(end_xyzw)[None, :], (1, 4)
    )[0]
    if float(np.dot(start, end)) < 0.0:
        end *= -1.0
    rotations = Rotation.from_quat(np.stack((start, end)))
    interpolated = Slerp([0.0, 1.0], rotations)([amount]).as_quat()[0]
    return np.ascontiguousarray(interpolated, dtype=np.float32)


def synthesize_smpl_world_quats(segment_world_xyzw):
    """Map the 23 Xsens segment world rotations to the SMPL24 skeleton."""
    segments = _normalize_xyzw(
        "Xsens segment world quaternion array",
        segment_world_xyzw,
        _SEGMENT_QUATERNION_SHAPE,
    )
    smpl = np.empty(_SMPL_WORLD_QUATERNION_SHAPE, dtype=np.float32)
    for smpl_index, segment_id in DIRECT_SMPL_SEGMENT_IDS.items():
        smpl[smpl_index] = segments[segment_id - 1]
    smpl[6] = shortest_path_slerp(segments[2], segments[3], 0.5)
    smpl[22] = smpl[20]
    smpl[23] = smpl[21]
    return smpl


def world_to_parent_local_quats(world_xyzw, parents):
    """Convert SMPL world rotations to ``inverse(parent) * child`` locals."""
    world = _normalize_xyzw(
        "SMPL world quaternion array",
        world_xyzw,
        _SMPL_WORLD_QUATERNION_SHAPE,
    )
    if len(parents) != len(world):
        raise ValueError("SMPL parent list must contain 24 entries")
    local = np.empty_like(world)
    rotations = Rotation.from_quat(world)
    for index, parent in enumerate(parents):
        if parent == -1:
            if index != 0:
                raise ValueError("only the SMPL root may have parent -1")
            local[index] = world[index]
            continue
        if isinstance(parent, bool) or not isinstance(
            parent, (int, np.integer)
        ):
            raise ValueError("SMPL parent indices must be integers")
        if parent < 0 or parent >= index:
            raise ValueError("SMPL parents must precede their children")
        local[index] = (rotations[parent].inv() * rotations[index]).as_quat()
    return np.ascontiguousarray(local, dtype=np.float32)


def _validated_derived_array(
    name, values, expected_shape, *, quaternion_rows=False
):
    source = np.asarray(values)
    if not np.issubdtype(source.dtype, np.number) or np.issubdtype(
        source.dtype, np.complexfloating
    ):
        raise ValueError(f"derived {name} must use a real numeric dtype")
    if source.shape != expected_shape or not np.isfinite(source).all():
        raise ValueError(
            f"derived {name} must be finite with shape {expected_shape}"
        )
    result = np.array(source, dtype=np.float32, order="C", copy=True)
    if quaternion_rows:
        rows = result.reshape(-1, 4).astype(np.float64)
        norms = np.linalg.norm(rows, axis=1)
        if not np.all(np.isclose(norms, 1.0, atol=1e-5)):
            raise ValueError(f"derived {name} must contain unit quaternions")
    return _immutable_float32_copy(result)


def _immutable_float32_copy(values):
    contiguous = np.ascontiguousarray(values, dtype=np.float32)
    immutable = np.frombuffer(contiguous.tobytes(), dtype=np.float32)
    return immutable.reshape(contiguous.shape)


def _readonly_state_copy(values):
    if values is None:
        return None
    return _immutable_float32_copy(values)


def _convert_with_previous(
    packet: XsensPacket,
    frame_index: int,
    previous_quats: np.ndarray | None,
) -> tuple[ConvertedXsensFrame, np.ndarray]:
    positions_xrt = xsens_positions_to_xrt(packet.segment_positions_xsens)
    if positions_xrt.shape != _SEGMENT_POSITION_SHAPE:
        raise ValueError(
            f"Xsens segment position array must have shape "
            f"{_SEGMENT_POSITION_SHAPE}"
        )

    raw_xyzw = _normalize_wxyz_as_xyzw(
        "Xsens segment quaternion array",
        packet.segment_quat_wxyz_xsens,
        _SEGMENT_QUATERNION_SHAPE,
    )
    aligned_raw_xyzw = align_quaternion_signs(raw_xyzw, previous_quats)
    segment_quat_xrt = _xsens_xyzw_to_xrt(aligned_raw_xyzw)
    smpl_world_quats = synthesize_smpl_world_quats(segment_quat_xrt)

    body_poses = np.zeros((24, 7), dtype=np.float32)
    body_poses[:, 3:] = smpl_world_quats
    body_poses[0, :3] = positions_xrt[0]
    fk_result = compute_from_body_poses(SMPL24_PARENTS, body_poses)

    fk_pose = _validated_derived_array(
        "FK smpl_pose", fk_result["smpl_pose"], (1, 69)
    )
    smpl_body_pose = _validated_derived_array(
        "smpl_body_pose", fk_pose[0, :63].reshape(21, 3), (21, 3)
    )
    fk_joints = _validated_derived_array(
        "FK smpl_joints_local",
        fk_result["smpl_joints_local"],
        (1, 24, 3),
    )
    smpl_joints = _validated_derived_array(
        "smpl_joints", fk_joints[0], (24, 3)
    )
    fk_root_quat = _validated_derived_array(
        "FK global_orient_quat",
        fk_result["global_orient_quat"],
        (1, 4),
        quaternion_rows=True,
    )
    body_quat_w = _validated_derived_array(
        "body_quat_w", fk_root_quat[0], (4,), quaternion_rows=True
    )
    wrist_batch = _validated_derived_array(
        "ELF3 joint_pos",
        build_elf3_joint_pos(smpl_body_pose[None, ...]),
        (1, 29),
    )
    joint_pos = _validated_derived_array("joint_pos", wrist_batch[0], (29,))

    frame = ConvertedXsensFrame(
        frame_index=int(frame_index),
        receive_timestamp_ns=int(packet.receive_timestamp_ns),
        segment_positions_xrt=_validated_derived_array(
            "segment_positions_xrt", positions_xrt, _SEGMENT_POSITION_SHAPE
        ),
        segment_quat_xrt_xyzw=_validated_derived_array(
            "segment_quat_xrt_xyzw",
            segment_quat_xrt,
            _SEGMENT_QUATERNION_SHAPE,
            quaternion_rows=True,
        ),
        smpl_body_pose=smpl_body_pose,
        smpl_joints=smpl_joints,
        body_quat_w=body_quat_w,
        joint_pos=joint_pos,
    )
    return frame, np.ascontiguousarray(aligned_raw_xyzw, dtype=np.float32)


class XsensMotionConverter:
    """Stateful quaternion continuity with transactional frame conversion."""

    def __init__(self) -> None:
        self._previous_raw_quats_xyzw = None

    @property
    def previous_raw_quats_xyzw(self) -> np.ndarray | None:
        """Return a read-only copy of the committed sign-continuity state."""
        return _readonly_state_copy(self._previous_raw_quats_xyzw)

    def reset_epoch(self) -> None:
        """Clear sign continuity for a committed producer epoch change."""
        self._previous_raw_quats_xyzw = None

    def convert(
        self, packet: XsensPacket, *, frame_index: int
    ) -> ConvertedXsensFrame:
        """Convert and commit one frame only after all outputs validate."""
        frame, candidate_previous = _convert_with_previous(
            packet, frame_index, self._previous_raw_quats_xyzw
        )
        self._previous_raw_quats_xyzw = _readonly_state_copy(
            candidate_previous
        )
        return frame

    def convert_many(
        self,
        items: Iterable[tuple[XsensPacket, int]],
        *,
        reset_epoch: bool = False,
    ) -> tuple[ConvertedXsensFrame, ...]:
        """Convert a batch and atomically commit its final sign state."""
        candidate_previous = (
            None if reset_epoch else self._previous_raw_quats_xyzw
        )
        frames = []
        for packet, frame_index in items:
            frame, candidate_previous = _convert_with_previous(
                packet, frame_index, candidate_previous
            )
            frames.append(frame)
        self._previous_raw_quats_xyzw = _readonly_state_copy(
            candidate_previous
        )
        return tuple(frames)


__all__ = [
    "ConvertedXsensFrame",
    "DIRECT_SMPL_SEGMENT_IDS",
    "SMPL24_PARENTS",
    "XSENS_TO_XRT",
    "XsensMotionConverter",
    "align_quaternion_signs",
    "shortest_path_slerp",
    "synthesize_smpl_world_quats",
    "world_to_parent_local_quats",
    "xsens_positions_to_xrt",
    "xsens_world_quaternions_to_xrt",
]
