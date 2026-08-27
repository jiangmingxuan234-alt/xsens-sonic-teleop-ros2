"""Parser for the fixed-size MXTP02 Xsens UDP datagram."""

from dataclasses import dataclass
import struct

import numpy as np
from numpy.typing import NDArray


PACKET_SIZE = 760
HEADER_SIZE = 24
SEGMENT_COUNT = 23
PAYLOAD_SIZE = 736
HEADER_STRUCT = struct.Struct(">6sIBBIBBBBHH")
SEGMENT_STRUCT = struct.Struct(">I7f")

_SEGMENT_IDS = frozenset(range(1, SEGMENT_COUNT + 1))
_QUATERNION_MIN_NORM = np.nextafter(np.float32(0.95), np.float32(-np.inf))
_QUATERNION_MAX_NORM = np.nextafter(np.float32(1.05), np.float32(np.inf))


class XsensProtocolError(ValueError):
    """Raised when a datagram does not conform to the MXTP02 contract."""


@dataclass(frozen=True)
class Mxtp02Header:
    sample_counter: int
    datagram_counter: int
    item_count: int
    time_code: int
    character_id: int
    body_segment_count: int
    prop_count: int
    finger_segment_count: int
    reserved: int
    payload_size: int


@dataclass(frozen=True)
class XsensPacket:
    header: Mxtp02Header
    receive_timestamp_ns: int
    sender_address: tuple[str, int]
    segment_positions_xsens: NDArray[np.float32]
    segment_quat_wxyz_xsens: NDArray[np.float32]
    raw_payload: bytes


def parse_mxtp02_packet(
    payload: bytes,
    *,
    receive_timestamp_ns: int,
    sender_address: tuple[str, int],
) -> XsensPacket:
    """Parse one MXTP02 datagram after complete structural validation."""
    raw = bytes(payload)
    if len(raw) != PACKET_SIZE:
        raise XsensProtocolError(f"MXTP02 packet must be {PACKET_SIZE} bytes")

    (
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
    ) = HEADER_STRUCT.unpack_from(raw)
    if identifier != b"MXTP02":
        raise XsensProtocolError("identifier must be MXTP02")
    if item_count != SEGMENT_COUNT:
        raise XsensProtocolError(f"item count must be {SEGMENT_COUNT}")
    if character_id != 0:
        raise XsensProtocolError("character ID must be 0")
    if body_segment_count != SEGMENT_COUNT:
        raise XsensProtocolError(f"body segment count must be {SEGMENT_COUNT}")
    if prop_count != 0:
        raise XsensProtocolError("prop count must be 0")
    if finger_segment_count != 0:
        raise XsensProtocolError("finger segment count must be 0")
    if payload_size != PAYLOAD_SIZE:
        raise XsensProtocolError(f"payload size must be {PAYLOAD_SIZE}")

    positions = np.empty((SEGMENT_COUNT, 3), dtype=np.float32)
    quaternions = np.empty((SEGMENT_COUNT, 4), dtype=np.float32)
    decoded_ids: set[int] = set()
    for row_index in range(SEGMENT_COUNT):
        offset = HEADER_SIZE + row_index * SEGMENT_STRUCT.size
        segment_id, *values = SEGMENT_STRUCT.unpack_from(raw, offset)
        if segment_id not in _SEGMENT_IDS:
            raise XsensProtocolError("segment IDs must be in the range 1..23")
        if segment_id in decoded_ids:
            raise XsensProtocolError("segment IDs must be unique")
        decoded_ids.add(segment_id)
        positions[segment_id - 1] = values[:3]
        quaternions[segment_id - 1] = values[3:]

    if decoded_ids != _SEGMENT_IDS:
        raise XsensProtocolError(
            "segment IDs must contain every ID from 1 through 23"
        )
    if not np.isfinite(positions).all() or not np.isfinite(quaternions).all():
        raise XsensProtocolError(
            "MXTP02 packet contains non-finite position or quaternion values"
        )
    quaternion_norms = np.linalg.norm(quaternions.astype(np.float64), axis=1)
    if np.any(
        (quaternion_norms < float(_QUATERNION_MIN_NORM))
        | (quaternion_norms > float(_QUATERNION_MAX_NORM))
    ):
        raise XsensProtocolError(
            "MXTP02 quaternion norm must be within [0.95, 1.05]"
        )

    positions.setflags(write=False)
    quaternions.setflags(write=False)
    return XsensPacket(
        header=Mxtp02Header(
            sample_counter=sample_counter,
            datagram_counter=datagram_counter,
            item_count=item_count,
            time_code=time_code,
            character_id=character_id,
            body_segment_count=body_segment_count,
            prop_count=prop_count,
            finger_segment_count=finger_segment_count,
            reserved=reserved,
            payload_size=payload_size,
        ),
        receive_timestamp_ns=int(receive_timestamp_ns),
        sender_address=(str(sender_address[0]), int(sender_address[1])),
        segment_positions_xsens=positions,
        segment_quat_wxyz_xsens=quaternions,
        raw_payload=raw,
    )
