import numpy as np
import pytest

from xsens.protocol import (
    HEADER_SIZE,
    PACKET_SIZE,
    PAYLOAD_SIZE,
    SEGMENT_COUNT,
    Mxtp02Header,
    XsensProtocolError,
    parse_mxtp02_packet,
)
from xsens_test_helpers import build_mxtp02_packet


DISTINCT_VALID_QUATERNIONS = np.array(
    [
        [0.960, 0.001, -0.002, 0.002],
        [0.961, 0.002, -0.003, 0.004],
        [0.962, 0.003, -0.004, 0.006],
        [0.963, 0.004, -0.005, 0.008],
        [0.964, 0.005, -0.006, 0.010],
        [0.965, 0.006, -0.007, 0.012],
        [0.966, 0.007, -0.008, 0.014],
        [0.967, 0.008, -0.009, 0.016],
        [0.968, 0.009, -0.010, 0.018],
        [0.969, 0.010, -0.011, 0.020],
        [0.970, 0.011, -0.012, 0.022],
        [0.971, 0.012, -0.013, 0.024],
        [0.972, 0.013, -0.014, 0.026],
        [0.973, 0.014, -0.015, 0.028],
        [0.974, 0.015, -0.016, 0.030],
        [0.975, 0.016, -0.017, 0.032],
        [0.976, 0.017, -0.018, 0.034],
        [0.977, 0.018, -0.019, 0.036],
        [0.978, 0.019, -0.020, 0.038],
        [0.979, 0.020, -0.021, 0.040],
        [0.980, 0.021, -0.022, 0.042],
        [0.981, 0.022, -0.023, 0.044],
        [0.982, 0.023, -0.024, 0.046],
    ],
    dtype=np.float32,
)


def test_decodes_all_header_metadata_and_freezes_arrays():
    positions = np.arange(69, dtype=np.float32).reshape(23, 3) / 10.0
    payload = build_mxtp02_packet(
        sample_counter=0xFEDCBA98,
        time_code=0x89ABCDEF,
        datagram_counter=0xFE,
        reserved=0xFEDC,
        positions=positions,
        quaternions_wxyz=DISTINCT_VALID_QUATERNIONS,
    )
    packet = parse_mxtp02_packet(
        payload,
        receive_timestamp_ns=987_654_321,
        sender_address=("127.0.0.1", 43123),
    )
    assert (PACKET_SIZE, HEADER_SIZE, PAYLOAD_SIZE, SEGMENT_COUNT) == (
        760,
        24,
        736,
        23,
    )
    assert packet.header == Mxtp02Header(
        sample_counter=0xFEDCBA98,
        datagram_counter=0xFE,
        item_count=23,
        time_code=0x89ABCDEF,
        character_id=0,
        body_segment_count=23,
        prop_count=0,
        finger_segment_count=0,
        reserved=0xFEDC,
        payload_size=736,
    )
    assert packet.receive_timestamp_ns == 987_654_321
    assert packet.sender_address == ("127.0.0.1", 43123)
    assert packet.raw_payload == payload
    np.testing.assert_array_equal(packet.segment_positions_xsens, positions)
    np.testing.assert_array_equal(
        packet.segment_quat_wxyz_xsens, DISTINCT_VALID_QUATERNIONS
    )
    assert packet.segment_positions_xsens.dtype == np.float32
    assert packet.segment_quat_wxyz_xsens.dtype == np.float32
    assert packet.segment_positions_xsens.shape == (23, 3)
    assert packet.segment_quat_wxyz_xsens.shape == (23, 4)
    assert not packet.segment_positions_xsens.flags.writeable
    assert not packet.segment_quat_wxyz_xsens.flags.writeable
    with pytest.raises(ValueError, match="read-only"):
        packet.segment_positions_xsens[0, 0] = 9.0


def test_segment_rows_are_id_indexed_independent_of_arrival_order():
    positions = np.arange(69, dtype=np.float32).reshape(23, 3)
    packet = parse_mxtp02_packet(
        build_mxtp02_packet(
            segment_order=tuple(reversed(range(1, 24))),
            positions=positions,
            quaternions_wxyz=DISTINCT_VALID_QUATERNIONS,
        ),
        receive_timestamp_ns=1,
        sender_address=("127.0.0.1", 4000),
    )
    np.testing.assert_array_equal(packet.segment_positions_xsens, positions)
    np.testing.assert_array_equal(
        packet.segment_quat_wxyz_xsens, DISTINCT_VALID_QUATERNIONS
    )


@pytest.mark.parametrize(
    "payload",
    [
        build_mxtp02_packet()[:-1],
        build_mxtp02_packet() + b"x",
        build_mxtp02_packet(identifier=b"BAD002"),
        build_mxtp02_packet(item_count=22),
        build_mxtp02_packet(character_id=1),
        build_mxtp02_packet(body_segment_count=22),
        build_mxtp02_packet(prop_count=1),
        build_mxtp02_packet(finger_segment_count=1),
        build_mxtp02_packet(payload_size=735),
        build_mxtp02_packet(segment_order=tuple(range(1, 23))),
        build_mxtp02_packet(segment_order=tuple(range(1, 23)) + (22,)),
        build_mxtp02_packet(segment_order=tuple(range(1, 23)) + (24,)),
    ],
)
def test_rejects_malformed_header_or_segment_set(payload):
    with pytest.raises(XsensProtocolError):
        parse_mxtp02_packet(
            payload,
            receive_timestamp_ns=5,
            sender_address=("127.0.0.1", 4000),
        )


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_rejects_nonfinite_position_or_quaternion(bad):
    positions = np.zeros((23, 3), dtype=np.float32)
    positions[4, 2] = bad
    quats = np.tile(np.array([1, 0, 0, 0], np.float32), (23, 1))
    quats[7, 1] = bad
    for payload in (
        build_mxtp02_packet(positions=positions),
        build_mxtp02_packet(quaternions_wxyz=quats),
    ):
        with pytest.raises(XsensProtocolError):
            parse_mxtp02_packet(
                payload,
                receive_timestamp_ns=5,
                sender_address=("127.0.0.1", 4000),
            )


def test_norm_boundaries_are_closed_at_float32_wire_precision():
    for accepted in (
        np.nextafter(np.float32(0.95), np.float32(-np.inf)),
        np.float32(0.95),
        np.float32(1.05),
        np.nextafter(np.float32(1.05), np.float32(np.inf)),
    ):
        quats = np.zeros((23, 4), dtype=np.float32)
        quats[:, 0] = accepted
        parse_mxtp02_packet(
            build_mxtp02_packet(quaternions_wxyz=quats),
            receive_timestamp_ns=5,
            sender_address=("127.0.0.1", 4000),
        )
    for rejected in (
        np.nextafter(
            np.nextafter(np.float32(0.95), np.float32(-np.inf)),
            np.float32(-np.inf),
        ),
        np.nextafter(
            np.nextafter(np.float32(1.05), np.float32(np.inf)),
            np.float32(np.inf),
        ),
    ):
        quats = np.zeros((23, 4), dtype=np.float32)
        quats[:, 0] = rejected
        with pytest.raises(XsensProtocolError):
            parse_mxtp02_packet(
                build_mxtp02_packet(quaternions_wxyz=quats),
                receive_timestamp_ns=5,
                sender_address=("127.0.0.1", 4000),
            )
