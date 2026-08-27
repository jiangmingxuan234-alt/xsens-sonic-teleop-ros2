from collections import deque
import socket
import struct

import numpy as np


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
