import dataclasses

import pytest

from xsens.protocol import PACKET_SIZE
from xsens.udp_receiver import ReceivedDatagram, XsensUdpReceiver
from xsens_test_helpers import FakeDatagramSocket, reserve_udp_port


def test_receiver_filters_ip_and_uses_monotonic_receive_timestamp():
    sock = FakeDatagramSocket([
        (b"x" * 760, ("10.0.0.4", 4000)),
        (b"y" * 760, ("127.0.0.1", 4001)),
    ])
    receiver = XsensUdpReceiver(
        allowed_sender_host="127.0.0.1",
        clock_ns=lambda: 123_456_789,
        sock=sock,
    )

    received = receiver.poll()

    assert received is not None
    assert received.receive_timestamp_ns == 123_456_789
    assert received.sender_address == ("127.0.0.1", 4001)
    assert receiver.stats.unexpected_sender == 1


def test_receiver_does_not_invent_a_local_frame_index():
    fields = dataclasses.fields(ReceivedDatagram)

    assert [field.name for field in fields] == [
        "payload", "receive_timestamp_ns", "sender_address"
    ]


@pytest.mark.parametrize("size", [PACKET_SIZE - 1, PACKET_SIZE + 1])
def test_poll_skips_wrong_sizes_and_uses_oversize_receive_buffer(size):
    sock = FakeDatagramSocket([
        (b"x" * size, ("127.0.0.1", 4000)),
        (b"y" * PACKET_SIZE, ("127.0.0.1", 4001)),
    ])
    receiver = XsensUdpReceiver(sock=sock, clock_ns=lambda: 9)

    received = receiver.poll()

    assert received is not None
    assert received.sender_address == ("127.0.0.1", 4001)
    assert receiver.stats.invalid_size == 1
    assert set(sock.recv_sizes) == {PACKET_SIZE + 1}


def test_timestamp_is_not_taken_for_filtered_or_incomplete_datagrams():
    ticks = iter([17])
    sock = FakeDatagramSocket([
        (b"x" * PACKET_SIZE, ("10.0.0.4", 4000)),
        (b"y" * (PACKET_SIZE - 1), ("127.0.0.1", 4001)),
        (b"z" * PACKET_SIZE, ("127.0.0.1", 4002)),
    ])
    receiver = XsensUdpReceiver(
        allowed_sender_host="127.0.0.1",
        clock_ns=lambda: next(ticks),
        sock=sock,
    )

    received = receiver.poll()

    assert received is not None
    assert received.receive_timestamp_ns == 17


def test_drain_preserves_order_and_limit():
    queued = [
        (bytes([value]) * PACKET_SIZE, ("127.0.0.1", 4100 + value))
        for value in range(3)
    ]
    receiver = XsensUdpReceiver(
        sock=FakeDatagramSocket(queued), clock_ns=lambda: 1
    )

    assert [
        item.sender_address[1] for item in receiver.drain(limit=2)
    ] == [4100, 4101]
    assert [
        item.sender_address[1] for item in receiver.drain(limit=2)
    ] == [4102]


@pytest.mark.parametrize("port", [True, False, -1, 0, 65536])
def test_rejects_unsafe_ports(port):
    with pytest.raises(ValueError, match="port"):
        XsensUdpReceiver(port=port, sock=FakeDatagramSocket())


def test_stats_snapshot_cannot_change_receiver_statistics():
    receiver = XsensUdpReceiver(sock=FakeDatagramSocket())
    snapshot = receiver.stats
    snapshot.delivered = 99

    assert receiver.stats.delivered == 0


def test_close_is_idempotent_and_real_port_can_be_rebound():
    fake = FakeDatagramSocket()
    receiver = XsensUdpReceiver(sock=fake)
    receiver.close()
    receiver.close()

    assert fake.closed
    assert fake.close_calls == 1
    assert fake.bound == ("0.0.0.0", 9763)
    assert fake.blocking is False

    port = reserve_udp_port()
    for _ in range(2):
        live = XsensUdpReceiver(bind_host="127.0.0.1", port=port)
        live.close()
