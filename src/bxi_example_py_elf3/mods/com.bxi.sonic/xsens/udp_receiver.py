"""Nonblocking UDP boundary for complete Xsens datagrams."""

from dataclasses import dataclass, replace
import socket
import time
from typing import Callable

from .protocol import PACKET_SIZE


@dataclass(frozen=True)
class ReceivedDatagram:
    """One complete datagram received from an allowed IPv4 sender."""

    payload: bytes
    receive_timestamp_ns: int
    sender_address: tuple[str, int]


@dataclass
class ReceiverStats:
    """Counters for datagrams observed by ``XsensUdpReceiver``."""

    received: int = 0
    delivered: int = 0
    invalid_size: int = 0
    unexpected_sender: int = 0


class XsensUdpReceiver:
    """Own a nonblocking IPv4 UDP socket and deliver complete datagrams."""

    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        port: int = 9763,
        allowed_sender_host: str | None = "127.0.0.1",
        *,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        sock: socket.socket | None = None,
    ) -> None:
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("port must be an integer in 1..65535")
        self._validate_host(bind_host, "bind_host")
        self._allowed_sender_ip = self._resolve_allowed_sender(
            allowed_sender_host
        )
        self._socket = (
            sock
            if sock is not None
            else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        )
        self._socket.setblocking(False)
        self._socket.bind((bind_host, port))
        self._clock_ns = clock_ns
        self._stats = ReceiverStats()
        self._closed = False

    @staticmethod
    def _validate_host(host: str, name: str) -> None:
        if not isinstance(host, str):
            raise ValueError(f"{name} must be a hostname or IPv4 address")
        try:
            socket.gethostbyname(host)
        except (OSError, TypeError) as error:
            raise ValueError(
                f"{name} must be a hostname or IPv4 address"
            ) from error

    @classmethod
    def _resolve_allowed_sender(cls, host: str | None) -> str | None:
        if host is None:
            return None
        cls._validate_host(host, "allowed_sender_host")
        return socket.gethostbyname(host)

    @property
    def stats(self) -> ReceiverStats:
        """Return a copy so callers cannot mutate the receiver's counters."""
        return replace(self._stats)

    def poll(self) -> ReceivedDatagram | None:
        """Return the next complete allowed datagram, if one is available."""
        while True:
            try:
                payload, sender_address = self._socket.recvfrom(
                    PACKET_SIZE + 1
                )
            except BlockingIOError:
                return None
            self._stats.received += 1
            sender_ip, sender_port = sender_address
            if (
                self._allowed_sender_ip is not None
                and sender_ip != self._allowed_sender_ip
            ):
                self._stats.unexpected_sender += 1
                continue
            if len(payload) != PACKET_SIZE:
                self._stats.invalid_size += 1
                continue
            self._stats.delivered += 1
            return ReceivedDatagram(
                payload=bytes(payload),
                receive_timestamp_ns=int(self._clock_ns()),
                sender_address=(str(sender_ip), int(sender_port)),
            )

    def drain(self, limit: int = 256) -> list[ReceivedDatagram]:
        """Return up to ``limit`` allowed datagrams in arrival order."""
        if type(limit) is not int or limit < 1:
            raise ValueError("limit must be a positive integer")
        datagrams = []
        while len(datagrams) < limit:
            datagram = self.poll()
            if datagram is None:
                break
            datagrams.append(datagram)
        return datagrams

    def close(self) -> None:
        """Release the owned socket once."""
        if self._closed:
            return
        self._socket.close()
        self._closed = True
