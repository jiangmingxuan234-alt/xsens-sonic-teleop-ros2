"""Standalone ZeroLab UDP recorder without ROS or pose-processing resources."""

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import signal
import sys
import threading
import time
from typing import Callable, Sequence

from .protocol import ZeroLabProtocolError, parse_zerolab_packet
from .recording import (
    RawRecord,
    RawRecordingWriter,
    build_recording_metadata,
)
from .udp_receiver import ZeroLabUdpReceiver


@dataclass(frozen=True)
class RecordingSummary:
    valid_packets: int
    invalid_packets: int


def record_stream(
    receiver: ZeroLabUdpReceiver,
    output: Path,
    *,
    should_stop: Callable[[], bool],
    sleep_fn: Callable[[float], None] = time.sleep,
    start_time_utc: str | None = None,
) -> RecordingSummary:
    writer = None
    valid_packets = 0
    invalid_packets = 0
    try:
        while not should_stop():
            datagrams = receiver.drain()
            for datagram in datagrams:
                try:
                    packet = parse_zerolab_packet(
                        datagram.payload,
                        receive_timestamp_ns=datagram.receive_timestamp_ns,
                        local_frame_index=datagram.local_frame_index,
                        sender_address=datagram.sender_address,
                    )
                except ZeroLabProtocolError:
                    invalid_packets += 1
                    continue
                if writer is None:
                    metadata = build_recording_metadata(
                        sender_address=packet.sender_address,
                        start_time_utc=(
                            start_time_utc
                            or datetime.now(timezone.utc).isoformat()
                        ),
                    )
                    writer = RawRecordingWriter(Path(output), metadata)
                writer.append(RawRecord(
                    packet.receive_timestamp_ns,
                    packet.local_frame_index,
                    packet.raw_payload,
                ))
                valid_packets += 1
            if not datagrams:
                sleep_fn(0.005)
    finally:
        if writer is not None:
            writer.close()
    return RecordingSummary(valid_packets, invalid_packets)


class _RecordArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise ValueError(message)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = _RecordArgumentParser(
        description="Record raw ZeroLab F2 Pro UDP packets."
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--port", default=18000, type=int)
    parser.add_argument("--allowed-sender", default="")
    parser.add_argument("--duration-seconds", default=0.0, type=float)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = build_arg_parser().parse_args(argv)
    except ValueError as exc:
        print(f"zerolab record-only: {exc}", file=sys.stderr)
        return 2
    except SystemExit as exc:
        return int(exc.code)

    if not math.isfinite(args.duration_seconds) or args.duration_seconds < 0:
        print(
            "zerolab record-only: duration-seconds must be non-negative",
            file=sys.stderr,
        )
        return 2

    stop_event = threading.Event()

    def request_stop(_signum, _frame):
        stop_event.set()

    previous_handlers = {
        signal.SIGINT: signal.getsignal(signal.SIGINT),
        signal.SIGTERM: signal.getsignal(signal.SIGTERM),
    }
    receiver = None
    try:
        for signum in previous_handlers:
            signal.signal(signum, request_stop)
        receiver = ZeroLabUdpReceiver(
            bind_host=args.bind_host,
            port=args.port,
            allowed_sender_host=args.allowed_sender or None,
        )
        deadline = (
            time.monotonic() + args.duration_seconds
            if args.duration_seconds > 0
            else None
        )

        def should_stop() -> bool:
            return stop_event.is_set() or (
                deadline is not None and time.monotonic() >= deadline
            )

        record_stream(receiver, args.output, should_stop=should_stop)
    except (OSError, ValueError) as exc:
        print(f"zerolab record-only: {exc}", file=sys.stderr)
        return 2
    finally:
        try:
            if receiver is not None:
                receiver.close()
        finally:
            for signum, previous_handler in previous_handlers.items():
                signal.signal(signum, previous_handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
