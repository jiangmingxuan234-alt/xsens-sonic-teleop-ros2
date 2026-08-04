from pathlib import Path
import signal
import subprocess
import sys

import numpy as np

from zerolab.record_cli import (
    RecordingSummary,
    build_arg_parser,
    main,
    record_stream,
)
from zerolab.recording import iter_raw_records, read_recording_metadata
from zerolab.udp_receiver import ReceivedDatagram


def valid_payload():
    root = np.zeros(3, dtype="<f4")
    quats = np.zeros((47, 4), dtype="<f4")
    quats[:, 3] = 1.0
    hands = np.zeros(12, dtype="<u2")
    positions = np.zeros((17, 3), dtype="<f4")
    return (
        root.tobytes()
        + quats.tobytes()
        + hands.tobytes()
        + positions.tobytes()
    )


class FakeReceiver:
    def __init__(self, datagrams=()):
        self.datagrams = list(datagrams)
        self.drained = False
        self.closed = False

    def drain(self):
        if self.drained:
            return []
        self.drained = True
        return self.datagrams

    def close(self):
        self.closed = True


def test_record_only_validates_and_writes_without_pose_resources(tmp_path):
    bad = bytearray(valid_payload())
    bad[12:28] = bytes(16)
    receiver = FakeReceiver([
        ReceivedDatagram(bytes(bad), 100, 0, ("10.0.0.2", 4000)),
        ReceivedDatagram(valid_payload(), 120, 1, ("10.0.0.2", 4000)),
    ])

    summary = record_stream(
        receiver,
        tmp_path / "trial",
        should_stop=lambda: receiver.drained,
        sleep_fn=lambda _seconds: None,
        start_time_utc="2026-08-04T00:00:00Z",
    )

    assert summary == RecordingSummary(valid_packets=1, invalid_packets=1)
    records = list(iter_raw_records(tmp_path / "trial"))
    assert [
        (record.receive_timestamp_ns, record.local_frame_index)
        for record in records
    ] == [(120, 1)]
    assert records[0].payload == valid_payload()
    metadata = read_recording_metadata(tmp_path / "trial")
    assert metadata["sender"] == {"host": "10.0.0.2", "port": 4000}
    assert metadata["start_time_utc"] == "2026-08-04T00:00:00Z"


def test_record_only_propagates_disk_open_failure(tmp_path):
    regular_file = tmp_path / "not-a-directory"
    regular_file.write_text("occupied", encoding="utf-8")
    receiver = FakeReceiver([
        ReceivedDatagram(valid_payload(), 120, 1, ("10.0.0.2", 4000)),
    ])

    with np.testing.assert_raises(OSError):
        record_stream(
            receiver,
            regular_file / "trial",
            should_stop=lambda: receiver.drained,
            sleep_fn=lambda _seconds: None,
            start_time_utc="2026-08-04T00:00:00Z",
        )


def test_importing_record_only_cli_does_not_load_pose_stack():
    sonic_root = (
        Path(__file__).resolve().parents[1] / "mods" / "com.bxi.sonic"
    )
    script = f"""
import sys
sys.path.insert(0, {str(sonic_root)!r})
import zerolab.record_cli
forbidden = ("rclpy", "zmq", "zerolab.converter", "zerolab.source_node")
loaded = [name for name in forbidden if name in sys.modules]
if loaded:
    raise SystemExit("unexpected pose-stack imports: " + ", ".join(loaded))
"""

    result = subprocess.run(
        [sys.executable, "-I", "-c", script],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr


def test_argument_parser_supports_record_only_options():
    parser = build_arg_parser()

    defaults = parser.parse_args(["--output", "/tmp/trial"])
    configured = parser.parse_args([
        "--output", "/tmp/other",
        "--bind-host", "127.0.0.1",
        "--port", "19000",
        "--allowed-sender", "10.0.0.2",
        "--duration-seconds", "5.5",
    ])

    assert vars(defaults) == {
        "output": Path("/tmp/trial"),
        "bind_host": "0.0.0.0",
        "port": 18000,
        "allowed_sender": "",
        "duration_seconds": 0.0,
    }
    assert vars(configured) == {
        "output": Path("/tmp/other"),
        "bind_host": "127.0.0.1",
        "port": 19000,
        "allowed_sender": "10.0.0.2",
        "duration_seconds": 5.5,
    }


def test_main_rejects_negative_duration_without_creating_receiver(
    monkeypatch, capsys
):
    def unexpected_receiver(*_args, **_kwargs):
        raise AssertionError("receiver must not be created")

    monkeypatch.setattr(
        "zerolab.record_cli.ZeroLabUdpReceiver", unexpected_receiver
    )

    result = main(["--output", "/tmp/trial", "--duration-seconds", "-0.1"])

    assert result == 2
    assert capsys.readouterr().err == (
        "zerolab record-only: duration-seconds must be non-negative\n"
    )


def test_main_rejects_nonfinite_duration_without_creating_receiver(
    monkeypatch, capsys
):
    def unexpected_receiver(*_args, **_kwargs):
        raise AssertionError("receiver must not be created")

    monkeypatch.setattr(
        "zerolab.record_cli.ZeroLabUdpReceiver", unexpected_receiver
    )

    results = [
        main(["--output", "/tmp/trial", "--duration-seconds", value])
        for value in ("nan", "inf")
    ]

    assert results == [2, 2]
    assert capsys.readouterr().err == 2 * (
        "zerolab record-only: duration-seconds must be non-negative\n"
    )


def test_main_reports_bind_failure_with_error_status(monkeypatch, capsys):
    attempts = []

    def failing_receiver(*args, **kwargs):
        attempts.append((args, kwargs))
        raise OSError("address already in use")

    monkeypatch.setattr(
        "zerolab.record_cli.ZeroLabUdpReceiver", failing_receiver
    )

    result = main(["--output", "/tmp/trial"])

    assert result == 2
    assert len(attempts) == 1
    assert capsys.readouterr().err == (
        "zerolab record-only: address already in use\n"
    )


def test_main_reports_disk_failure_and_closes_receiver(
    tmp_path, monkeypatch, capsys
):
    regular_file = tmp_path / "not-a-directory"
    regular_file.write_text("occupied", encoding="utf-8")
    receiver = FakeReceiver([
        ReceivedDatagram(valid_payload(), 120, 1, ("10.0.0.2", 4000)),
    ])
    monkeypatch.setattr(
        "zerolab.record_cli.ZeroLabUdpReceiver",
        lambda *_args, **_kwargs: receiver,
    )
    monotonic_values = iter((0.0, 0.0))
    monkeypatch.setattr(
        "zerolab.record_cli.time.monotonic", lambda: next(monotonic_values)
    )

    result = main([
        "--output", str(regular_file / "trial"),
        "--duration-seconds", "1",
    ])

    assert result == 2
    assert receiver.closed
    assert "zerolab record-only:" in capsys.readouterr().err


def test_main_deadline_is_a_clean_stop_and_closes_receiver(monkeypatch):
    receiver = FakeReceiver()
    monkeypatch.setattr(
        "zerolab.record_cli.ZeroLabUdpReceiver",
        lambda *_args, **_kwargs: receiver,
    )
    monotonic_values = iter((10.0, 11.0))
    monkeypatch.setattr(
        "zerolab.record_cli.time.monotonic", lambda: next(monotonic_values)
    )

    result = main([
        "--output", "/tmp/unopened-trial",
        "--duration-seconds", "1",
    ])

    assert result == 0
    assert receiver.closed


def test_main_restores_signal_handlers_when_receiver_close_fails(monkeypatch):
    class CloseFailingReceiver(FakeReceiver):
        def close(self):
            raise OSError("close failed")

    receiver = CloseFailingReceiver()
    monkeypatch.setattr(
        "zerolab.record_cli.ZeroLabUdpReceiver",
        lambda *_args, **_kwargs: receiver,
    )
    monotonic_values = iter((10.0, 11.0))
    monkeypatch.setattr(
        "zerolab.record_cli.time.monotonic", lambda: next(monotonic_values)
    )
    previous_handlers = {
        signum: signal.getsignal(signum)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }

    try:
        with np.testing.assert_raises(OSError):
            main([
                "--output", "/tmp/unopened-trial",
                "--duration-seconds", "1",
            ])
        assert {
            signum: signal.getsignal(signum)
            for signum in previous_handlers
        } == previous_handlers
    finally:
        for signum, previous_handler in previous_handlers.items():
            signal.signal(signum, previous_handler)
