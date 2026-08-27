from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
import zmq
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)
from std_msgs.msg import Int64MultiArray, MultiArrayDimension

from bxi_example_py_elf3.framework.mod_api import NodeBuildContext
from pico.zmq_messages import pack_pose_message
from xsens.converter import XsensMotionConverter
from xsens.source_core import (
    AcceptClassification,
    AcceptResult,
    ArmCommandClassification,
    ArmCommandResult,
    SourceCoreStats,
    XsensSourceCore,
)
from xsens.source_node import (
    SOURCE_DEFAULTS,
    XsensPoseStatusPublisher,
    XsensSourceNode,
    create_node,
    random_epoch_candidate,
    validate_source_params,
)
from xsens.udp_receiver import ReceivedDatagram, ReceiverStats
from xsens_test_helpers import (
    FakeClock,
    FakeZmqContext,
    build_mxtp02_packet,
    make_status,
    make_xsens_pose_fields,
    reserve_tcp_port,
    reserve_udp_port,
)


MOD_ROOT = Path(__file__).resolve().parents[1] / "mods" / "com.bxi.sonic"


@pytest.fixture
def rclpy_runtime():
    started_here = not rclpy.ok()
    if started_here:
        rclpy.init(args=[])
    yield
    if started_here and rclpy.ok():
        rclpy.shutdown()


def source_context(udp_port: int, pose_port: int, node_name: str):
    params = dict(SOURCE_DEFAULTS)
    params.update(
        {
            "udp_bind_host": "127.0.0.1",
            "udp_port": udp_port,
            "allowed_sender": "127.0.0.1",
            "pose_port": pose_port,
        }
    )
    return NodeBuildContext(
        mod_id="com.bxi.sonic",
        node_id=f"com.bxi.sonic/{node_name}",
        node_name=node_name,
        mod_root=MOD_ROOT,
        params=params,
    )


@pytest.fixture
def xsens_source_context():
    return source_context(
        reserve_udp_port(), reserve_tcp_port(), "xsens_unit"
    )


class FakeReceiver:
    def __init__(self, events=None):
        self.pending = []
        self.closed = False
        self.stats = ReceiverStats()
        self.events = events
        self.drain_error = None

    def drain(self, limit=256):
        if self.drain_error is not None:
            error, self.drain_error = self.drain_error, None
            raise error
        batch, self.pending = self.pending[:limit], self.pending[limit:]
        return batch

    def close(self):
        if self.events is not None:
            self.events.append("receiver")
        self.closed = True


class FakePublisher:
    def __init__(self, events=None):
        self.sent = []
        self.attempted = []
        self.fail_next_topic = None
        self.closed = False
        self.drop_count = 0
        self.events = events

    def send(self, topic, fields):
        self.attempted.append(topic)
        if self.fail_next_topic == topic:
            self.fail_next_topic = None
            self.drop_count += 1
            return False
        self.sent.append(
            (topic, {key: value.copy() for key, value in fields.items()})
        )
        return True

    def close(self):
        if self.events is not None:
            self.events.append("publisher")
        self.closed = True


class FakeCore:
    def __init__(self):
        self.source_epoch = 71
        self.commands = []
        self.accept_results = []
        self.status_sequence = 0
        self.pose_fields = make_xsens_pose_fields(source_epoch=71)
        self.stats = SourceCoreStats()
        self.locked_sender = ("127.0.0.1", 4000, 0)
        self.newest_frame_index = 109
        self.newest_producer_monotonic_ns = 1_000_000_000
        self.invalid_inputs = 0
        self.calls = []

    def expire(self, now_ns):
        self.calls.append(("expire", now_ns))
        return False

    def check_stale(self, now_ns):
        self.calls.append(("stale", now_ns))
        return False

    def accept(self, packet):
        self.calls.append(("accept", packet.receive_timestamp_ns))
        if self.accept_results:
            return self.accept_results.pop(0)
        return AcceptResult(True, AcceptClassification.FORWARD)

    def note_invalid_input(self):
        self.invalid_inputs += 1

    def handle_arm_command(self, command):
        if command in self.commands:
            return ArmCommandResult(
                ArmCommandClassification.DUPLICATE, False, False, False
            )
        self.commands.append(command)
        return ArmCommandResult(
            ArmCommandClassification.DISARMED, True, True, True
        )

    def build_status_fields(self, now_ns):
        self.status_sequence += 1
        return make_status(
            status_sequence=self.status_sequence,
            status_monotonic_ns=now_ns,
            source_epoch=self.source_epoch,
            newest_frame_index=self.newest_frame_index,
            producer_monotonic_ns=self.newest_producer_monotonic_ns,
        )

    def build_pose_fields(self):
        return self.pose_fields


class NodeHarness:
    def __init__(self, context, *, integrated=False):
        self.clock = FakeClock(1_000_000_000)
        self.receiver = FakeReceiver()
        self.publisher = FakePublisher()
        self.converter = XsensMotionConverter()
        if integrated:
            draws = iter((71, 72, 73))
            self.core = XsensSourceCore(
                self.converter,
                clock_ns=self.clock.monotonic_ns,
                epoch_factory=lambda: next(draws),
            )
        else:
            self.core = FakeCore()
        self.node = XsensSourceNode(
            context,
            clock_ns=self.clock.monotonic_ns,
            receiver_factory=lambda **kwargs: self.receiver,
            converter_factory=lambda: self.converter,
            core_factory=lambda *args, **kwargs: self.core,
            publisher_factory=lambda *args, **kwargs: self.publisher,
        )

    def feed(self, *, counter, timestamp_ns, time_code=0):
        self.clock.now_ns = timestamp_ns
        self.receiver.pending.append(
            ReceivedDatagram(
                payload=build_mxtp02_packet(
                    sample_counter=counter, time_code=time_code
                ),
                receive_timestamp_ns=timestamp_ns,
                sender_address=("127.0.0.1", 4000),
            )
        )
        self.node._tick()


@pytest.fixture
def node_harness(rclpy_runtime, xsens_source_context):
    harness = NodeHarness(xsens_source_context)
    yield harness
    harness.node.destroy_node()


@pytest.fixture
def integrated_node_harness(rclpy_runtime, xsens_source_context):
    harness = NodeHarness(xsens_source_context, integrated=True)
    yield harness
    harness.node.destroy_node()


def make_arm_message(command_id, target_epoch, requested_epoch):
    message = Int64MultiArray()
    message.layout.dim = []
    message.layout.data_offset = 0
    message.data = [command_id, target_epoch, requested_epoch]
    return message


def test_defaults_are_the_state_scoped_xsens_contract():
    assert SOURCE_DEFAULTS == {
        "udp_bind_host": "0.0.0.0",
        "udp_port": 9763,
        "allowed_sender": "127.0.0.1",
        "pose_host": "127.0.0.1",
        "pose_port": 5559,
        "pose_topic": "pose",
        "status_topic": "xsens_status",
        "arm_command_topic": "sonic/xsens_arm_command",
        "status_rate_hz": 50.0,
        "input_rate_hz": 60.0,
        "publish_rate_hz": 50.0,
        "window_frames": 10,
        "same_epoch_resume_frames": 10,
        "ready_frames": 30,
        "stale_seconds": 0.5,
        "epoch_candidate_frames": 2,
        "epoch_candidate_timeout_s": 0.25,
        "max_pelvis_span_m": 0.15,
        "max_segment_deviation_deg": 20.0,
    }
    assert validate_source_params({}) == SOURCE_DEFAULTS


@pytest.mark.parametrize(
    "params,match",
    [
        ({"unknown": 1}, "unknown"),
        ({"udp_bind_host": ""}, "udp_bind_host"),
        ({"allowed_sender": 7}, "allowed_sender"),
        ({"udp_port": 0}, "udp_port"),
        ({"udp_port": True}, "udp_port"),
        ({"pose_port": 65536}, "pose_port"),
        ({"pose_host": "0.0.0.0"}, "pose_host"),
        ({"pose_topic": ""}, "pose_topic"),
        ({"status_topic": ""}, "status_topic"),
        ({"arm_command_topic": ""}, "arm_command_topic"),
        ({"status_rate_hz": 49.0}, "status_rate_hz"),
        ({"status_rate_hz": True}, "status_rate_hz"),
        ({"input_rate_hz": 50.0}, "input_rate_hz"),
        ({"input_rate_hz": np.inf}, "input_rate_hz"),
        ({"publish_rate_hz": 60.0}, "publish_rate_hz"),
        ({"window_frames": 9}, "window_frames"),
        ({"window_frames": True}, "window_frames"),
        ({"same_epoch_resume_frames": 9}, "same_epoch_resume_frames"),
        ({"ready_frames": 29}, "ready_frames"),
        ({"stale_seconds": 0.0}, "stale_seconds"),
        ({"stale_seconds": False}, "stale_seconds"),
        ({"epoch_candidate_frames": 1}, "epoch_candidate_frames"),
        ({"epoch_candidate_frames": True}, "epoch_candidate_frames"),
        ({"epoch_candidate_timeout_s": 0.0}, "epoch_candidate_timeout_s"),
        ({"max_pelvis_span_m": -0.1}, "max_pelvis_span_m"),
        ({"max_segment_deviation_deg": 181.0}, "max_segment_deviation_deg"),
    ],
)
def test_source_rejects_unknown_or_unsafe_params(params, match):
    with pytest.raises(ValueError, match=match):
        validate_source_params(params)


def test_random_epoch_candidate_delegates_unfiltered_to_randbits(monkeypatch):
    calls = []
    monkeypatch.setattr(
        "xsens.source_node.secrets.randbits",
        lambda bits: calls.append(bits) or 0,
    )
    assert random_epoch_candidate() == 0
    assert calls == [63]


def test_publisher_packs_topic_and_uses_nonblocking_pub_socket():
    context = FakeZmqContext()
    publisher = XsensPoseStatusPublisher(
        "127.0.0.1", 5559, zmq_context=context
    )
    fields = make_status(status_sequence=41)
    assert publisher.send("xsens_status", fields)
    assert context.socket_types == [zmq.PUB]
    assert (zmq.LINGER, 0) in context.fake_socket.options
    assert context.fake_socket.bound == "tcp://127.0.0.1:5559"
    assert context.fake_socket.messages == [
        (pack_pose_message(fields, topic="xsens_status"), zmq.NOBLOCK)
    ]


def test_publisher_counts_nonblocking_drops_and_keeps_context_injected():
    context = FakeZmqContext()
    publisher = XsensPoseStatusPublisher(
        "127.0.0.1", 5559, zmq_context=context
    )
    context.fake_socket.send_error = zmq.Again()
    assert not publisher.send("pose", make_xsens_pose_fields())
    assert publisher.drop_count == 1
    publisher.close()
    publisher.close()
    assert context.fake_socket.close_calls == 1
    assert context.term_calls == 0


def test_publisher_owns_default_context_and_rolls_back_bind_failure(
    monkeypatch,
):
    context = FakeZmqContext()
    monkeypatch.setattr("xsens.source_node.zmq.Context", lambda: context)
    context.fake_socket.bind_error = RuntimeError("bind failed")
    with pytest.raises(RuntimeError, match="bind failed"):
        XsensPoseStatusPublisher("127.0.0.1", 5559)
    assert context.fake_socket.closed
    assert context.term_calls == 1


def test_publisher_terminates_owned_context_once(monkeypatch):
    context = FakeZmqContext()
    monkeypatch.setattr("xsens.source_node.zmq.Context", lambda: context)
    publisher = XsensPoseStatusPublisher("127.0.0.1", 5559)
    publisher.close()
    publisher.close()
    assert context.fake_socket.close_calls == 1
    assert context.term_calls == 1


def test_node_passes_exact_factory_arguments_and_epoch_factory(
    rclpy_runtime, xsens_source_context
):
    calls = []
    clock = FakeClock(123)
    receiver = FakeReceiver()
    publisher = FakePublisher()
    converter = object()
    core = FakeCore()

    def epoch_factory():
        return 71

    def receiver_factory(**kwargs):
        calls.append(("receiver", kwargs))
        return receiver

    def converter_factory():
        calls.append(("converter", {}))
        return converter

    def core_factory(value, **kwargs):
        calls.append(("core", value, kwargs))
        return core

    def publisher_factory(host, port):
        calls.append(("publisher", host, port))
        return publisher

    node = XsensSourceNode(
        xsens_source_context,
        clock_ns=clock.monotonic_ns,
        epoch_factory=epoch_factory,
        receiver_factory=receiver_factory,
        converter_factory=converter_factory,
        core_factory=core_factory,
        publisher_factory=publisher_factory,
    )
    try:
        params = xsens_source_context.params
        assert calls[0] == (
            "receiver",
            {
                "bind_host": params["udp_bind_host"],
                "port": params["udp_port"],
                "allowed_sender_host": params["allowed_sender"],
                "clock_ns": clock.monotonic_ns,
            },
        )
        assert calls[1] == ("converter", {})
        assert calls[2][0:2] == ("core", converter)
        assert calls[2][2] == {
            "clock_ns": clock.monotonic_ns,
            "epoch_factory": epoch_factory,
            "window_frames": 10,
            "same_epoch_resume_frames": 10,
            "ready_frames": 30,
            "stale_seconds": 0.5,
            "epoch_candidate_frames": 2,
            "epoch_candidate_timeout_s": 0.25,
            "max_pelvis_span_m": 0.15,
            "max_segment_deviation_deg": 20.0,
        }
        assert calls[3] == ("publisher", "127.0.0.1", params["pose_port"])
        assert publisher.sent[0][0] == "xsens_status"
    finally:
        node.destroy_node()


def test_arm_subscription_is_reliable_and_volatile(node_harness):
    requested = node_harness.node._subscription.qos_profile
    assert requested.reliability is ReliabilityPolicy.RELIABLE
    assert requested.durability is DurabilityPolicy.VOLATILE
    assert requested.history is HistoryPolicy.KEEP_LAST
    assert requested.depth == 10

    topic = node_harness.node.resolve_topic_name("sonic/xsens_arm_command")
    endpoints = node_harness.node.get_subscriptions_info_by_topic(topic)
    assert len(endpoints) == 1
    qos = endpoints[0].qos_profile
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.VOLATILE


def test_arm_callback_rejects_nonempty_layout_and_bad_length(node_harness):
    message = Int64MultiArray()
    message.layout.dim = [
        MultiArrayDimension(label="invalid", size=3, stride=3)
    ]
    message.data = [1, 2, 2]
    node_harness.node._arm_command_callback(message)

    message = Int64MultiArray()
    message.data = [1, 2]
    node_harness.node._arm_command_callback(message)
    assert node_harness.core.commands == []
    assert node_harness.core.invalid_inputs == 2


@pytest.mark.parametrize(
    "values",
    [
        [True, 71, 0],
        [1, -(2**63) - 1, 0],
        [1, 71, 2**63],
    ],
)
def test_arm_callback_rejects_values_outside_signed_int64(
    values, node_harness
):
    message = SimpleNamespace(
        layout=SimpleNamespace(dim=[], data_offset=0), data=values
    )
    node_harness.node._arm_command_callback(message)
    assert node_harness.core.commands == []
    assert node_harness.core.invalid_inputs == 1


def test_arm_callback_rejects_nonzero_data_offset(node_harness):
    message = make_arm_message(1, 71, 0)
    message.layout.data_offset = 1
    node_harness.node._arm_command_callback(message)
    assert node_harness.core.commands == []
    assert node_harness.core.invalid_inputs == 1


def test_first_seen_command_publishes_status_and_duplicate_waits_for_tick(
    node_harness,
):
    message = make_arm_message(31, node_harness.core.source_epoch, 0)
    node_harness.node._arm_command_callback(message)
    sends_after_first = len(node_harness.publisher.sent)
    assert node_harness.publisher.sent[-1][0] == "xsens_status"
    node_harness.node._arm_command_callback(message)
    assert len(node_harness.publisher.sent) == sends_after_first
    node_harness.node._tick()
    assert node_harness.publisher.sent[-1][0] == "xsens_status"


def test_startup_and_empty_ticks_publish_status_heartbeat(node_harness):
    assert [topic for topic, _ in node_harness.publisher.sent] == [
        "xsens_status"
    ]
    first = int(node_harness.publisher.sent[-1][1]["status_sequence"][0])
    node_harness.node._tick()
    second = int(node_harness.publisher.sent[-1][1]["status_sequence"][0])
    assert node_harness.publisher.sent[-1][0] == "xsens_status"
    assert second == first + 1


def test_tick_uses_one_now_and_checks_backlog_stale_before_each_accept(
    node_harness,
):
    node_harness.core.calls.clear()
    node_harness.receiver.pending.extend(
        [
            ReceivedDatagram(
                build_mxtp02_packet(sample_counter=index),
                timestamp,
                ("127.0.0.1", 4000),
            )
            for index, timestamp in ((1, 900_000_000), (2, 950_000_000))
        ]
    )
    node_harness.node._tick()
    assert node_harness.core.calls == [
        ("expire", 1_000_000_000),
        ("stale", 900_000_000),
        ("accept", 900_000_000),
        ("stale", 950_000_000),
        ("accept", 950_000_000),
        ("stale", 1_000_000_000),
    ]


def test_epoch_status_precedes_first_new_epoch_pose(node_harness):
    node_harness.publisher.sent.clear()
    node_harness.core.accept_results = [
        AcceptResult(True, AcceptClassification.EPOCH_COMMITTED, True)
    ]
    node_harness.receiver.pending.append(
        ReceivedDatagram(
            build_mxtp02_packet(), 1, ("127.0.0.1", 4000)
        )
    )
    node_harness.node._tick()
    assert [topic for topic, _ in node_harness.publisher.sent[:2]] == [
        "xsens_status",
        "pose",
    ]


def test_status_send_failure_suppresses_pose_until_status_succeeds(
    node_harness,
):
    node_harness.publisher.sent.clear()
    node_harness.publisher.fail_next_topic = "xsens_status"
    node_harness.core.accept_results = [
        AcceptResult(True, AcceptClassification.EPOCH_COMMITTED, True)
    ]
    node_harness.receiver.pending.append(
        ReceivedDatagram(
            build_mxtp02_packet(), 1, ("127.0.0.1", 4000)
        )
    )
    node_harness.node._tick()
    assert "pose" not in [topic for topic, _ in node_harness.publisher.sent]

    node_harness.node._tick()
    assert [topic for topic, _ in node_harness.publisher.sent[-2:]] == [
        "xsens_status",
        "pose",
    ]


def test_failed_receipt_status_keeps_existing_pose_behind_barrier(
    node_harness,
):
    node_harness.publisher.sent.clear()
    node_harness.node._pose_pending = True
    node_harness.publisher.fail_next_topic = "xsens_status"
    message = make_arm_message(32, node_harness.core.source_epoch, 0)
    node_harness.node._arm_command_callback(message)
    assert node_harness.publisher.sent == []

    node_harness.node._tick()
    assert [topic for topic, _ in node_harness.publisher.sent] == [
        "xsens_status",
        "pose",
    ]


def test_pose_send_failure_retries_same_window_after_next_status(node_harness):
    node_harness.publisher.sent.clear()
    node_harness.publisher.fail_next_topic = "pose"
    node_harness.receiver.pending.append(
        ReceivedDatagram(
            build_mxtp02_packet(), 1, ("127.0.0.1", 4000)
        )
    )
    node_harness.node._tick()
    assert node_harness.publisher.attempted[-2:] == ["xsens_status", "pose"]
    assert [topic for topic, _ in node_harness.publisher.sent] == [
        "xsens_status"
    ]
    node_harness.node._tick()
    assert [topic for topic, _ in node_harness.publisher.sent[-2:]] == [
        "xsens_status",
        "pose",
    ]


def test_malformed_packet_isolated_from_following_valid_packet(node_harness):
    node_harness.publisher.sent.clear()
    node_harness.receiver.pending.extend(
        [
            ReceivedDatagram(b"bad", 1, ("127.0.0.1", 4000)),
            ReceivedDatagram(
                build_mxtp02_packet(), 2, ("127.0.0.1", 4000)
            ),
        ]
    )
    node_harness.node._tick()
    assert node_harness.core.invalid_inputs == 1
    assert [call[0] for call in node_harness.core.calls].count("accept") == 1
    assert [topic for topic, _ in node_harness.publisher.sent] == [
        "xsens_status",
        "pose",
    ]


def test_drain_failure_does_not_stop_status_heartbeat(node_harness):
    node_harness.publisher.sent.clear()
    node_harness.receiver.drain_error = RuntimeError("poll failed")
    node_harness.node._tick()
    assert [topic for topic, _ in node_harness.publisher.sent] == [
        "xsens_status"
    ]
    node_harness.node._tick()
    assert [topic for topic, _ in node_harness.publisher.sent] == [
        "xsens_status",
        "xsens_status",
    ]


def test_conversion_failure_is_transactional_and_publishes_no_pose(
    node_harness,
):
    node_harness.core.accept_results = [
        AcceptResult(False, AcceptClassification.CONVERSION_REJECTED)
    ]
    node_harness.core.pose_fields = None
    node_harness.receiver.pending.append(
        ReceivedDatagram(
            build_mxtp02_packet(), 1, ("127.0.0.1", 4000)
        )
    )
    before = sum(t == "pose" for t, _ in node_harness.publisher.sent)
    node_harness.node._tick()
    after = sum(t == "pose" for t, _ in node_harness.publisher.sent)
    assert after == before
    assert node_harness.publisher.sent[-1][0] == "xsens_status"


def test_final_stale_edge_suppresses_pose_from_accepted_backlog(node_harness):
    node_harness.publisher.sent.clear()
    node_harness.receiver.pending.append(
        ReceivedDatagram(
            build_mxtp02_packet(), 900_000_000, ("127.0.0.1", 4000)
        )
    )
    original_check_stale = node_harness.core.check_stale

    def check_stale(now_ns):
        original_check_stale(now_ns)
        return now_ns == node_harness.clock.now_ns

    node_harness.core.check_stale = check_stale
    node_harness.node._tick()
    assert [topic for topic, _ in node_harness.publisher.sent] == [
        "xsens_status"
    ]


def test_tick_drains_in_order_and_publishes_only_newest_progressing_window(
    integrated_node_harness,
):
    harness = integrated_node_harness
    start_ns = harness.clock.now_ns
    source_times = [
        start_ns + ((index + 1) * 1_000_000_000) // 60
        for index in range(60)
    ]
    queued = 0
    for tick in range(1, 51):
        cutoff_ns = start_ns + tick * 20_000_000
        while queued < len(source_times) and source_times[queued] <= cutoff_ns:
            counter = queued + 1
            harness.receiver.pending.append(
                ReceivedDatagram(
                    payload=build_mxtp02_packet(sample_counter=counter),
                    receive_timestamp_ns=source_times[queued],
                    sender_address=("127.0.0.1", 4000),
                )
            )
            queued += 1
        harness.clock.now_ns = cutoff_ns
        harness.node._tick()
    newest = [
        int(fields["frame_index"][-1])
        for topic, fields in harness.publisher.sent
        if topic == "pose"
    ]
    assert newest
    assert all(right > left for left, right in zip(newest, newest[1:]))
    assert newest[-1] == 60
    assert len(newest) <= 50


def test_malformed_packet_between_candidates_does_not_advance_candidate(
    integrated_node_harness,
):
    harness = integrated_node_harness
    harness.feed(counter=1000, timestamp_ns=1_000_000_000)
    harness.feed(counter=900, timestamp_ns=1_600_000_000)
    harness.receiver.pending.append(
        ReceivedDatagram(
            payload=b"bad",
            receive_timestamp_ns=1_610_000_000,
            sender_address=("127.0.0.1", 4000),
        )
    )
    harness.node._tick()
    assert harness.core.candidate_frame_count == 1
    harness.feed(counter=901, timestamp_ns=1_620_000_000)
    assert harness.core.source_epoch == 72


def test_bind_failure_rolls_back_receiver(rclpy_runtime, xsens_source_context):
    receiver = FakeReceiver()
    with pytest.raises(RuntimeError, match="bind failed"):
        XsensSourceNode(
            xsens_source_context,
            receiver_factory=lambda **kwargs: receiver,
            publisher_factory=lambda *args, **kwargs: (_ for _ in ()).throw(
                RuntimeError("bind failed")
            ),
        )
    assert receiver.closed


def test_resources_are_constructed_before_startup_status_in_contract_order(
    monkeypatch, rclpy_runtime, xsens_source_context
):
    events = []
    receiver = FakeReceiver(events)
    publisher = FakePublisher(events)
    core = FakeCore()

    def receiver_factory(**kwargs):
        events.append("receiver")
        return receiver

    def publisher_factory(*args, **kwargs):
        events.append("publisher")
        original_send = publisher.send

        def send(topic, fields):
            events.append("status")
            return original_send(topic, fields)

        publisher.send = send
        return publisher

    monkeypatch.setattr(
        XsensSourceNode,
        "create_subscription",
        lambda self, *args, **kwargs: events.append("subscription")
        or object(),
    )
    monkeypatch.setattr(
        XsensSourceNode,
        "create_timer",
        lambda self, *args, **kwargs: events.append("timer") or object(),
    )
    monkeypatch.setattr(
        XsensSourceNode,
        "destroy_subscription",
        lambda self, resource: True,
    )
    monkeypatch.setattr(
        XsensSourceNode, "destroy_timer", lambda self, resource: True
    )
    node = XsensSourceNode(
        xsens_source_context,
        receiver_factory=receiver_factory,
        core_factory=lambda *args, **kwargs: core,
        publisher_factory=publisher_factory,
    )
    try:
        assert events[:5] == [
            "receiver",
            "publisher",
            "subscription",
            "timer",
            "status",
        ]
    finally:
        node.destroy_node()


def test_destroy_node_reverses_resources_and_is_idempotent(node_harness):
    events = []
    node = node_harness.node
    node_harness.receiver.events = events
    node_harness.publisher.events = events
    original_destroy_timer = node.destroy_timer
    original_destroy_subscription = node.destroy_subscription

    def destroy_timer(timer):
        events.append("timer")
        return original_destroy_timer(timer)

    def destroy_subscription(subscription):
        events.append("subscription")
        return original_destroy_subscription(subscription)

    node.destroy_timer = destroy_timer
    node.destroy_subscription = destroy_subscription
    first = node.destroy_node()
    second = node.destroy_node()
    assert second == first
    assert events == ["timer", "subscription", "publisher", "receiver"]
    assert node_harness.receiver.closed and node_harness.publisher.closed


def test_repeated_source_lifecycle_uses_reserved_nonproduction_ports(
    rclpy_runtime,
):
    udp_port, pose_port = reserve_udp_port(), reserve_tcp_port()
    for suffix in ("a", "b"):
        node = XsensSourceNode(
            source_context(udp_port, pose_port, f"xsens_{suffix}")
        )
        node.destroy_node()


class CaptureLogger:
    def __init__(self):
        self.events = []

    def info(self, message):
        self.events.append(("info", str(message)))

    def warning(self, message):
        self.events.append(("warning", str(message)))

    def error(self, message):
        self.events.append(("error", str(message)))


def test_diagnostics_are_transition_or_summary_scoped_without_pose_arrays(
    node_harness,
):
    logger = CaptureLogger()
    node_harness.node.get_logger = lambda: logger
    node_harness.node._last_diagnostic_summary_ns = node_harness.clock.now_ns
    for _ in range(100):
        node_harness.clock.advance_ns(20_000_000)
        node_harness.node._tick()
    transition_count = len(logger.events)
    assert transition_count <= 1
    node_harness.clock.advance_ns(5_000_000_000)
    node_harness.node._tick()
    assert len(logger.events) == transition_count + 1
    text = "\n".join(message for _, message in logger.events)
    assert "term1_local" not in text
    assert "smpl_joints" not in text
    assert "array(" not in text


def test_diagnostics_include_unsent_status_age_and_transport_malformed_count(
    node_harness,
):
    logger = CaptureLogger()
    node_harness.node.get_logger = lambda: logger
    node_harness.receiver.stats.invalid_size = 2
    node_harness.node._last_diagnostic_summary_ns = node_harness.clock.now_ns
    node_harness.clock.advance_ns(5_000_000_000)
    node_harness.publisher.fail_next_topic = "xsens_status"
    node_harness.node._tick()
    text = "\n".join(message for _, message in logger.events)
    assert "status_age_ms=5000.0" in text
    assert "malformed=2" in text


def test_create_node_uses_context(rclpy_runtime, xsens_source_context):
    node = create_node(xsens_source_context)
    try:
        assert isinstance(node, XsensSourceNode)
    finally:
        node.destroy_node()
