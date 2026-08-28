from collections.abc import Mapping
from itertools import count
from pathlib import Path

import numpy as np
import pytest
import rclpy

from bxi_example_py_elf3.framework.mod_api import NodeBuildContext
from pico.pose_to_smpl_ref_bridge import (
    SmplRefBridgeNode,
    StreamedSmplRefMerger,
    _build_authoritative_xsens_smpl_ref,
    _validate_xsens_status_fields,
    _validated_bridge_params,
)
from pico.zmq_messages import pack_pose_message
from xsens.source_core import XsensReason
from xsens_test_helpers import (
    FakeClock,
    make_core,
    make_status,
    make_xsens_pose_fields,
    reserve_tcp_port,
)


MOD_ROOT = Path(__file__).resolve().parents[1] / "mods" / "com.bxi.sonic"
PRODUCTION_PORTS = {5556, 5557, 5559, 9763}
POSE_KEYS = {
    "frame_index", "smpl_joints", "body_quat_w", "joint_pos",
    "stream_mode", "calibration_ready", "producer_monotonic_ns",
    "source_epoch",
}
REFERENCE_KEYS = {
    "term1_local", "root_quat", "wrist", "frame_index",
    "source_ready", "producer_monotonic_ns", "source_epoch",
    "source_newest_frame_index",
}
STATUS_KEYS = {
    "status_sequence", "status_monotonic_ns", "source_epoch",
    "last_arm_command_id", "last_arm_target_epoch",
    "last_requested_arm_epoch", "accepted_arm_epoch",
    "producer_monotonic_ns", "newest_frame_index", "ready",
    "reference_window_ready", "source_stale", "ready_frames",
    "recovery_frames", "reason_code",
}
POSE_SCHEMA = {
    "frame_index": (np.dtype(np.int64), (10,)),
    "smpl_joints": (np.dtype(np.float32), (10, 24, 3)),
    "body_quat_w": (np.dtype(np.float32), (10, 4)),
    "joint_pos": (np.dtype(np.float32), (10, 29)),
    "stream_mode": (np.dtype(np.int32), (1,)),
    "calibration_ready": (np.dtype(np.bool_), (1,)),
    "producer_monotonic_ns": (np.dtype(np.int64), (1,)),
    "source_epoch": (np.dtype(np.int64), (1,)),
}
STATUS_SCHEMA = {
    "status_sequence": (np.dtype(np.int64), (1,)),
    "status_monotonic_ns": (np.dtype(np.int64), (1,)),
    "source_epoch": (np.dtype(np.int64), (1,)),
    "last_arm_command_id": (np.dtype(np.int64), (1,)),
    "last_arm_target_epoch": (np.dtype(np.int64), (1,)),
    "last_requested_arm_epoch": (np.dtype(np.int64), (1,)),
    "accepted_arm_epoch": (np.dtype(np.int64), (1,)),
    "producer_monotonic_ns": (np.dtype(np.int64), (1,)),
    "newest_frame_index": (np.dtype(np.int64), (1,)),
    "ready": (np.dtype(np.bool_), (1,)),
    "reference_window_ready": (np.dtype(np.bool_), (1,)),
    "source_stale": (np.dtype(np.bool_), (1,)),
    "ready_frames": (np.dtype(np.int32), (1,)),
    "recovery_frames": (np.dtype(np.int32), (1,)),
    "reason_code": (np.dtype(np.int32), (1,)),
}


def _wrong_dtype(value):
    if value.dtype == np.dtype(np.bool_):
        return value.astype(np.int32)
    if value.dtype == np.dtype(np.int32):
        return value.astype(np.int64)
    if value.dtype == np.dtype(np.int64):
        return value.astype(np.int32)
    if value.dtype == np.dtype(np.float32):
        return value.astype(np.float64)
    raise AssertionError(f"test has no wrong-dtype mapping for {value.dtype}")


def _status_with(**changes):
    fields = make_status()
    for name, scalar in changes.items():
        dtype, shape = STATUS_SCHEMA[name]
        assert shape == (1,)
        fields[name] = np.array([scalar], dtype=dtype)
    return fields


def test_authoritative_xsens_window_is_converted_one_for_one():
    fields = make_xsens_pose_fields(indices=np.arange(100, 110, dtype=np.int64))
    output = _build_authoritative_xsens_smpl_ref(fields)
    assert set(output) == REFERENCE_KEYS
    assert output["term1_local"].shape == (10, 72)
    assert output["term1_local"].dtype == np.float32
    assert output["root_quat"].shape == (10, 4)
    assert output["root_quat"].dtype == np.float32
    assert output["wrist"].shape == (10, 6)
    assert output["wrist"].dtype == np.float32
    assert output["frame_index"].tolist() == [100]
    assert output["source_newest_frame_index"].tolist() == [109]
    assert output["source_epoch"].tolist() == fields["source_epoch"].tolist()
    assert output["producer_monotonic_ns"].tolist() == fields["producer_monotonic_ns"].tolist()


def test_xsens_false_readiness_is_forwarded_immediately():
    fields = make_xsens_pose_fields(calibration_ready=False)
    output = _build_authoritative_xsens_smpl_ref(fields)
    assert output["source_ready"].tolist() == [False]


@pytest.mark.parametrize(
    "indices",
    [
        np.arange(9, dtype=np.int64),
        np.arange(11, dtype=np.int64),
        np.array([0, 1, 2, 3, 4, 4, 6, 7, 8, 9], dtype=np.int64),
        np.array([0, 1, 2, 3, 4, 6, 5, 7, 8, 9], dtype=np.int64),
    ],
)
def test_authoritative_xsens_rejects_wrong_or_nonprogressing_rows(indices):
    with pytest.raises(ValueError):
        _build_authoritative_xsens_smpl_ref(make_xsens_pose_fields(indices=indices))


def test_authoritative_pose_rejects_int64_extreme_backward_step():
    int64_min = -9_223_372_036_854_775_808
    indices = np.array(
        [1, *range(int64_min, int64_min + 9)], dtype=np.int64
    )
    with pytest.raises(ValueError, match="frame_index"):
        _build_authoritative_xsens_smpl_ref(
            make_xsens_pose_fields(indices=indices)
        )


def test_status_send_failure_blocks_reference_until_barrier_succeeds(bridge_harness):
    bridge_harness.output.fail_next_topic = "xsens_status"
    bridge_harness.input_status(make_status(status_sequence=2, source_epoch=71))
    bridge_harness.input_pose(make_xsens_pose_fields(source_epoch=71))
    bridge_harness.tick()
    assert [topic for topic, _ in bridge_harness.output.attempts] == ["xsens_status"]
    assert bridge_harness.output.topics == []
    bridge_harness.tick()
    assert bridge_harness.output.topics == ["xsens_status", "smpl_ref"]
    assert [topic for topic, _ in bridge_harness.output.attempts] == [
        "xsens_status", "xsens_status", "smpl_ref",
    ]


def test_reference_waits_until_forwarded_status_covers_pose_frame(bridge_harness):
    bridge_harness.input_status(make_status(
        status_sequence=2, source_epoch=71, newest_frame_index=108,
    ))
    bridge_harness.input_pose(make_xsens_pose_fields(source_epoch=71))
    bridge_harness.tick()
    assert bridge_harness.output.topics == ["xsens_status"]

    bridge_harness.input_status(make_status(
        status_sequence=3, source_epoch=71, newest_frame_index=109,
    ))
    bridge_harness.tick()
    assert bridge_harness.output.topics == [
        "xsens_status", "xsens_status", "smpl_ref",
    ]


_NODE_IDS = count()


def _reserved_nonproduction_tcp_port(*, exclude=()):
    excluded = PRODUCTION_PORTS | set(exclude)
    while True:
        port = reserve_tcp_port()
        if port not in excluded:
            return port


@pytest.fixture
def rclpy_runtime():
    started_here = not rclpy.ok()
    if started_here:
        rclpy.init(args=[])
    yield
    if started_here and rclpy.ok():
        rclpy.shutdown()


class FakePackedInput:
    def __init__(self):
        self.pending = []
        self.closed = False

    def drain(self):
        batch, self.pending = tuple(self.pending), []
        return batch

    def close(self):
        self.closed = True


class FakePackedOutput:
    def __init__(self):
        self.fail_next_topic = None
        self.attempts = []
        self.sent = []
        self.topics = []
        self.fields = []
        self.closed = False

    def send(self, topic: str, fields: Mapping[str, np.ndarray]) -> bool:
        snapshot = {
            name: np.array(value, copy=True)
            for name, value in fields.items()
        }
        self.attempts.append((topic, snapshot))
        if self.fail_next_topic == topic:
            self.fail_next_topic = None
            return False
        self.sent.append((topic, snapshot))
        self.topics.append(topic)
        self.fields.append(snapshot)
        return True

    def close(self):
        self.closed = True


def bridge_context(*, source_kind, input_port, output_port, node_name):
    params = {
        "pico_host": "127.0.0.1", "pico_port": input_port,
        "out_host": "127.0.0.1", "out_port": output_port,
        "input_pose_topic": "pose", "input_status_topic": "xsens_status",
        "output_reference_topic": "smpl_ref", "output_status_topic": "xsens_status",
        "source_kind": source_kind,
        "authoritative_input_window": source_kind == "xsens",
        "readiness_debounce_messages": 1 if source_kind == "xsens" else 3,
        "rate_hz": 50.0, "history_frames": 5, "max_gap_frames": 200,
        "catch_up_enabled": True, "stale_warning_seconds": 0.5,
    }
    return NodeBuildContext(
        mod_id="com.bxi.sonic",
        node_id=f"com.bxi.sonic/{node_name}",
        node_name=node_name,
        mod_root=MOD_ROOT,
        params=params,
    )


class BridgeHarness:
    def __init__(self, *, source_kind="xsens"):
        input_port = _reserved_nonproduction_tcp_port()
        output_port = _reserved_nonproduction_tcp_port(exclude={input_port})
        self.clock = FakeClock(1_000_000_000)
        self.input = FakePackedInput()
        self.output = FakePackedOutput()
        node_name = f"bridge_{source_kind}_{next(_NODE_IDS)}"
        self.node = SmplRefBridgeNode(
            bridge_context(
                source_kind=source_kind,
                input_port=input_port,
                output_port=output_port,
                node_name=node_name,
            ),
            monotonic=self.clock.monotonic,
            monotonic_ns=self.clock.monotonic_ns,
            input_transport=self.input,
            output_transport=self.output,
        )
        self.closed = False

    def input_status(self, fields):
        self.input.pending.append(pack_pose_message(fields, topic="xsens_status"))

    def input_pose(self, fields):
        self.input.pending.append(pack_pose_message(fields, topic="pose"))

    def input_raw(self, message):
        self.input.pending.append(message)

    def tick(self):
        self.node._tick()

    def close(self):
        if not self.closed:
            self.closed = True
            self.node.destroy_node()


@pytest.fixture
def bridge_harness(rclpy_runtime):
    harness = BridgeHarness()
    yield harness
    harness.close()


def test_authoritative_window_never_fills_merges_or_clamps_rows():
    fields = make_xsens_pose_fields(indices=np.arange(100, 110, dtype=np.int64))
    fields["smpl_joints"][:, 0, 0] = np.arange(10, dtype=np.float32)
    output = _build_authoritative_xsens_smpl_ref(fields)
    assert set(output) == REFERENCE_KEYS
    np.testing.assert_array_equal(
        output["term1_local"][:, 0], np.arange(10, dtype=np.float32)
    )
    assert output["frame_index"].tolist() == [100]
    assert output["source_newest_frame_index"].tolist() == [109]


def test_xsens_status_forwards_without_pose_and_preserves_all_fields(bridge_harness):
    status = make_status(status_sequence=2, source_epoch=71)
    bridge_harness.input_status(status)
    bridge_harness.tick()
    assert bridge_harness.output.topics == ["xsens_status"]
    topic, forwarded = bridge_harness.output.sent[0]
    assert topic == "xsens_status"
    assert set(forwarded) == STATUS_KEYS
    for name, value in status.items():
        np.testing.assert_array_equal(forwarded[name], value)


def test_real_source_startup_and_heartbeat_status_forward_fail_closed(
    bridge_harness,
):
    core, clock = make_core(epoch_draws=(91, 92), now_ns=1_000_000_000)

    startup = core.build_status_fields(clock.now_ns)
    bridge_harness.input_status(startup)
    bridge_harness.tick()
    clock.advance_ns(20_000_000)
    heartbeat = core.build_status_fields(clock.now_ns)
    bridge_harness.input_status(heartbeat)
    bridge_harness.tick()

    assert bridge_harness.output.topics == ["xsens_status", "xsens_status"]
    assert core.build_pose_fields() is None
    for expected, (topic, forwarded) in zip(
        (startup, heartbeat), bridge_harness.output.sent
    ):
        assert topic == "xsens_status"
        assert int(forwarded["source_epoch"][0]) == 91
        assert int(forwarded["producer_monotonic_ns"][0]) == 0
        assert int(forwarded["newest_frame_index"][0]) == -1
        assert not bool(forwarded["ready"][0])
        assert not bool(forwarded["reference_window_ready"][0])
        assert not bool(forwarded["source_stale"][0])
        assert int(forwarded["accepted_arm_epoch"][0]) == 0
        assert int(forwarded["reason_code"][0]) == XsensReason.NO_DATA
        for name, value in expected.items():
            np.testing.assert_array_equal(forwarded[name], value)
    assert "smpl_ref" not in bridge_harness.output.topics


def test_status_only_stale_edge_clears_pending_pose_immediately(bridge_harness):
    bridge_harness.output.fail_next_topic = "xsens_status"
    bridge_harness.input_status(make_status(status_sequence=2, source_epoch=71))
    bridge_harness.input_pose(make_xsens_pose_fields(source_epoch=71))
    bridge_harness.tick()
    bridge_harness.input_status(make_status(
        status_sequence=3, source_epoch=71, source_stale=True,
        reference_window_ready=False, ready=False, ready_frames=0,
        recovery_frames=0, reason_code=XsensReason.ARMED_STALE,
    ))
    bridge_harness.tick()
    bridge_harness.tick()
    assert "smpl_ref" not in [
        topic for topic, _ in bridge_harness.output.attempts
    ]


def test_status_only_epoch_change_clears_old_epoch_pose_before_tick(bridge_harness):
    bridge_harness.output.fail_next_topic = "xsens_status"
    bridge_harness.input_status(make_status(status_sequence=2, source_epoch=71))
    bridge_harness.input_pose(make_xsens_pose_fields(source_epoch=71))
    bridge_harness.tick()
    bridge_harness.input_status(make_status(
        status_sequence=3, source_epoch=72,
        producer_monotonic_ns=bridge_harness.clock.now_ns,
        newest_frame_index=209, ready=False, reference_window_ready=False,
        ready_frames=10, reason_code=XsensReason.COLLECTING_STABILITY,
    ))
    bridge_harness.tick()
    bridge_harness.tick()
    assert "smpl_ref" not in [
        topic for topic, _ in bridge_harness.output.attempts
    ]


def test_producer_timestamp_republication_never_extends_freshness(bridge_harness):
    produced = bridge_harness.clock.now_ns
    bridge_harness.clock.advance_ns(500_000_000)
    bridge_harness.input_status(make_status(
        status_sequence=1,
        status_monotonic_ns=bridge_harness.clock.now_ns,
        producer_monotonic_ns=produced,
        newest_frame_index=109,
    ))
    bridge_harness.input_pose(make_xsens_pose_fields(
        indices=np.arange(100, 110, dtype=np.int64),
        producer_monotonic_ns=produced,
    ))
    bridge_harness.tick()
    assert bridge_harness.output.topics == ["xsens_status", "smpl_ref"]

    bridge_harness.clock.advance_ns(1)
    bridge_harness.input_status(make_status(
        status_sequence=2,
        status_monotonic_ns=bridge_harness.clock.now_ns,
        producer_monotonic_ns=produced,
        newest_frame_index=119,
    ))
    bridge_harness.input_pose(make_xsens_pose_fields(
        indices=np.arange(110, 120, dtype=np.int64),
        producer_monotonic_ns=produced,
    ))
    bridge_harness.tick()
    assert bridge_harness.output.topics == [
        "xsens_status", "smpl_ref", "xsens_status",
    ]
    references = [fields for topic, fields in bridge_harness.output.sent
                  if topic == "smpl_ref"]
    assert len(references) == 1
    assert references[0]["source_newest_frame_index"].tolist() == [109]


def test_exact_topic_matching_rejects_pose_suffix(bridge_harness):
    bridge_harness.input_status(make_status(status_sequence=2, source_epoch=71))
    bridge_harness.input_raw(pack_pose_message(
        make_xsens_pose_fields(source_epoch=71), topic="pose_suffix",
    ))
    bridge_harness.tick()
    assert bridge_harness.output.topics == ["xsens_status"]
    assert "smpl_ref" not in [
        topic for topic, _ in bridge_harness.output.attempts
    ]


def test_optional_metadata_does_not_select_xsens_without_source_kind(rclpy_runtime):
    harness = BridgeHarness(source_kind="legacy")
    try:
        assert harness.node._source_kind == "legacy"
        for start in (1, 2, 3):
            harness.input_pose(make_xsens_pose_fields(
                indices=np.arange(start, start + 10, dtype=np.int64)
            ))
            harness.tick()
            assert harness.node._pending_xsens_pose is None
            if start < 3:
                assert "smpl_ref" not in harness.output.topics
        assert harness.output.topics == ["smpl_ref"]
    finally:
        harness.close()


def test_legacy_alias_only_and_canonical_only_params_resolve():
    alias = _validated_bridge_params({"pico_topic": "body", "out_topic": "ref"})
    canonical = _validated_bridge_params({
        "input_pose_topic": "body", "output_reference_topic": "ref"
    })
    assert alias["input_pose_topic"] == canonical["input_pose_topic"] == "body"
    assert alias["output_reference_topic"] == canonical["output_reference_topic"] == "ref"
    assert "pico_topic" not in alias and "out_topic" not in alias


def test_equal_aliases_pass_and_conflicting_aliases_fail():
    params = _validated_bridge_params({
        "pico_topic": "pose", "input_pose_topic": "pose",
        "out_topic": "smpl_ref", "output_reference_topic": "smpl_ref",
    })
    assert params["input_pose_topic"] == "pose"
    with pytest.raises(ValueError, match="conflicting"):
        _validated_bridge_params({"pico_topic": "a", "input_pose_topic": "b"})


def test_unknown_raw_bridge_param_is_rejected_before_defaults():
    with pytest.raises(ValueError, match="unknown bridge params"):
        _validated_bridge_params({"unknown_bridge_key": 1})


@pytest.mark.parametrize(
    "params",
    [
        {"source_kind": "xsens", "authoritative_input_window": False,
         "readiness_debounce_messages": 1},
        {"source_kind": "xsens", "authoritative_input_window": True,
         "readiness_debounce_messages": 2},
    ],
)
def test_xsens_requires_authoritative_true_and_debounce_one(params):
    with pytest.raises(ValueError):
        _validated_bridge_params(params)


def test_legacy_bridge_keeps_merger_and_three_message_debounce(rclpy_runtime):
    params = _validated_bridge_params({"source_kind": "legacy"})
    assert params["authoritative_input_window"] is False
    assert params["readiness_debounce_messages"] == 3
    harness = BridgeHarness(source_kind="legacy")
    try:
        assert isinstance(harness.node._merger, StreamedSmplRefMerger)
        for start in (10, 11):
            harness.input_pose(make_xsens_pose_fields(
                indices=np.arange(start, start + 10, dtype=np.int64),
            ))
            harness.tick()
        assert harness.output.topics == []
        harness.input_pose(make_xsens_pose_fields(
            indices=np.arange(12, 22, dtype=np.int64),
        ))
        harness.tick()
        assert harness.output.topics == ["smpl_ref"]
        assert harness.node._merger.total_merges == 1
    finally:
        harness.close()


def test_injected_transports_are_borrowed_not_closed(rclpy_runtime):
    harness = BridgeHarness()
    harness.close()
    assert harness.input.closed is False
    assert harness.output.closed is False


def test_bridge_lifecycle_uses_reserved_nonproduction_ports(rclpy_runtime):
    input_port = _reserved_nonproduction_tcp_port()
    output_port = _reserved_nonproduction_tcp_port(exclude={input_port})
    assert input_port != output_port
    assert {input_port, output_port}.isdisjoint(PRODUCTION_PORTS)
    clock = FakeClock(1_000_000_000)
    for suffix in ("a", "b"):
        node = SmplRefBridgeNode(
            bridge_context(
                source_kind="xsens",
                input_port=input_port,
                output_port=output_port,
                node_name=f"xsens_bridge_lifecycle_{suffix}",
            ),
            monotonic=clock.monotonic,
            monotonic_ns=clock.monotonic_ns,
        )
        node.destroy_node()


@pytest.mark.parametrize("field", sorted(POSE_KEYS))
def test_authoritative_pose_rejects_wrong_dtype_for_every_field(field):
    fields = make_xsens_pose_fields()
    fields[field] = _wrong_dtype(fields[field])
    with pytest.raises(ValueError, match=field):
        _build_authoritative_xsens_smpl_ref(fields)


@pytest.mark.parametrize(
    "field", ["frame_index", "smpl_joints", "body_quat_w", "joint_pos"],
)
def test_authoritative_pose_rejects_wrong_shape_for_every_row_field(field):
    fields = make_xsens_pose_fields()
    fields[field] = np.expand_dims(fields[field], axis=-1)
    with pytest.raises(ValueError, match=field):
        _build_authoritative_xsens_smpl_ref(fields)


@pytest.mark.parametrize(
    "field",
    ["stream_mode", "calibration_ready", "producer_monotonic_ns", "source_epoch"],
)
def test_authoritative_pose_rejects_wrong_metadata_scalar_shape(field):
    fields = make_xsens_pose_fields()
    fields[field] = np.repeat(fields[field], 2)
    with pytest.raises(ValueError, match=field):
        _build_authoritative_xsens_smpl_ref(fields)


@pytest.mark.parametrize(
    "field,index,bad_value",
    [
        ("smpl_joints", (0, 0, 0), np.nan),
        ("smpl_joints", (0, 0, 0), np.inf),
        ("body_quat_w", (0, 0), np.nan),
        ("body_quat_w", (0, 0), np.inf),
        ("joint_pos", (0, 0), np.nan),
        ("joint_pos", (0, 0), np.inf),
    ],
)
def test_authoritative_pose_rejects_nan_and_inf(field, index, bad_value):
    fields = make_xsens_pose_fields()
    fields[field][index] = bad_value
    with pytest.raises(ValueError, match=field):
        _build_authoritative_xsens_smpl_ref(fields)


def test_authoritative_pose_rejects_zero_root_quaternion():
    fields = make_xsens_pose_fields()
    fields["body_quat_w"][4] = 0.0
    with pytest.raises(ValueError, match="body_quat_w"):
        _build_authoritative_xsens_smpl_ref(fields)


@pytest.mark.parametrize(
    "field,value",
    [
        ("stream_mode", 0),
        ("stream_mode", 2),
        ("producer_monotonic_ns", 0),
        ("producer_monotonic_ns", -1),
        ("source_epoch", 0),
        ("source_epoch", -1),
    ],
)
def test_authoritative_pose_rejects_invalid_scalar_semantics(field, value):
    fields = make_xsens_pose_fields()
    dtype, _ = POSE_SCHEMA[field]
    fields[field] = np.array([value], dtype=dtype)
    with pytest.raises(ValueError, match=field):
        _build_authoritative_xsens_smpl_ref(fields)


def test_authoritative_pose_rejects_missing_field():
    fields = make_xsens_pose_fields()
    fields.pop("joint_pos")
    with pytest.raises(ValueError, match="joint_pos"):
        _build_authoritative_xsens_smpl_ref(fields)


def test_authoritative_pose_rejects_extra_field():
    fields = make_xsens_pose_fields()
    fields["unexpected"] = np.array([1], dtype=np.int64)
    with pytest.raises(ValueError, match="unexpected"):
        _build_authoritative_xsens_smpl_ref(fields)


@pytest.mark.parametrize("field", sorted(STATUS_KEYS))
def test_xsens_status_rejects_wrong_dtype_for_every_field(field):
    fields = make_status()
    fields[field] = _wrong_dtype(fields[field])
    with pytest.raises(ValueError, match=field):
        _validate_xsens_status_fields(fields)


@pytest.mark.parametrize("field", sorted(STATUS_KEYS))
def test_xsens_status_rejects_wrong_shape_for_every_field(field):
    fields = make_status()
    fields[field] = np.repeat(fields[field], 2)
    with pytest.raises(ValueError, match=field):
        _validate_xsens_status_fields(fields)


def test_xsens_status_rejects_missing_field():
    fields = make_status()
    fields.pop("status_monotonic_ns")
    with pytest.raises(ValueError, match="status_monotonic_ns"):
        _validate_xsens_status_fields(fields)


def test_xsens_status_rejects_extra_field():
    fields = make_status()
    fields["unexpected"] = np.array([1], dtype=np.int64)
    with pytest.raises(ValueError, match="unexpected"):
        _validate_xsens_status_fields(fields)


@pytest.mark.parametrize(
    "changes",
    [
        pytest.param({"status_sequence": 0}, id="zero-status-sequence"),
        pytest.param({"status_monotonic_ns": -1}, id="negative-status-time"),
        pytest.param({"source_epoch": -1}, id="negative-source-epoch"),
        pytest.param({"source_epoch": 0}, id="zero-epoch-with-live-data"),
        pytest.param({"last_arm_command_id": -1}, id="negative-command-id"),
        pytest.param({"last_arm_target_epoch": -1}, id="negative-target"),
        pytest.param({"last_requested_arm_epoch": -1}, id="negative-request"),
        pytest.param({"last_arm_target_epoch": 71}, id="target-without-command"),
        pytest.param({"last_requested_arm_epoch": 71}, id="request-without-command"),
        pytest.param(
            {"last_arm_command_id": 7}, id="command-without-positive-target",
        ),
        pytest.param(
            {
                "last_arm_command_id": 7,
                "last_arm_target_epoch": 71,
                "last_requested_arm_epoch": 72,
            },
            id="request-neither-zero-nor-target",
        ),
        pytest.param({"accepted_arm_epoch": -1}, id="negative-authorization"),
        pytest.param({"accepted_arm_epoch": 71}, id="authorization-without-receipt"),
        pytest.param(
            {
                "last_arm_command_id": 7,
                "last_arm_target_epoch": 71,
                "last_requested_arm_epoch": 71,
                "accepted_arm_epoch": 72,
            },
            id="authorization-not-current-source-epoch",
        ),
    ],
)
def test_xsens_status_rejects_invalid_epoch_receipt_or_authorization_relationships(
    changes,
):
    with pytest.raises(ValueError):
        _validate_xsens_status_fields(_status_with(**changes))


@pytest.mark.parametrize(
    "changes",
    [
        pytest.param({"producer_monotonic_ns": -1}, id="negative-producer-time"),
        pytest.param(
            {"producer_monotonic_ns": 0}, id="zero-producer-with-frame",
        ),
        pytest.param(
            {"newest_frame_index": -1}, id="producer-with-sentinel-frame",
        ),
        pytest.param({"newest_frame_index": -2}, id="frame-below-sentinel"),
    ],
)
def test_xsens_status_rejects_producer_sentinel_mismatch_or_bad_index(changes):
    with pytest.raises(ValueError):
        _validate_xsens_status_fields(_status_with(**changes))


def test_xsens_status_accepts_positive_epoch_exact_no_data_sentinels():
    fields = _status_with(
        source_epoch=91,
        producer_monotonic_ns=0,
        newest_frame_index=-1,
        ready=False,
        reference_window_ready=False,
        source_stale=False,
        ready_frames=0,
        recovery_frames=0,
        accepted_arm_epoch=0,
        reason_code=XsensReason.NO_DATA,
    )
    validated = _validate_xsens_status_fields(fields)
    assert set(validated) == STATUS_KEYS
    for name, value in fields.items():
        np.testing.assert_array_equal(validated[name], value)


@pytest.mark.parametrize(
    "changes",
    [
        pytest.param({"source_epoch": 0}, id="missing-process-epoch"),
        pytest.param({"ready": True}, id="ready"),
        pytest.param(
            {"reference_window_ready": True}, id="reference-window-ready",
        ),
        pytest.param({"source_stale": True}, id="stale-without-data"),
        pytest.param({"ready_frames": 1}, id="readiness-progress"),
        pytest.param({"recovery_frames": 1}, id="recovery-progress"),
        pytest.param(
            {
                "last_arm_command_id": 7,
                "last_arm_target_epoch": 91,
                "last_requested_arm_epoch": 91,
                "accepted_arm_epoch": 91,
            },
            id="accepted-authorization",
        ),
    ],
)
def test_xsens_status_rejects_malformed_positive_epoch_no_data_states(changes):
    fields = _status_with(
        source_epoch=91,
        producer_monotonic_ns=0,
        newest_frame_index=-1,
        ready=False,
        reference_window_ready=False,
        source_stale=False,
        ready_frames=0,
        recovery_frames=0,
        accepted_arm_epoch=0,
        reason_code=XsensReason.NO_DATA,
    )
    for name, scalar in changes.items():
        dtype, shape = STATUS_SCHEMA[name]
        assert shape == (1,)
        fields[name] = np.array([scalar], dtype=dtype)
    with pytest.raises(ValueError):
        _validate_xsens_status_fields(fields)


@pytest.mark.parametrize(
    "changes",
    [
        pytest.param({"ready_frames": -1}, id="negative-ready-frames"),
        pytest.param({"ready_frames": 31}, id="too-many-ready-frames"),
        pytest.param({"recovery_frames": -1}, id="negative-recovery-frames"),
        pytest.param({"recovery_frames": 11}, id="too-many-recovery-frames"),
        pytest.param({"reason_code": -1}, id="reason-below-enum"),
        pytest.param({"reason_code": 12}, id="reason-above-enum"),
        pytest.param(
            {"source_stale": True, "ready": True}, id="stale-but-ready",
        ),
        pytest.param(
            {
                "source_stale": True,
                "ready": False,
                "reference_window_ready": True,
            },
            id="stale-but-window-ready",
        ),
        pytest.param(
            {"ready": True, "reference_window_ready": False},
            id="ready-without-complete-window",
        ),
    ],
)
def test_xsens_status_rejects_invalid_readiness_recovery_or_reason_state(changes):
    with pytest.raises(ValueError):
        _validate_xsens_status_fields(_status_with(**changes))
