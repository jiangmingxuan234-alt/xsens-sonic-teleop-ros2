from __future__ import annotations

from collections import deque
from dataclasses import replace
from threading import Event, Lock
from typing import Iterator, Mapping, Sequence

import numpy as np
import pytest

from bxi_example_py_elf3.framework.inference import InferenceFrame
from bxi_example_py_elf3.framework.joints import JointStateView
from pico.zmq_messages import pack_pose_message
from policy import (
    MODEL_INPUT_DIM,
    SMPL_ROOT_ORI_START,
    SMPL_TOKENIZER_DIM,
    ArmGateProof,
    JoinedSourceSnapshot,
    SONIC_PARAMETERS,
    SmplReferenceFrame,
    SonicTeleopPolicy,
)
from xsens.source_core import XsensReason
from xsens_test_helpers import FakeClock, make_status


def make_reference(
    *,
    epoch: int | None = 71,
    newest_frame: int | None = 109,
    source_ready: bool = True,
    producer_monotonic_ns: int | None = 1_000_000_000,
    row_marker: float | Sequence[float] = 0.0,
    root_yaw_rad: float = 0.0,
    with_anchor: bool = True,
) -> SmplReferenceFrame:
    marker = np.asarray(row_marker, dtype=np.float32)
    if marker.ndim == 0:
        marker = np.repeat(marker.reshape(1), 10)
    marker = marker.reshape(10)
    term = np.repeat(marker[:, None], 72, axis=1).astype(np.float32)
    wrist = np.repeat(marker[:, None], 6, axis=1).astype(np.float32)
    root = np.zeros((10, 4), dtype=np.float32)
    root[:, 0] = np.cos(root_yaw_rad / 2.0)
    root[:, 3] = np.sin(root_yaw_rad / 2.0)
    anchor = root.copy() if with_anchor else None
    return SmplReferenceFrame(
        term1_local=term, root_quat=root, wrist=wrist, anchor_quat=anchor,
        frame_index=-1 if newest_frame is None else int(newest_frame) - 9,
        sequence=1,
        source_ready=bool(source_ready),
        producer_monotonic_ns=producer_monotonic_ns,
        source_epoch=epoch,
        source_newest_frame_index=newest_frame,
        received_monotonic=0.0,
    )


def make_inference_frame(qpos_offset: float = 0.0) -> InferenceFrame:
    position = (
        SONIC_PARAMETERS.default_position.astype(np.float32)
        + np.float32(qpos_offset)
    )
    joints = JointStateView(
        SONIC_PARAMETERS.layout,
        position.copy(),
        np.zeros(29, dtype=np.float32),
    )
    return InferenceFrame(
        joints=joints,
        quat_wxyz=np.array([1, 0, 0, 0], dtype=np.float32),
        angular_velocity=np.zeros(3, dtype=np.float32),
    )


class DeterministicBackend:
    def run(self, inputs: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
        observation = np.asarray(inputs["obs_dict"], dtype=np.float32)
        human = float(np.mean(observation[0, :720]))
        root = float(observation[0, SMPL_ROOT_ORI_START + 1])
        proprio = float(np.mean(observation[0, SMPL_TOKENIZER_DIM:]))
        signal = 0.01 * human + 0.25 * root + 0.01 * proprio
        return {
            "action": np.full(
                (1, 29), np.tanh(signal) * np.float32(0.1), dtype=np.float32
            )
        }

    def close(self) -> None:
        pass


class TestableSonicPolicy(SonicTeleopPolicy):
    __test__ = False

    def _load_stream_reference(self):
        rows = 3520
        self.ref_term1 = np.zeros((rows, 72), np.float32)
        self.ref_root_quat = np.zeros((rows, 4), np.float32)
        self.ref_root_quat[:, 0] = 1.0
        self.ref_wrist = np.zeros((rows, 6), np.float32)
        self.ref_anchor_quat = None

    def _init_backend(self, backend: str) -> None:
        self._backend = DeterministicBackend()
        self.input_buffer = np.zeros((1, MODEL_INPUT_DIM), np.float32)
        self._inputs = {"obs_dict": self.input_buffer}

    def _init_zmq(self) -> None:
        self._message_lock = Lock()
        self._reference_messages = deque(maxlen=64)
        self._status_messages = deque(maxlen=64)
        self._zmq_stop = Event()
        self._zmq_thread = None

    def inject_status(
        self,
        fields: Mapping[str, np.ndarray],
        received_mono: float | None = None,
    ) -> None:
        received = (
            self._monotonic() if received_mono is None else received_mono
        )
        message = pack_pose_message(fields, topic=self.xsens_status_zmq_topic)
        with self._message_lock:
            self._status_messages.append((message, float(received)))

    def inject_reference(
        self,
        reference: SmplReferenceFrame,
        received_mono: float | None = None,
    ) -> None:
        received = (
            self._monotonic() if received_mono is None else received_mono
        )
        fields = {
            "term1_local": reference.term1_local,
            "root_quat": reference.root_quat,
            "wrist": reference.wrist,
            "frame_index": np.array([reference.frame_index], np.int64),
            "source_ready": np.array([reference.source_ready], np.bool_),
        }
        if reference.producer_monotonic_ns is not None:
            fields["producer_monotonic_ns"] = np.array(
                [reference.producer_monotonic_ns], np.int64
            )
        if reference.source_epoch is not None:
            fields["source_epoch"] = np.array(
                [reference.source_epoch], np.int64
            )
        if reference.source_newest_frame_index is not None:
            fields["source_newest_frame_index"] = np.array(
                [reference.source_newest_frame_index], np.int64
            )
        if reference.anchor_quat is not None:
            fields["anchor_quat"] = reference.anchor_quat
        message = pack_pose_message(fields, topic=self.smpl_ref_zmq_topic)
        with self._message_lock:
            self._reference_messages.append((message, float(received)))


class CaptureLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.events = []

    def info(self, message):
        self.infos.append(str(message))
        self.events.append(("info", str(message)))

    def warning(self, message):
        self.warnings.append(str(message))
        self.events.append(("warning", str(message)))


class PolicyHarness:
    def __init__(self) -> None:
        self.clock = FakeClock(1_000_000_000)
        self.policy = TestableSonicPolicy(
            "unused.onnx", "unused.npz", use_smpl_ref_zmq=True,
            monotonic=self.clock.monotonic,
            monotonic_ns=self.clock.monotonic_ns,
        )
        self.policy.test_clock = self.clock
        self.policy.bind_logger(CaptureLogger())
        self.policy.configure_runtime(
            yaw_bias_rad=np.pi / 2,
            live_ref_timeout_s=0.5,
            idle_frame_start=3509,
            source_blend_duration_s=0.4,
            source_kind="xsens",
            status_timeout_s=0.2,
        )
        self.policy.reset(make_inference_frame())

    def step(self, *, qpos_offset: float = 0.0) -> np.ndarray:
        self.policy.step(make_inference_frame(qpos_offset), 0.02)
        return self.policy.target_dof_pos.copy()

    def advance_and_step(
        self, delta_ns: int, *, qpos_offset: float = 0.0
    ) -> np.ndarray:
        self.clock.advance_ns(delta_ns)
        return self.step(qpos_offset=qpos_offset)

    def refresh_status_receipt(self) -> None:
        snapshot = self.policy.poll_source_snapshot()
        status = snapshot.status
        if status is None:
            return
        self.policy.inject_status(
            make_status(
                status_sequence=status.status_sequence + 1,
                status_monotonic_ns=self.clock.now_ns,
                source_epoch=status.source_epoch,
                last_arm_command_id=status.last_arm_command_id,
                last_arm_target_epoch=status.last_arm_target_epoch,
                last_requested_arm_epoch=status.last_requested_arm_epoch,
                accepted_arm_epoch=status.accepted_arm_epoch,
                producer_monotonic_ns=status.producer_monotonic_ns,
                newest_frame_index=status.newest_frame_index,
                ready=status.ready,
                reference_window_ready=status.reference_window_ready,
                source_stale=status.source_stale,
                ready_frames=status.ready_frames,
                recovery_frames=status.recovery_frames,
                reason_code=status.reason_code,
            ),
            received_mono=self.clock.monotonic(),
        )

    def finish_blend(self, *, qpos_offset: float = 0.0) -> np.ndarray:
        if not self.policy.source_blend_active:
            return self.step(qpos_offset=qpos_offset)
        end = (
            self.policy.source_blend_started_at
            + self.policy.source_blend_duration_s
        )
        remaining_ns = max(
            0,
            int(np.ceil((end - self.clock.monotonic()) * 1_000_000_000)),
        )
        self.clock.advance_ns(remaining_ns)
        self.refresh_status_receipt()
        return self.step(qpos_offset=qpos_offset)

    def close(self) -> None:
        self.policy.close()


@pytest.fixture
def policy_harness() -> Iterator[PolicyHarness]:
    harness = PolicyHarness()
    yield harness
    harness.close()


@pytest.fixture
def policy(policy_harness: PolicyHarness) -> TestableSonicPolicy:
    return policy_harness.policy


@pytest.fixture
def fake_clock(policy_harness: PolicyHarness) -> FakeClock:
    return policy_harness.clock


def inject_exact_authorized_pair(
    policy: TestableSonicPolicy,
    *,
    epoch: int,
    command_id: int,
    sequence: int,
    newest: int,
    pre_sequence: int | None = None,
    target_epoch: int | None = None,
    requested_epoch: int | None = None,
    accepted_epoch: int | None = None,
    reference: SmplReferenceFrame | None = None,
    producer_ns: int | None = None,
    received_mono: float | None = None,
) -> ArmGateProof:
    produced = policy.test_clock.now_ns if producer_ns is None else producer_ns
    target = epoch if target_epoch is None else target_epoch
    requested = epoch if requested_epoch is None else requested_epoch
    accepted = epoch if accepted_epoch is None else accepted_epoch
    ref = reference or make_reference(
        epoch=epoch, newest_frame=newest, producer_monotonic_ns=produced
    )
    policy.inject_status(make_status(
        status_sequence=sequence, status_monotonic_ns=policy.test_clock.now_ns,
        source_epoch=epoch, last_arm_command_id=command_id,
        last_arm_target_epoch=target, last_requested_arm_epoch=requested,
        accepted_arm_epoch=accepted, producer_monotonic_ns=produced,
        newest_frame_index=newest, ready=True, reference_window_ready=True,
        source_stale=False, ready_frames=30, recovery_frames=10,
        reason_code=XsensReason.ARMED_FRESH,
    ), received_mono=received_mono)
    policy.inject_reference(ref, received_mono=received_mono)
    policy.poll_source_snapshot()
    return ArmGateProof(
        command_id=command_id,
        target_source_epoch=epoch,
        requested_arm_epoch=epoch,
        pre_status_sequence=(
            sequence - 1 if pre_sequence is None else pre_sequence
        ),
    )


def authorize_reference(
    policy: TestableSonicPolicy,
    reference: SmplReferenceFrame,
) -> ArmGateProof:
    assert reference.source_epoch is not None
    assert reference.source_newest_frame_index is not None
    proof = inject_exact_authorized_pair(
        policy, epoch=reference.source_epoch, command_id=7, sequence=11,
        newest=reference.source_newest_frame_index, reference=reference,
    )
    assert policy.open_live_reference_gate(
        source_epoch=reference.source_epoch,
        minimum_source_frame_index=reference.source_newest_frame_index,
        reset_yaw=True,
        arm_proof=proof,
    )
    return proof


def enter_hold(
    policy: TestableSonicPolicy,
    reference: SmplReferenceFrame | None = None,
) -> SmplReferenceFrame:
    live = reference or make_reference(row_marker=np.arange(10))
    authorize_reference(policy, live)
    policy.step(make_inference_frame(), 0.02)
    policy.test_clock.advance_ns(400_000_000)
    snapshot = policy.poll_source_snapshot()
    assert snapshot.status is not None
    policy.inject_status(
        make_status(
            status_sequence=snapshot.status.status_sequence + 1,
            status_monotonic_ns=policy.test_clock.now_ns,
            source_epoch=snapshot.status.source_epoch,
            last_arm_command_id=snapshot.status.last_arm_command_id,
            last_arm_target_epoch=snapshot.status.last_arm_target_epoch,
            last_requested_arm_epoch=snapshot.status.last_requested_arm_epoch,
            accepted_arm_epoch=snapshot.status.accepted_arm_epoch,
            producer_monotonic_ns=snapshot.status.producer_monotonic_ns,
            newest_frame_index=snapshot.status.newest_frame_index,
            ready=True,
            reference_window_ready=True,
            source_stale=False,
            ready_frames=30,
            recovery_frames=10,
            reason_code=XsensReason.ARMED_FRESH,
        )
    )
    policy.step(make_inference_frame(), 0.02)
    policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=False
    )
    policy.step(make_inference_frame(), 0.02)
    return live


def enter_completed_live(
    harness: PolicyHarness,
    *,
    epoch: int = 71,
    root_yaw: float = 0.1,
) -> SmplReferenceFrame:
    reference = make_reference(
        epoch=epoch, newest_frame=110, row_marker=np.arange(10),
        root_yaw_rad=root_yaw,
        producer_monotonic_ns=harness.clock.now_ns,
    )
    authorize_reference(harness.policy, reference)
    harness.step()
    harness.finish_blend()
    assert not harness.policy.source_blend_active
    return reference


def inject_healthy_pair(
    policy: TestableSonicPolicy,
    reference: SmplReferenceFrame,
    *,
    sequence: int,
    recovery_frames: int = 10,
) -> None:
    assert reference.source_epoch is not None
    assert reference.source_newest_frame_index is not None
    assert reference.producer_monotonic_ns is not None
    policy.inject_status(make_status(
        status_sequence=sequence,
        status_monotonic_ns=policy.test_clock.now_ns,
        source_epoch=reference.source_epoch,
        accepted_arm_epoch=reference.source_epoch,
        producer_monotonic_ns=reference.producer_monotonic_ns,
        newest_frame_index=reference.source_newest_frame_index,
        ready=True, reference_window_ready=True, source_stale=False,
        ready_frames=30, recovery_frames=recovery_frames,
        reason_code=(
            XsensReason.ARMED_FRESH
            if recovery_frames >= 10
            else XsensReason.ARMED_RECOVERING
        ),
    ))
    policy.inject_reference(reference)
    policy.poll_source_snapshot()


def inject_same_epoch_recovery(
    policy: TestableSonicPolicy,
    *,
    newest: int = 120,
) -> SmplReferenceFrame:
    assert policy.armed_source_epoch is not None
    reference = make_reference(
        epoch=policy.armed_source_epoch, newest_frame=newest,
        row_marker=np.arange(10) + 20,
        producer_monotonic_ns=policy.test_clock.now_ns,
    )
    inject_healthy_pair(policy, reference, sequence=14)
    assert policy.open_live_reference_gate(
        source_epoch=reference.source_epoch,
        minimum_source_frame_index=newest,
        reset_yaw=False, arm_proof=None,
    )
    return reference


def enter_rearm_blend(
    harness: PolicyHarness,
    *,
    old_epoch: int = 71,
    new_epoch: int = 72,
) -> tuple[SmplReferenceFrame, float]:
    enter_completed_live(harness, epoch=old_epoch, root_yaw=0.1)
    old_yaw = harness.policy.active_yaw_offset
    harness.policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=True
    )
    harness.step()
    harness.finish_blend()
    reference = make_reference(
        epoch=new_epoch, newest_frame=210, row_marker=np.arange(10) + 40,
        root_yaw_rad=1.0, producer_monotonic_ns=harness.clock.now_ns,
    )
    proof = inject_exact_authorized_pair(
        harness.policy,
        epoch=new_epoch, command_id=8, sequence=20, newest=210,
        reference=reference,
    )
    assert harness.policy.open_live_reference_gate(
        source_epoch=new_epoch, minimum_source_frame_index=210,
        reset_yaw=True, arm_proof=proof,
    )
    harness.step()
    return reference, old_yaw


def settle_hold(
    harness: PolicyHarness,
    *,
    rearm_required: bool,
) -> SmplReferenceFrame:
    reference = enter_completed_live(harness)
    harness.policy.close_live_reference_gate(
        hold_last_reference=True,
        rearm_required=rearm_required,
    )
    harness.step()
    harness.finish_blend()
    assert not harness.policy.source_blend_active
    return reference


def test_cached_ack_and_reference_cannot_open_gate(policy, fake_clock):
    policy.inject_status(make_status(
        status_sequence=10,
        source_epoch=71,
        accepted_arm_epoch=0,
        newest_frame_index=109,
    ))
    policy.inject_reference(make_reference(epoch=71, newest_frame=109))
    snapshot = policy.poll_source_snapshot()
    assert snapshot.status.status_sequence == 10
    assert not policy.open_live_reference_gate(
        source_epoch=71,
        minimum_source_frame_index=109,
        reset_yaw=True,
        arm_proof=ArmGateProof(7, 71, 71, 10),
    )


def test_xsens_gate_is_closed_by_default(policy):
    assert policy.source_kind == "xsens"
    assert policy.live_reference_gate_open is False
    assert policy.armed_source_epoch is None
    assert policy.minimum_source_frame_index is None


def test_exact_authorized_reference_join_opens_gate(policy):
    policy.inject_status(make_status(
        status_sequence=11,
        source_epoch=71,
        last_arm_command_id=7,
        last_arm_target_epoch=71,
        last_requested_arm_epoch=71,
        accepted_arm_epoch=71,
        newest_frame_index=110,
        ready=True,
        reference_window_ready=True,
    ))
    policy.inject_reference(make_reference(epoch=71, newest_frame=110))
    policy.poll_source_snapshot()
    assert policy.open_live_reference_gate(
        source_epoch=71,
        minimum_source_frame_index=110,
        reset_yaw=True,
        arm_proof=ArmGateProof(7, 71, 71, 10),
    )


def test_forwarded_reference_metadata_and_local_receipt_are_distinct(
    policy, fake_clock,
):
    reference = make_reference(
        epoch=71,
        newest_frame=110,
        producer_monotonic_ns=900_000_000,
    )
    policy.inject_reference(
        reference, received_mono=fake_clock.monotonic()
    )
    decoded = policy.poll_source_snapshot().reference
    assert decoded is not None
    assert decoded.source_epoch == 71
    assert decoded.source_newest_frame_index == 110
    assert decoded.producer_monotonic_ns == 900_000_000
    assert decoded.received_monotonic == pytest.approx(1.0)


def test_status_and_reference_queues_do_not_overwrite_each_other(policy):
    reference = make_reference(epoch=71, newest_frame=110)
    policy.inject_reference(reference)
    for sequence in range(1, 51):
        policy.inject_status(
            make_status(status_sequence=sequence, source_epoch=71)
        )
    snapshot = policy.poll_source_snapshot()
    assert snapshot.reference.source_newest_frame_index == 110
    assert snapshot.status.status_sequence == 50

    for newest in range(111, 121):
        policy.inject_reference(make_reference(epoch=71, newest_frame=newest))
    snapshot = policy.poll_source_snapshot()
    assert snapshot.status.status_sequence == 50
    assert snapshot.reference.source_newest_frame_index == 120


@pytest.mark.parametrize(
    ("reference_topic", "status_topic"),
    [
        ("", "xsens_status"),
        ("smpl_ref", ""),
        (None, "xsens_status"),
        ("smpl_ref", None),
        (7, "xsens_status"),
        ("smpl_ref", 7),
        ("same", "same"),
        ("smpl", "smpl_ref"),
        ("smpl_ref", "smpl"),
    ],
)
def test_unsafe_topics_are_rejected_before_resource_construction(
    reference_topic,
    status_topic,
):
    with pytest.raises(ValueError, match="topics"):
        SonicTeleopPolicy(
            "definitely-missing.onnx",
            "definitely-missing.npz",
            use_smpl_ref_zmq=False,
            smpl_ref_zmq_topic=reference_topic,
            xsens_status_zmq_topic=status_topic,
        )


def test_real_message_classifier_routes_default_topics_to_separate_snapshots(
    policy,
):
    reference = make_reference(epoch=71, newest_frame=110)
    reference_fields = {
        "term1_local": reference.term1_local,
        "root_quat": reference.root_quat,
        "wrist": reference.wrist,
        "frame_index": np.array([reference.frame_index], np.int64),
        "source_ready": np.array([reference.source_ready], np.bool_),
        "producer_monotonic_ns": np.array(
            [reference.producer_monotonic_ns], np.int64
        ),
        "source_epoch": np.array([reference.source_epoch], np.int64),
        "source_newest_frame_index": np.array(
            [reference.source_newest_frame_index], np.int64
        ),
    }
    policy._append_source_message(
        pack_pose_message(
            make_status(status_sequence=11, source_epoch=71),
            topic=policy.xsens_status_zmq_topic,
        ),
        1.0,
    )
    policy._append_source_message(
        pack_pose_message(
            reference_fields,
            topic=policy.smpl_ref_zmq_topic,
        ),
        1.0,
    )

    snapshot = policy.poll_source_snapshot()

    assert snapshot.status.status_sequence == 11
    assert snapshot.reference.source_newest_frame_index == 110


def test_same_batch_newer_then_older_reference_retains_newest_progress(
    policy,
):
    policy.inject_reference(make_reference(epoch=71, newest_frame=110))
    policy.poll_source_snapshot()
    policy.inject_reference(make_reference(epoch=71, newest_frame=111))
    policy.inject_reference(make_reference(epoch=71, newest_frame=110))

    snapshot = policy.poll_source_snapshot()

    assert snapshot.reference.source_epoch == 71
    assert snapshot.reference.source_newest_frame_index == 111


def test_cross_poll_older_reference_cannot_replace_newer_same_epoch(policy):
    policy.inject_reference(make_reference(epoch=71, newest_frame=111))
    policy.poll_source_snapshot()
    policy.inject_reference(make_reference(epoch=71, newest_frame=110))

    snapshot = policy.poll_source_snapshot()

    assert snapshot.reference.source_epoch == 71
    assert snapshot.reference.source_newest_frame_index == 111


def test_same_batch_and_cross_poll_status_sequence_rollback_is_ignored(
    policy,
):
    policy.inject_status(make_status(
        status_sequence=12, source_epoch=71, newest_frame_index=111,
    ))
    policy.inject_status(make_status(
        status_sequence=11, source_epoch=71, newest_frame_index=111,
    ))
    snapshot = policy.poll_source_snapshot()
    assert snapshot.status.status_sequence == 12

    policy.inject_status(make_status(
        status_sequence=10, source_epoch=71, newest_frame_index=111,
    ))
    snapshot = policy.poll_source_snapshot()

    assert snapshot.status.status_sequence == 12
    assert snapshot.status.newest_frame_index == 111


def test_same_epoch_status_cannot_regress_cached_frame_progress(policy):
    policy.inject_status(make_status(
        status_sequence=11, source_epoch=71, newest_frame_index=111,
    ))
    policy.poll_source_snapshot()
    policy.inject_status(make_status(
        status_sequence=12, source_epoch=71, newest_frame_index=110,
    ))

    snapshot = policy.poll_source_snapshot()

    assert snapshot.status.status_sequence == 11
    assert snapshot.status.newest_frame_index == 111


def test_malformed_messages_do_not_lower_status_or_reference_watermarks(
    policy,
):
    policy.inject_status(make_status(
        status_sequence=12, source_epoch=71, newest_frame_index=111,
    ))
    policy.inject_reference(make_reference(epoch=71, newest_frame=111))
    policy.poll_source_snapshot()

    malformed_status = make_status(
        status_sequence=13, source_epoch=71, newest_frame_index=112,
    )
    malformed_status["ready"] = np.array([1], dtype=np.int32)
    policy.inject_status(malformed_status)
    policy.inject_status(make_status(
        status_sequence=11, source_epoch=71, newest_frame_index=110,
    ))
    policy.inject_reference(replace(
        make_reference(epoch=71, newest_frame=112),
        root_quat=np.zeros((10, 4), dtype=np.float32),
    ))
    policy.inject_reference(make_reference(epoch=71, newest_frame=110))

    snapshot = policy.poll_source_snapshot()

    assert snapshot.status.status_sequence == 12
    assert snapshot.status.newest_frame_index == 111
    assert snapshot.reference.source_newest_frame_index == 111


def test_consumed_reference_progress_rejects_same_epoch_inference_replay(
    policy_harness,
):
    policy = policy_harness.policy
    authorize_reference(policy, make_reference(
        epoch=71, newest_frame=110, row_marker=1.0,
    ))
    policy_harness.step()
    assert policy.input_buffer[0, 0] == pytest.approx(1.0)

    policy.inject_status(make_status(
        status_sequence=12, source_epoch=71, last_arm_command_id=7,
        last_arm_target_epoch=71, last_requested_arm_epoch=71,
        accepted_arm_epoch=71, newest_frame_index=111,
    ))
    policy.inject_reference(make_reference(
        epoch=71, newest_frame=111, row_marker=2.0,
    ))
    policy_harness.step()
    assert policy.input_buffer[0, 0] == pytest.approx(2.0)

    policy.inject_status(make_status(
        status_sequence=13, source_epoch=71, last_arm_command_id=7,
        last_arm_target_epoch=71, last_requested_arm_epoch=71,
        accepted_arm_epoch=71, newest_frame_index=111,
    ))
    policy.inject_reference(make_reference(
        epoch=71, newest_frame=110, row_marker=3.0,
    ))
    policy_harness.step()

    assert policy.live_reference_gate_open
    assert policy.input_buffer[0, 0] == pytest.approx(2.0)
    assert policy.poll_source_snapshot().reference.source_newest_frame_index == 111


def test_changed_epoch_restarts_lower_progress_only_after_new_arm_proof(
    policy_harness,
):
    policy = policy_harness.policy
    authorize_reference(policy, make_reference(
        epoch=71, newest_frame=110, row_marker=1.0,
    ))
    policy_harness.step()

    policy.inject_status(make_status(
        status_sequence=1, source_epoch=72, last_arm_command_id=8,
        last_arm_target_epoch=72, last_requested_arm_epoch=72,
        accepted_arm_epoch=72, newest_frame_index=10,
    ))
    policy.inject_reference(make_reference(
        epoch=72, newest_frame=10, row_marker=2.0,
    ))
    snapshot = policy.poll_source_snapshot()
    assert snapshot.status.status_sequence == 1
    assert snapshot.reference.source_newest_frame_index == 10

    policy_harness.step()
    assert not policy.live_reference_gate_open
    assert not policy.open_live_reference_gate(
        source_epoch=72, minimum_source_frame_index=10,
        reset_yaw=True, arm_proof=None,
    )
    assert policy.open_live_reference_gate(
        source_epoch=72, minimum_source_frame_index=10,
        reset_yaw=True, arm_proof=ArmGateProof(8, 72, 72, 0),
    )
    policy_harness.step()
    assert policy.input_buffer[0, 0] == pytest.approx(2.0)

    policy.inject_status(make_status(
        status_sequence=2, source_epoch=72, last_arm_command_id=8,
        last_arm_target_epoch=72, last_requested_arm_epoch=72,
        accepted_arm_epoch=72, newest_frame_index=10,
    ))
    policy.inject_reference(make_reference(
        epoch=72, newest_frame=9, row_marker=3.0,
    ))
    policy_harness.step()

    assert policy.live_reference_gate_open
    assert policy.input_buffer[0, 0] == pytest.approx(2.0)


def test_false_ready_reference_is_observable_but_cannot_open_gate(policy):
    policy.inject_status(make_status(
        status_sequence=11, source_epoch=71, last_arm_command_id=7,
        last_arm_target_epoch=71, last_requested_arm_epoch=71,
        accepted_arm_epoch=71, ready=True, reference_window_ready=True,
        newest_frame_index=110,
    ))
    policy.inject_reference(make_reference(
        epoch=71, newest_frame=110, source_ready=False
    ))
    assert policy.poll_source_snapshot().reference.source_ready is False
    assert not policy.open_live_reference_gate(
        source_epoch=71, minimum_source_frame_index=110, reset_yaw=True,
        arm_proof=ArmGateProof(7, 71, 71, 10),
    )


@pytest.mark.parametrize(
    "proof",
    [
        ArmGateProof(8, 71, 71, 10),
        ArmGateProof(7, 72, 72, 10),
        ArmGateProof(7, 71, 0, 10),
        ArmGateProof(7, 71, 71, 11),
    ],
)
def test_gate_rejects_mismatched_or_not_newer_arm_proof(policy, proof):
    inject_exact_authorized_pair(
        policy, epoch=71, command_id=7, sequence=11, newest=110
    )
    assert not policy.open_live_reference_gate(
        source_epoch=71, minimum_source_frame_index=110,
        reset_yaw=True, arm_proof=proof,
    )


@pytest.mark.parametrize(
    (
        "status_command",
        "status_target",
        "status_requested",
        "status_accepted",
        "status_sequence",
        "proof",
    ),
    [
        (8, 71, 71, 71, 11, ArmGateProof(7, 71, 71, 10)),
        (7, 72, 71, 71, 11, ArmGateProof(7, 71, 71, 10)),
        (7, 71, 72, 71, 11, ArmGateProof(7, 71, 71, 10)),
        (7, 71, 71, 0, 11, ArmGateProof(7, 71, 71, 10)),
        (7, 71, 71, 71, 10, ArmGateProof(7, 71, 71, 10)),
        (7, 71, 71, 71, 11, ArmGateProof(7, 72, 71, 10)),
        (7, 71, 71, 71, 11, ArmGateProof(7, 71, 72, 10)),
    ],
)
def test_gate_requires_complete_arm_receipt_equality(
    policy,
    status_command,
    status_target,
    status_requested,
    status_accepted,
    status_sequence,
    proof,
):
    policy.inject_status(
        make_status(
            status_sequence=status_sequence,
            status_monotonic_ns=policy.test_clock.now_ns,
            source_epoch=71,
            last_arm_command_id=status_command,
            last_arm_target_epoch=status_target,
            last_requested_arm_epoch=status_requested,
            accepted_arm_epoch=status_accepted,
            producer_monotonic_ns=policy.test_clock.now_ns,
            newest_frame_index=110,
            ready=True,
            reference_window_ready=True,
            source_stale=False,
            reason_code=XsensReason.ARMED_FRESH,
        )
    )
    policy.inject_reference(
        make_reference(
            epoch=71,
            newest_frame=110,
            producer_monotonic_ns=policy.test_clock.now_ns,
        )
    )
    policy.poll_source_snapshot()
    assert not policy.open_live_reference_gate(
        source_epoch=71,
        minimum_source_frame_index=110,
        reset_yaw=True,
        arm_proof=proof,
    )


def test_proofless_recovery_requires_retained_epoch_without_rearm_barrier(
    policy,
):
    live = make_reference(epoch=71, newest_frame=110)
    authorize_reference(policy, live)
    policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=False
    )
    inject_exact_authorized_pair(
        policy, epoch=71, command_id=7, sequence=12, newest=111
    )
    assert policy.open_live_reference_gate(
        source_epoch=71,
        minimum_source_frame_index=111,
        reset_yaw=False,
        arm_proof=None,
    )

    policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=True
    )
    inject_exact_authorized_pair(
        policy, epoch=71, command_id=7, sequence=13, newest=112
    )
    assert not policy.open_live_reference_gate(
        source_epoch=71,
        minimum_source_frame_index=112,
        reset_yaw=False,
        arm_proof=None,
    )

    inject_exact_authorized_pair(
        policy, epoch=72, command_id=8, sequence=14, newest=210
    )
    assert not policy.open_live_reference_gate(
        source_epoch=72,
        minimum_source_frame_index=210,
        reset_yaw=True,
        arm_proof=None,
    )


@pytest.mark.parametrize(
    "age_ns,expected", [(200_000_000, True), (200_000_001, False)]
)
def test_status_timeout_is_valid_at_0_2_and_invalid_above(
    policy, fake_clock, age_ns, expected,
):
    proof = inject_exact_authorized_pair(
        policy, epoch=71, command_id=7, sequence=11, newest=110
    )
    fake_clock.advance_ns(age_ns)
    assert policy.open_live_reference_gate(
        source_epoch=71, minimum_source_frame_index=110,
        reset_yaw=True, arm_proof=proof,
    ) is expected


@pytest.mark.parametrize(
    "age_ns,expected", [(500_000_000, True), (500_000_001, False)]
)
def test_producer_timeout_is_valid_at_0_5_and_invalid_above(
    policy, fake_clock, age_ns, expected,
):
    produced = fake_clock.now_ns
    fake_clock.advance_ns(age_ns)
    proof = inject_exact_authorized_pair(
        policy, epoch=71, command_id=7, sequence=11, newest=110,
        producer_ns=produced, received_mono=fake_clock.monotonic(),
    )
    assert policy.open_live_reference_gate(
        source_epoch=71, minimum_source_frame_index=110,
        reset_yaw=True, arm_proof=proof,
    ) is expected


@pytest.mark.parametrize(
    "change",
    [
        lambda ref: replace(ref, source_epoch=72),
        lambda ref: replace(ref, source_newest_frame_index=109),
        lambda ref: replace(
            ref, producer_monotonic_ns=ref.producer_monotonic_ns - 500_000_001
        ),
    ],
)
def test_too_old_or_mismatched_reference_cannot_open_gate(policy, change):
    reference = change(make_reference(
        epoch=71, newest_frame=110,
        producer_monotonic_ns=policy.test_clock.now_ns,
    ))
    proof = inject_exact_authorized_pair(
        policy, epoch=71, command_id=7, sequence=11, newest=110,
        reference=reference,
    )
    assert not policy.open_live_reference_gate(
        source_epoch=71, minimum_source_frame_index=110,
        reset_yaw=True, arm_proof=proof,
    )


def test_republication_does_not_replace_forwarded_producer_time(
    policy, fake_clock,
):
    old = make_reference(
        epoch=71, newest_frame=110, producer_monotonic_ns=fake_clock.now_ns
    )
    fake_clock.advance_ns(500_000_001)
    proof = inject_exact_authorized_pair(
        policy, epoch=71, command_id=7, sequence=11, newest=110,
        reference=old, producer_ns=fake_clock.now_ns,
        received_mono=fake_clock.monotonic(),
    )
    assert not policy.open_live_reference_gate(
        source_epoch=71, minimum_source_frame_index=110,
        reset_yaw=True, arm_proof=proof,
    )


@pytest.mark.parametrize(
    (
        "status_received_offset_ns",
        "reference_received_offset_ns",
        "status_offset_ns",
        "reference_offset_ns",
    ),
    [
        (1, 0, 0, 0),
        (0, 1, 0, 0),
        (0, 0, 1, 0),
        (0, 0, 0, 1),
    ],
)
def test_local_and_producer_ages_must_be_nonnegative(
    policy,
    fake_clock,
    status_received_offset_ns,
    reference_received_offset_ns,
    status_offset_ns,
    reference_offset_ns,
):
    reference = make_reference(
        epoch=71,
        newest_frame=110,
        producer_monotonic_ns=fake_clock.now_ns + reference_offset_ns,
    )
    policy.inject_status(
        make_status(
            status_sequence=11,
            status_monotonic_ns=fake_clock.now_ns,
            source_epoch=71,
            last_arm_command_id=7,
            last_arm_target_epoch=71,
            last_requested_arm_epoch=71,
            accepted_arm_epoch=71,
            producer_monotonic_ns=fake_clock.now_ns + status_offset_ns,
            newest_frame_index=110,
            ready=True,
            reference_window_ready=True,
            source_stale=False,
            reason_code=XsensReason.ARMED_FRESH,
        ),
        received_mono=(
            fake_clock.monotonic()
            + status_received_offset_ns / 1_000_000_000.0
        ),
    )
    policy.inject_reference(
        reference,
        received_mono=(
            fake_clock.monotonic()
            + reference_received_offset_ns / 1_000_000_000.0
        ),
    )
    policy.poll_source_snapshot()
    assert not policy.open_live_reference_gate(
        source_epoch=71,
        minimum_source_frame_index=110,
        reset_yaw=True,
        arm_proof=ArmGateProof(7, 71, 71, 10),
    )


def test_status_newest_frame_must_cover_joined_reference(policy):
    reference = make_reference(epoch=71, newest_frame=111)
    proof = inject_exact_authorized_pair(
        policy,
        epoch=71,
        command_id=7,
        sequence=11,
        newest=110,
        reference=reference,
    )
    assert not policy.open_live_reference_gate(
        source_epoch=71,
        minimum_source_frame_index=110,
        reset_yaw=True,
        arm_proof=proof,
    )


def test_legacy_reference_without_metadata_keeps_local_receive_freshness(
    policy, fake_clock,
):
    policy.configure_runtime(
        yaw_bias_rad=0.3, live_ref_timeout_s=0.5, idle_frame_start=42,
        source_blend_duration_s=0.25, source_kind="legacy",
        status_timeout_s=0.2,
    )
    reference = replace(
        make_reference(), producer_monotonic_ns=None, source_epoch=None,
        source_newest_frame_index=None,
    )
    policy.inject_reference(reference)
    assert policy.has_fresh_live_reference()
    fake_clock.advance_ns(500_000_000)
    assert policy.has_fresh_live_reference()
    fake_clock.advance_ns(1)
    assert not policy.has_fresh_live_reference()
    assert (policy.yaw_bias_rad, policy.idle_frame_start) == (0.3, 42)
    assert policy.source_blend_duration_s == 0.25


def test_default_configure_runtime_preserves_all_legacy_assignments(policy):
    policy.configure_runtime(
        yaw_bias_rad=0.25,
        live_ref_timeout_s=0.75,
        idle_frame_start=41,
        source_blend_duration_s=0.3,
    )
    assert policy.yaw_bias_rad == pytest.approx(0.25)
    assert policy.live_ref_timeout_s == pytest.approx(0.75)
    assert policy.idle_frame_start == 41
    assert policy.source_blend_duration_s == pytest.approx(0.3)
    assert policy.source_kind == "legacy"
    assert policy.status_timeout_s == pytest.approx(0.2)


def test_legacy_live_and_idle_inference_needs_no_status_metadata(
    policy_harness,
):
    policy_harness.policy.configure_runtime(
        yaw_bias_rad=0.0,
        live_ref_timeout_s=0.5,
        idle_frame_start=3509,
        source_blend_duration_s=0.0,
    )
    legacy = replace(
        make_reference(row_marker=5.0),
        producer_monotonic_ns=None,
        source_epoch=None,
        source_newest_frame_index=None,
    )
    policy_harness.policy.inject_reference(legacy)
    policy_harness.step()
    assert policy_harness.policy.reference_source == "live"
    assert policy_harness.policy.input_buffer[0, 0] == pytest.approx(5.0)

    policy_harness.clock.advance_ns(500_000_001)
    policy_harness.step()
    assert policy_harness.policy.reference_source == "idle"
    assert policy_harness.policy.input_buffer[0, 0] == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("field", "replacement_value"),
    [
        ("ready", np.array([1], dtype=np.int32)),
        ("status_sequence", np.array([5, 6], dtype=np.int64)),
        ("source_epoch", np.array([-1], dtype=np.int64)),
    ],
)
def test_malformed_status_dtype_shape_or_range_retains_last_valid_snapshot(
    policy, field, replacement_value,
):
    policy.inject_status(make_status(status_sequence=4, ready=False))
    assert policy.poll_source_snapshot().status.status_sequence == 4
    malformed = make_status(status_sequence=5)
    malformed[field] = replacement_value
    policy.inject_status(malformed)
    retained = policy.poll_source_snapshot().status
    assert retained is not None
    assert retained.status_sequence == 4
    assert retained.ready is False


def test_reset_clears_both_queues_caches_and_gate(policy):
    proof = inject_exact_authorized_pair(
        policy, epoch=71, command_id=7, sequence=11, newest=110
    )
    assert policy.open_live_reference_gate(
        source_epoch=71, minimum_source_frame_index=110,
        reset_yaw=True, arm_proof=proof,
    )
    policy.inject_status(make_status(status_sequence=12, source_epoch=72))
    policy.inject_reference(make_reference(epoch=72, newest_frame=210))
    policy.reset(make_inference_frame())
    snapshot = policy.poll_source_snapshot()
    assert snapshot == JoinedSourceSnapshot(status=None, reference=None)
    assert not policy.live_reference_gate_open
    assert policy.armed_source_epoch is None
    assert policy.minimum_source_frame_index is None
    assert policy.source_kind == "xsens"
    assert policy.status_timeout_s == pytest.approx(0.2)
    assert policy.live_ref_timeout_s == pytest.approx(0.5)
    np.testing.assert_array_equal(
        policy.target_dof_pos, policy.default_dof_pos
    )


@pytest.mark.parametrize("epoch", [0, -1])
def test_source_epoch_must_be_positive_to_open_gate(policy, epoch):
    proof = inject_exact_authorized_pair(
        policy, epoch=epoch, command_id=7, sequence=11, newest=110
    )
    assert not policy.open_live_reference_gate(
        source_epoch=epoch, minimum_source_frame_index=110,
        reset_yaw=True, arm_proof=proof,
    )


def test_gate_is_revalidated_before_every_xsens_inference(
    policy_harness,
):
    live = make_reference(
        epoch=71,
        newest_frame=110,
        row_marker=5.0,
        producer_monotonic_ns=policy_harness.clock.now_ns,
    )
    authorize_reference(policy_harness.policy, live)
    policy_harness.step()
    assert policy_harness.policy.live_reference_gate_open
    assert policy_harness.policy.input_buffer[0, 0] == pytest.approx(5.0)

    policy_harness.clock.advance_ns(200_000_001)
    policy_harness.step()
    assert not policy_harness.policy.live_reference_gate_open
    assert policy_harness.policy.reference_source in {"idle", "hold"}


def test_fresh_to_hold_tiles_newest_human_row(policy):
    live = make_reference(epoch=71, newest_frame=109, row_marker=np.arange(10))
    authorize_reference(policy, live)
    policy.close_live_reference_gate(
        hold_last_reference=True,
        rearm_required=False,
    )
    held = policy.held_reference
    np.testing.assert_array_equal(
        held.term1_local,
        np.repeat(live.term1_local[-1:], 10, axis=0),
    )
    np.testing.assert_array_equal(
        held.root_quat,
        np.repeat(live.root_quat[-1:], 10, axis=0),
    )


def test_hold_inference_uses_current_robot_proprioception(policy):
    enter_hold(policy)
    policy.step(make_inference_frame(qpos_offset=0.0), 0.02)
    first = policy.input_buffer.copy()
    policy.step(make_inference_frame(qpos_offset=0.1), 0.02)
    second = policy.input_buffer.copy()
    assert not np.array_equal(first, second)
    np.testing.assert_array_equal(
        policy.selected_reference.term1_local,
        policy.held_reference.term1_local,
    )


def test_reset_robot_target_seed_is_opt_in_and_runtime_config_survives(policy):
    measured = make_inference_frame(qpos_offset=0.25)
    policy.reset(measured, seed_target_from_robot=True)
    np.testing.assert_array_equal(
        policy.target_dof_pos, measured.joints.position
    )
    assert policy.source_kind == "xsens"
    assert policy.status_timeout_s == pytest.approx(0.2)
    assert policy.live_ref_timeout_s == pytest.approx(0.5)

    policy.reset(measured)
    np.testing.assert_array_equal(
        policy.target_dof_pos, policy.default_dof_pos
    )


@pytest.mark.parametrize("with_anchor", [False, True])
def test_hold_tiles_wrist_and_optional_anchor(policy, with_anchor):
    reference = make_reference(row_marker=np.arange(10), with_anchor=with_anchor)
    authorize_reference(policy, reference)
    policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=False
    )
    policy.step(make_inference_frame(), 0.02)
    held = policy.held_reference
    np.testing.assert_array_equal(
        held.term1_local,
        np.repeat(reference.term1_local[-1:], 10, axis=0),
    )
    np.testing.assert_array_equal(
        held.root_quat,
        np.repeat(reference.root_quat[-1:], 10, axis=0),
    )
    np.testing.assert_array_equal(
        held.wrist, np.repeat(reference.wrist[-1:], 10, axis=0)
    )
    assert (held.anchor_quat is None) is (not with_anchor)
    if with_anchor:
        np.testing.assert_array_equal(
            held.anchor_quat,
            np.repeat(reference.anchor_quat[-1:], 10, axis=0),
        )


def test_fresh_to_hold_starts_exactly_one_blend(policy_harness):
    enter_completed_live(policy_harness)
    policy_harness.policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=False
    )
    policy_harness.step()
    started = policy_harness.policy.source_blend_started_at
    origin = policy_harness.policy.source_blend_from
    policy_harness.step()
    assert policy_harness.policy.source_blend_active
    assert policy_harness.policy.source_blend_started_at == started
    np.testing.assert_array_equal(
        policy_harness.policy.source_blend_from, origin
    )


def test_repeated_reference_does_not_restart_blend(policy_harness):
    reference = make_reference(
        row_marker=np.arange(10),
        producer_monotonic_ns=policy_harness.clock.now_ns,
    )
    authorize_reference(policy_harness.policy, reference)
    policy_harness.step()
    assert policy_harness.policy.source_blend_active
    started = policy_harness.policy.source_blend_started_at
    origin = policy_harness.policy.source_blend_from
    policy_harness.policy.inject_reference(reference)
    policy_harness.policy.poll_source_snapshot()
    policy_harness.step()
    assert policy_harness.policy.source_blend_started_at == started
    np.testing.assert_array_equal(
        policy_harness.policy.source_blend_from, origin
    )


def test_ninth_recovery_frame_remains_hold_until_join(policy_harness):
    settle_hold(policy_harness, rearm_required=False)
    policy = policy_harness.policy
    recovery = make_reference(
        epoch=policy.armed_source_epoch,
        newest_frame=120,
        row_marker=np.arange(10) + 20,
        producer_monotonic_ns=policy_harness.clock.now_ns,
    )
    inject_healthy_pair(
        policy, recovery, sequence=20, recovery_frames=9
    )
    policy_harness.step()
    assert not policy.live_reference_gate_open
    np.testing.assert_array_equal(
        policy.selected_reference.term1_local,
        policy.held_reference.term1_local,
    )

    inject_healthy_pair(
        policy, recovery, sequence=21, recovery_frames=10
    )
    policy_harness.step()
    assert not policy.live_reference_gate_open
    assert policy.reference_source == "hold"
    assert policy.open_live_reference_gate(
        source_epoch=71,
        minimum_source_frame_index=120,
        reset_yaw=False,
        arm_proof=None,
    )
    policy_harness.step()
    assert policy.reference_source == "live"


def test_same_epoch_recovery_preserves_active_yaw(policy_harness):
    enter_completed_live(policy_harness, root_yaw=0.2)
    policy = policy_harness.policy
    yaw = policy.active_yaw_offset
    policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=False
    )
    policy_harness.step()
    inject_same_epoch_recovery(policy)
    policy_harness.step()
    policy_harness.finish_blend()
    assert policy.active_yaw_offset == pytest.approx(yaw)


def test_new_epoch_rearm_uses_independent_pending_yaw(policy_harness):
    _, old = enter_rearm_blend(
        policy_harness, old_epoch=71, new_epoch=72
    )
    pending = policy_harness.policy.pending_yaw_offset
    assert pending is not None
    assert pending != pytest.approx(old)
    assert policy_harness.policy.active_yaw_offset == pytest.approx(old)
    policy_harness.finish_blend()
    assert policy_harness.policy.active_yaw_offset == pytest.approx(pending)
    assert policy_harness.policy.pending_yaw_offset is None


def test_stale_during_initial_arm_blend_preempts_from_current_target(
    policy_harness,
):
    reference = make_reference(
        row_marker=np.arange(10),
        producer_monotonic_ns=policy_harness.clock.now_ns,
    )
    authorize_reference(policy_harness.policy, reference)
    policy_harness.step()
    original = policy_harness.policy.source_blend_from
    pending = policy_harness.policy.pending_yaw_offset
    assert pending is not None
    sampled = policy_harness.advance_and_step(200_000_000)
    policy_harness.policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=False
    )
    policy_harness.step()
    np.testing.assert_array_equal(
        policy_harness.policy.source_blend_from, sampled
    )
    assert not np.array_equal(
        policy_harness.policy.source_blend_from, original
    )
    assert policy_harness.policy.pending_yaw_offset == pytest.approx(pending)
    assert policy_harness.policy.hold_yaw_offset == pytest.approx(pending)


def test_epoch_change_during_initial_arm_blend_keeps_latest_human_hold(
    policy_harness,
):
    reference = make_reference(
        row_marker=np.arange(10),
        producer_monotonic_ns=policy_harness.clock.now_ns,
    )
    authorize_reference(policy_harness.policy, reference)
    policy_harness.step()
    original = policy_harness.policy.source_blend_from
    pending = policy_harness.policy.pending_yaw_offset
    assert pending is not None
    sampled = policy_harness.advance_and_step(200_000_000)
    policy_harness.policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=True
    )
    policy_harness.step()
    np.testing.assert_array_equal(
        policy_harness.policy.source_blend_from, sampled
    )
    assert not np.array_equal(
        policy_harness.policy.source_blend_from, original
    )
    assert policy_harness.policy.pending_yaw_offset is None
    assert policy_harness.policy.hold_yaw_offset == pytest.approx(pending)
    np.testing.assert_array_equal(
        policy_harness.policy.held_reference.term1_local,
        np.repeat(reference.term1_local[-1:], 10, axis=0),
    )


def test_stale_during_rearm_blend_preempts_from_current_target(
    policy_harness,
):
    enter_rearm_blend(policy_harness)
    original = policy_harness.policy.source_blend_from
    sampled = policy_harness.advance_and_step(200_000_000)
    pending = policy_harness.policy.pending_yaw_offset
    assert pending is not None
    policy_harness.policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=False,
        preserve_pending_yaw=True,
    )
    policy_harness.step()
    np.testing.assert_array_equal(
        policy_harness.policy.source_blend_from, sampled
    )
    assert not np.array_equal(
        policy_harness.policy.source_blend_from, original
    )
    assert policy_harness.policy.pending_yaw_offset == pytest.approx(pending)
    assert policy_harness.policy._selected_yaw_offset == pytest.approx(
        policy_harness.policy.hold_yaw_offset
    )


def test_old_hold_clears_only_after_successful_blend(policy_harness):
    settle_hold(policy_harness, rearm_required=False)
    recovery = inject_same_epoch_recovery(policy_harness.policy)
    policy_harness.step()
    policy_harness.clock.advance_ns(399_999_999)
    inject_healthy_pair(
        policy_harness.policy, recovery, sequence=30
    )
    policy_harness.step()
    assert policy_harness.policy.held_reference is not None

    policy_harness.clock.advance_ns(1)
    inject_healthy_pair(
        policy_harness.policy, recovery, sequence=31
    )
    policy_harness.step()
    assert policy_harness.policy.held_reference is None


def test_pending_yaw_survives_only_same_armed_epoch(policy_harness):
    enter_rearm_blend(policy_harness)
    pending = policy_harness.policy.pending_yaw_offset
    policy_harness.policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=False,
        preserve_pending_yaw=True,
    )
    policy_harness.step()
    assert policy_harness.policy.pending_yaw_offset == pytest.approx(pending)
    policy_harness.policy.close_live_reference_gate(
        hold_last_reference=True, rearm_required=True,
        preserve_pending_yaw=False,
    )
    assert policy_harness.policy.pending_yaw_offset is None


@pytest.mark.parametrize(
    "context",
    ["fresh", "same_epoch_hold", "rearm_hold"],
)
def test_manual_alignment_updates_only_phase_owned_yaw_context(
    policy_harness, monkeypatch, context,
):
    if context == "fresh":
        enter_completed_live(policy_harness)
    elif context == "same_epoch_hold":
        settle_hold(policy_harness, rearm_required=False)
    else:
        settle_hold(policy_harness, rearm_required=True)

    policy = policy_harness.policy
    before_target = policy.target_dof_pos.copy()
    before_active = policy.active_yaw_offset
    before_hold = policy.hold_yaw_offset
    held = policy.held_reference
    monkeypatch.setattr(policy, "_compute_selected_yaw_offset", lambda: 3.0)
    assert policy.reset_xsens_alignment(context)
    if context == "fresh":
        assert policy.active_yaw_offset == pytest.approx(3.0)
        assert policy.hold_yaw_offset == pytest.approx(before_hold)
    elif context == "same_epoch_hold":
        assert policy.active_yaw_offset == pytest.approx(3.0)
        assert policy.hold_yaw_offset == pytest.approx(3.0)
    else:
        assert policy.active_yaw_offset == pytest.approx(before_active)
        assert policy.hold_yaw_offset == pytest.approx(3.0)
    assert policy.held_reference is held
    assert policy.source_blend_active
    np.testing.assert_array_equal(policy.source_blend_from, before_target)


def test_stale_during_alignment_blend_preempts_from_current_target(
    policy_harness, monkeypatch,
):
    enter_completed_live(policy_harness)
    policy = policy_harness.policy
    policy_harness.clock.advance_ns(1)
    fresh = make_reference(
        epoch=policy.armed_source_epoch,
        newest_frame=120,
        row_marker=np.arange(10),
        root_yaw_rad=0.1,
        producer_monotonic_ns=policy_harness.clock.now_ns,
    )
    inject_healthy_pair(policy, fresh, sequence=13)
    policy_harness.step()
    monkeypatch.setattr(
        policy,
        "_compute_selected_yaw_offset",
        lambda: policy.active_yaw_offset + 0.5,
    )
    assert policy.reset_xsens_alignment("fresh")
    original = policy.source_blend_from
    policy_harness.step()
    sampled = policy_harness.advance_and_step(200_000_000)
    policy.close_live_reference_gate(
        hold_last_reference=True,
        rearm_required=False,
    )
    policy_harness.step()
    np.testing.assert_array_equal(policy.source_blend_from, sampled)
    assert not np.array_equal(policy.source_blend_from, original)


@pytest.mark.parametrize("blocked_by", ["active_blend", "pending_yaw"])
def test_manual_alignment_rejects_active_blend_or_retained_pending_yaw(
    policy_harness, blocked_by,
):
    if blocked_by == "active_blend":
        reference = make_reference(
            producer_monotonic_ns=policy_harness.clock.now_ns
        )
        authorize_reference(policy_harness.policy, reference)
        policy_harness.step()
        context = "fresh"
    else:
        enter_rearm_blend(policy_harness)
        policy_harness.policy.close_live_reference_gate(
            hold_last_reference=True,
            rearm_required=False,
            preserve_pending_yaw=True,
        )
        policy_harness.step()
        policy_harness.finish_blend()
        assert not policy_harness.policy.source_blend_active
        assert policy_harness.policy.pending_yaw_offset is not None
        context = "same_epoch_hold"

    policy = policy_harness.policy
    before = (
        policy.active_yaw_offset,
        policy.hold_yaw_offset,
        policy.pending_yaw_offset,
        policy.source_blend_started_at,
    )
    held = policy.held_reference
    assert not policy.reset_xsens_alignment(context)
    assert (
        policy.active_yaw_offset,
        policy.hold_yaw_offset,
        policy.pending_yaw_offset,
        policy.source_blend_started_at,
    ) == before
    assert policy.held_reference is held
