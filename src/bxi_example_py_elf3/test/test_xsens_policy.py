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
        signal = human + 0.25 * root + 0.01 * proprio
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
