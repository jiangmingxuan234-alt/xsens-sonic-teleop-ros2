from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Int64MultiArray

from bxi_example_py_elf3.framework.mod_api.transition import MotorFrame
from bxi_example_py_elf3.framework.runtime.mod_loader import (
    _discover_mods,
    _load_definition,
    _remove_module_prefixes,
)
from bxi_example_py_elf3.framework.runtime.resource_manager import ResourceManager
from policy import SONIC_PARAMETERS
from xsens.source_core import XsensReason
from xsens_test_helpers import (
    CaptureLogger,
    PolicyHarness,
    make_inference_frame,
    make_reference,
    make_status,
)


MOD_ROOT = Path(__file__).resolve().parents[1] / "mods" / "com.bxi.sonic"


@pytest.fixture(scope="module")
def state_types():
    """Load state.py in the same dynamic package shape used in production."""
    resources = ResourceManager()
    package_name = None
    try:
        discovered = _discover_mods((MOD_ROOT,))
        _, package = _load_definition(discovered["com.bxi.sonic"], resources)
        package_name = package.__name__
        state_module = sys.modules[f"{package_name}.state"]
        yield SimpleNamespace(
            XsensHoldCause=getattr(state_module, "XsensHoldCause"),
            XsensLinkStatus=getattr(state_module, "XsensLinkStatus"),
            XsensPhase=getattr(state_module, "XsensPhase"),
            XsensSonicTeleopState=getattr(
                state_module, "XsensSonicTeleopState"
            ),
        )
    finally:
        resources.close()
        if package_name is not None:
            _remove_module_prefixes((package_name,))


class FakeCommandPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        clone = Int64MultiArray()
        clone.layout.dim = []
        clone.layout.data_offset = int(message.layout.data_offset)
        clone.data = list(message.data)
        self.messages.append(SimpleNamespace(
            layout=clone.layout,
            data=list(clone.data),
        ))


class FakeRosNode:
    def __init__(self):
        self.publisher = FakeCommandPublisher()
        self.publisher_qos = None

    def create_publisher(self, message_type, topic, qos):
        assert message_type is Int64MultiArray
        assert topic == "sonic/xsens_arm_command"
        self.publisher_qos = qos
        return self.publisher

    def destroy_publisher(self, publisher):
        assert publisher is self.publisher


class FakeStateContext:
    def __init__(self, measured_offset=0.25):
        self.ros_node = FakeRosNode()
        self.robot_layout = SONIC_PARAMETERS.layout
        self.inference_frame = make_inference_frame(measured_offset)
        self.robot_joints = self.inference_frame.joints
        self.current_quat_xyzw = np.array([0, 0, 0, 1], np.float32)
        self.current_quat_wxyz = np.array([1, 0, 0, 0], np.float32)
        self.current_omega = np.zeros(3, np.float32)
        self.current_raw_cmd_vel = np.zeros(3, np.float32)
        self.current_cmd_vel = np.zeros(3, np.float32)
        self.speed_profiles = {}
        self.slot_values = {"btn_10": 0}
        self.motor_frames = []
        self.requested_states = []

    def remote_slot_value(self, slot):
        if slot not in self.slot_values:
            raise KeyError(f"undeclared remote slot: {slot}")
        return self.slot_values[slot]

    def set_motor_target(self, frame):
        self.motor_frames.append(MotorFrame.create(
            frame.layout, frame.qpos, frame.kp, frame.kd
        ))

    def request_state(self, state_name, **kwargs):
        self.requested_states.append((state_name, kwargs))
        return True


class ReadyPolicyHandle:
    def __init__(self, policy):
        self._policy = policy

    def get(self):
        return self._policy

    @property
    def status(self):
        return "ready"


class StateHarness:
    def __init__(
        self, state_types, *, clock=None,
        command_ids=(101, 102, 103, 104, 105, 106),
        measured_offset=0.25,
    ):
        self.types = state_types
        self.policy_harness = PolicyHarness()
        self.clock = clock or self.policy_harness.clock
        self.policy = self.policy_harness.policy
        self.policy._monotonic = self.clock.monotonic
        self.policy._monotonic_ns = self.clock.monotonic_ns
        self.policy.test_clock = self.clock
        self.ctx = FakeStateContext(measured_offset)
        self.logger = CaptureLogger()
        self._command_ids = iter(command_ids)
        self.state = state_types.XsensSonicTeleopState(
            "sonic_xsens", 77, ReadyPolicyHandle(self.policy),
            operator_prompt="保持近似中立姿势，等待 Xsens READY",
            command_id_factory=lambda: next(self._command_ids),
            monotonic=self.clock.monotonic,
        )
        self.state._bind_logger(self.logger)
        self.state.on_bind(self.ctx)
        self.entered = False
        self._live_command_start = 0

    @property
    def published_commands(self):
        return self.ctx.ros_node.publisher.messages

    @property
    def arm_commands_after_live(self):
        return self.published_commands[self._live_command_start:]

    def command_count(self):
        return len(self.published_commands)

    def enter_with_slot(self, value=0):
        self.ctx.slot_values["btn_10"] = value
        self.state.on_prepare(self.ctx, SimpleNamespace(name="normal"))
        self.state.on_enter(self.ctx)
        self.entered = True

    def set_slot(self, value):
        self.ctx.slot_values["btn_10"] = value

    def observe_slot(self, value):
        self.set_slot(value)
        self.tick()

    def tick(self, dt=0.02):
        self.state.on_update(self.ctx, dt)

    def advance_ns(self, delta, *, tick=True):
        self.clock.advance_ns(delta)
        if tick:
            self.tick()

    def publish_status(
        self, *, sequence, epoch, command_id=0, target=0, requested=0,
        accepted=0, newest_frame=109, ready=True,
        reference_window_ready=True, source_stale=False, ready_frames=30,
        recovery_frames=0, producer_ns=None, received_mono=None,
        reason_code=None,
    ):
        produced = self.clock.now_ns if producer_ns is None else producer_ns
        reason = (
            XsensReason.READY if ready else XsensReason.COLLECTING_STABILITY
        ) if reason_code is None else reason_code
        self.policy.inject_status(make_status(
            status_sequence=sequence, status_monotonic_ns=self.clock.now_ns,
            source_epoch=epoch, last_arm_command_id=command_id,
            last_arm_target_epoch=target,
            last_requested_arm_epoch=requested,
            accepted_arm_epoch=accepted,
            producer_monotonic_ns=produced,
            newest_frame_index=newest_frame, ready=ready,
            reference_window_ready=reference_window_ready,
            source_stale=source_stale, ready_frames=ready_frames,
            recovery_frames=recovery_frames, reason_code=reason,
        ), received_mono=received_mono)

    def publish_reference(
        self, *, epoch, newest_frame, producer_ns=None, row_marker=0.0,
        root_yaw_rad=0.0,
    ):
        reference = make_reference(
            epoch=epoch, newest_frame=newest_frame,
            producer_monotonic_ns=(
                self.clock.now_ns if producer_ns is None else producer_ns
            ),
            row_marker=row_marker, root_yaw_rad=root_yaw_rad,
        )
        self.policy.inject_reference(reference)
        return reference

    def complete_entry_disarm(self, *, epoch, sequence):
        command = self.last_command()
        assert command.data[1:] == [epoch, 0]
        self.publish_status(
            sequence=sequence, epoch=epoch, command_id=command.data[0],
            target=epoch, requested=0, accepted=0, ready=False,
            reference_window_ready=False, ready_frames=0,
        )
        self.tick()

    def make_ready(self, *, epoch, status_sequence, newest_frame):
        if not self.entered:
            self.enter_with_slot(0)
        if self.state.current_source_epoch != epoch:
            self.publish_status(
                sequence=status_sequence - 2, epoch=epoch, ready=False,
                reference_window_ready=False, ready_frames=0,
            )
            self.tick()
            if self.state.disarm_pending:
                self.complete_entry_disarm(
                    epoch=epoch, sequence=status_sequence - 1
                )
        self.publish_status(
            sequence=status_sequence, epoch=epoch, accepted=0,
            newest_frame=newest_frame, ready=True,
            reference_window_ready=True, ready_frames=30,
        )
        self.publish_reference(epoch=epoch, newest_frame=newest_frame)
        self.tick()
        assert self.state.phase is self.types.XsensPhase.READY

    def release_and_press(self):
        self.observe_slot(0)
        self.set_slot(11)
        assert self.state.on_action(self.ctx, "activate_xsens")

    def enter_live(self, *, epoch, newest_frame, root_yaw_rad=0.0):
        self.make_ready(epoch=epoch, status_sequence=20, newest_frame=newest_frame)
        self.release_and_press()
        command = self.last_command()
        self.publish_status(
            sequence=21, epoch=epoch, command_id=command.data[0],
            target=epoch, requested=epoch, accepted=epoch,
            newest_frame=newest_frame + 1,
        )
        self.publish_reference(
            epoch=epoch, newest_frame=newest_frame + 1,
            row_marker=np.arange(10), root_yaw_rad=root_yaw_rad,
        )
        self.tick()
        assert self.state.phase is self.types.XsensPhase.LIVE
        self._live_command_start = len(self.published_commands)

    def change_epoch(
        self, *, epoch, sequence, ready_frames=2, ready=False,
        newest_frame=2,
    ):
        self.publish_status(
            sequence=sequence, epoch=epoch, accepted=0,
            newest_frame=newest_frame, ready=ready,
            reference_window_ready=ready, ready_frames=ready_frames,
            reason_code=(
                XsensReason.READY if ready else XsensReason.SESSION_RESET
            ),
        )
        self.tick()

    def enter_same_epoch_hold(self, *, sequence=30):
        epoch = self.state.current_source_epoch
        self.publish_status(
            sequence=sequence, epoch=epoch, accepted=epoch,
            source_stale=True, ready=False, reference_window_ready=False,
            recovery_frames=0, reason_code=XsensReason.ARMED_STALE,
        )
        self.tick()

    def last_command(self):
        return self.published_commands[-1]

    def exit(self):
        self.state.on_exit(self.ctx)
        self.entered = False

    def close(self):
        if self.entered:
            self.exit()
        self.state.on_unbind(self.ctx)
        self.policy_harness.close()


@pytest.fixture
def state_harness(state_types):
    harness = StateHarness(state_types)
    yield harness
    harness.close()


@pytest.mark.parametrize(
    "override",
    [
        {"require_live_reference": True},
        {"manual_live_enable": False},
        {"manual_enable_neutral_value": 0.0},
        {"seed_entry_from_robot": False},
        {"hold_last_live_reference": False},
        {"auto_resume_same_epoch": False},
        {"rearm_on_source_epoch_change": False},
        {"hardware_gripper": True},
    ],
)
def test_compatibility_flags_reject_alternate_modes(state_types, override):
    policy_harness = PolicyHarness()
    try:
        with pytest.raises(ValueError):
            state_types.XsensSonicTeleopState(
                "sonic_xsens",
                77,
                ReadyPolicyHandle(policy_harness.policy),
                operator_prompt="等待 Xsens READY",
                **override,
            )
    finally:
        policy_harness.close()


def test_neutral_latch_requires_exact_zero(state_harness):
    state_harness.enter_with_slot(11)
    state_harness.observe_slot(2)
    assert not state_harness.state.neutral_latch_open
    assert state_harness.state.on_action(state_harness.ctx, "activate_xsens") is True
    assert state_harness.published_commands == []
    state_harness.observe_slot(0)
    assert state_harness.state.neutral_latch_open


def test_arm_press_waits_for_exact_ack_and_reference_join(state_harness):
    state_harness.make_ready(epoch=71, status_sequence=20, newest_frame=109)
    state_harness.release_and_press()
    command = state_harness.last_command()
    assert command.data == [command.data[0], 71, 71]
    assert state_harness.state.phase is state_harness.types.XsensPhase.READY
    assert state_harness.state.arm_pending

    state_harness.publish_status(
        sequence=21, epoch=71, command_id=command.data[0],
        target=71, requested=71, accepted=71, newest_frame=110,
    )
    state_harness.tick()
    assert state_harness.state.phase is state_harness.types.XsensPhase.READY
    assert state_harness.state.arm_pending
    state_harness.publish_reference(epoch=71, newest_frame=110)
    state_harness.tick()
    assert state_harness.state.phase is state_harness.types.XsensPhase.LIVE
    assert (
        state_harness.state.link_status
        is state_harness.types.XsensLinkStatus.FRESH
    )


def test_pending_arm_cancels_when_reference_epoch_regresses(state_harness):
    state_harness.make_ready(epoch=71, status_sequence=20, newest_frame=109)
    state_harness.release_and_press()
    arm = state_harness.last_command()
    state_harness.publish_reference(epoch=72, newest_frame=209)
    state_harness.tick()
    assert not state_harness.state.arm_pending
    assert state_harness.state.disarm_pending
    assert state_harness.last_command().data[1:] == [71, 0]
    assert state_harness.last_command().data[0] != arm.data[0]


def test_exact_ack_waits_when_same_epoch_reference_is_temporarily_ahead(
    state_harness,
):
    state_harness.make_ready(epoch=71, status_sequence=20, newest_frame=109)
    state_harness.release_and_press()
    arm = state_harness.last_command()
    state_harness.publish_status(
        sequence=21,
        epoch=71,
        command_id=arm.data[0],
        target=71,
        requested=71,
        accepted=71,
        newest_frame=110,
    )
    state_harness.publish_reference(epoch=71, newest_frame=111)
    state_harness.tick()
    assert state_harness.state.arm_pending
    assert not state_harness.state.disarm_pending

    state_harness.publish_status(
        sequence=22,
        epoch=71,
        command_id=arm.data[0],
        target=71,
        requested=71,
        accepted=71,
        newest_frame=111,
    )
    state_harness.tick()
    assert state_harness.state.phase is state_harness.types.XsensPhase.LIVE
    assert state_harness.state.link_status is state_harness.types.XsensLinkStatus.FRESH


def test_ready_regressions_before_and_after_arm_have_distinct_barriers(state_harness):
    state_harness.make_ready(epoch=71, status_sequence=20, newest_frame=109)
    baseline_count = state_harness.command_count()
    state_harness.publish_status(sequence=21, epoch=71, ready=False)
    state_harness.tick()
    assert (
        state_harness.state.phase
        is state_harness.types.XsensPhase.WAITING_FOR_DATA
    )
    assert state_harness.command_count() == baseline_count

    state_harness.make_ready(epoch=71, status_sequence=22, newest_frame=110)
    state_harness.set_slot(0)
    state_harness.publish_status(
        sequence=23, epoch=71, ready=False, newest_frame=110
    )
    assert state_harness.state.on_action(state_harness.ctx, "activate_xsens") is True
    assert state_harness.command_count() == baseline_count
    assert not state_harness.state.neutral_latch_open

    state_harness.make_ready(epoch=71, status_sequence=24, newest_frame=111)
    state_harness.release_and_press()
    assert state_harness.state.arm_pending
    state_harness.publish_status(
        sequence=25, epoch=71, ready=False, newest_frame=111
    )
    state_harness.tick()
    cancel = state_harness.last_command()
    assert cancel.data[1:] == [71, 0]
    assert state_harness.state.disarm_pending


def test_entry_disarm_requires_exact_newer_receipt(state_harness):
    state_harness.enter_with_slot(0)
    state_harness.publish_status(sequence=4, epoch=71, accepted=0)
    state_harness.tick()
    command = state_harness.last_command()
    assert command.data[1:] == [71, 0]
    qos = state_harness.ctx.ros_node.publisher_qos
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.VOLATILE
    assert qos.history is HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    state_harness.publish_status(
        sequence=5, epoch=71, command_id=0, target=0,
        requested=0, accepted=0,
    )
    state_harness.tick()
    assert state_harness.state.disarm_pending
    state_harness.publish_status(
        sequence=6, epoch=71, command_id=command.data[0], target=71,
        requested=0, accepted=0,
    )
    state_harness.tick()
    assert not state_harness.state.disarm_pending


def test_same_epoch_stale_auto_recovers_only_after_join(state_harness):
    state_harness.enter_live(epoch=71, newest_frame=110)
    state_harness.publish_status(
        sequence=30, epoch=71, accepted=71, source_stale=True,
        recovery_frames=0, reference_window_ready=False,
        newest_frame=111,
    )
    state_harness.tick()
    assert (
        state_harness.state.link_status
        is state_harness.types.XsensLinkStatus.HOLD
    )
    assert (
        state_harness.state.hold_cause
        is state_harness.types.XsensHoldCause.PRODUCER_STALE
    )
    state_harness.publish_status(
        sequence=31, epoch=71, accepted=71, source_stale=False,
        recovery_frames=10, reference_window_ready=True, newest_frame=120,
    )
    state_harness.tick()
    assert (
        state_harness.state.link_status
        is state_harness.types.XsensLinkStatus.HOLD
    )
    state_harness.publish_reference(epoch=71, newest_frame=120)
    state_harness.tick()
    assert (
        state_harness.state.link_status
        is state_harness.types.XsensLinkStatus.FRESH
    )
    assert state_harness.state.hold_cause is None
    assert state_harness.arm_commands_after_live == []


def test_producer_stale_hold_cannot_downgrade_across_heartbeat_loss(
    state_harness,
):
    state_harness.enter_live(epoch=71, newest_frame=110)
    state_harness.publish_status(
        sequence=30,
        epoch=71,
        accepted=71,
        source_stale=True,
        ready=False,
        reference_window_ready=False,
        recovery_frames=0,
        newest_frame=111,
        reason_code=XsensReason.ARMED_STALE,
    )
    state_harness.tick()
    assert (
        state_harness.state.hold_cause
        is state_harness.types.XsensHoldCause.PRODUCER_STALE
    )

    state_harness.advance_ns(200_000_001)
    assert (
        state_harness.state.hold_cause
        is state_harness.types.XsensHoldCause.PRODUCER_STALE
    )

    state_harness.publish_status(
        sequence=31,
        epoch=71,
        accepted=71,
        source_stale=False,
        recovery_frames=0,
        newest_frame=112,
        reason_code=XsensReason.ARMED_FRESH,
    )
    state_harness.publish_reference(epoch=71, newest_frame=112)
    state_harness.tick()
    assert state_harness.state.link_status is state_harness.types.XsensLinkStatus.HOLD
    assert (
        state_harness.state.hold_cause
        is state_harness.types.XsensHoldCause.PRODUCER_STALE
    )


def test_new_epoch_substitutes_atomic_disarm_barrier(state_harness):
    state_harness.enter_with_slot(0)
    state_harness.publish_status(
        sequence=4, epoch=71, accepted=0, ready=False,
        reference_window_ready=False, ready_frames=0,
    )
    state_harness.tick()
    assert state_harness.state.disarm_pending
    command_count = state_harness.command_count()

    state_harness.publish_status(
        sequence=1, epoch=72, accepted=0, ready=False,
        reference_window_ready=False, ready_frames=2, newest_frame=2,
        reason_code=XsensReason.SESSION_RESET,
    )
    state_harness.tick()

    assert state_harness.command_count() == command_count
    assert not state_harness.state.disarm_pending
    assert state_harness.state.current_source_epoch == 72
    assert state_harness.state.ready_frames == 2
    assert (
        state_harness.state.phase
        is state_harness.types.XsensPhase.WAITING_FOR_DATA
    )


def test_waiting_and_ready_select_idle_frame_3509(state_harness):
    state_harness.enter_with_slot(0)
    state_harness.tick()
    assert state_harness.policy.reference_source == "idle"
    assert state_harness.policy.selected_reference.frame_index == 3509

    state_harness.make_ready(epoch=71, status_sequence=20, newest_frame=109)
    state_harness.publish_reference(
        epoch=71, newest_frame=109, row_marker=77.0
    )
    state_harness.tick()
    assert state_harness.state.phase is state_harness.types.XsensPhase.READY
    assert state_harness.policy.reference_source == "idle"
    assert state_harness.policy.selected_reference.frame_index == 3509
    assert not np.any(state_harness.policy.selected_reference.term1_local == 77.0)


def test_command_id_draw_rejects_negative_zero_large_and_collision(state_types):
    harness = StateHarness(
        state_types,
        command_ids=(101, -1, 0, 2**63, 101, 102, 103),
    )
    try:
        harness.make_ready(epoch=71, status_sequence=20, newest_frame=109)
        assert [message.data[0] for message in harness.published_commands] == [101]
        harness.release_and_press()
        assert [message.data[0] for message in harness.published_commands] == [
            101,
            102,
        ]
        assert harness.last_command().data == [102, 71, 71]
    finally:
        harness.close()


def test_status_and_join_timeout_boundaries_are_inclusive(state_types):
    status_harness = StateHarness(state_types)
    try:
        status_harness.enter_live(epoch=71, newest_frame=110)
        status_harness.advance_ns(200_000_000)
        assert (
            status_harness.state.link_status
            is state_types.XsensLinkStatus.FRESH
        )
        status_harness.advance_ns(1)
        assert (
            status_harness.state.link_status
            is state_types.XsensLinkStatus.HOLD
        )
        assert (
            status_harness.state.hold_cause
            is state_types.XsensHoldCause.STATUS_HEARTBEAT
        )
    finally:
        status_harness.close()

    join_harness = StateHarness(state_types)
    try:
        join_harness.make_ready(
            epoch=71, status_sequence=20, newest_frame=109
        )
        join_harness.release_and_press()
        arm = join_harness.last_command()
        count_after_arm = join_harness.command_count()
        join_harness.advance_ns(500_000_000, tick=False)
        join_harness.publish_status(
            sequence=21, epoch=71, accepted=0, newest_frame=109,
            ready=True, reference_window_ready=True, ready_frames=30,
        )
        join_harness.tick()
        assert join_harness.state.arm_pending
        assert join_harness.command_count() == count_after_arm

        join_harness.advance_ns(1)
        assert not join_harness.state.arm_pending
        assert join_harness.state.disarm_pending
        assert join_harness.command_count() == count_after_arm + 1
        assert join_harness.last_command().data == [
            join_harness.last_command().data[0],
            71,
            0,
        ]
        assert join_harness.last_command().data[0] != arm.data[0]
    finally:
        join_harness.close()


def test_local_status_timeout_recovers_without_udp_recovery_count(state_harness):
    state_harness.enter_live(epoch=71, newest_frame=110)
    commands = state_harness.command_count()
    state_harness.advance_ns(200_000_001)
    assert state_harness.state.link_status is state_harness.types.XsensLinkStatus.HOLD
    assert (
        state_harness.state.hold_cause
        is state_harness.types.XsensHoldCause.STATUS_HEARTBEAT
    )

    state_harness.publish_status(
        sequence=22, epoch=71, accepted=71, newest_frame=111,
        ready=True, reference_window_ready=True, source_stale=False,
        recovery_frames=0, reason_code=XsensReason.ARMED_FRESH,
    )
    state_harness.publish_reference(epoch=71, newest_frame=111, row_marker=11.0)
    state_harness.tick()
    assert state_harness.state.link_status is state_harness.types.XsensLinkStatus.FRESH
    assert state_harness.state.hold_cause is None
    assert state_harness.command_count() == commands


def test_heartbeat_recovery_latches_first_healthy_status_floor(state_harness):
    state_harness.enter_live(epoch=71, newest_frame=110)
    state_harness.advance_ns(200_000_001)
    assert (
        state_harness.state.hold_cause
        is state_harness.types.XsensHoldCause.STATUS_HEARTBEAT
    )

    state_harness.publish_status(
        sequence=22,
        epoch=71,
        accepted=71,
        newest_frame=120,
        reason_code=XsensReason.ARMED_FRESH,
    )
    state_harness.tick()
    assert state_harness.state.link_status is state_harness.types.XsensLinkStatus.HOLD

    state_harness.publish_status(
        sequence=23,
        epoch=71,
        accepted=71,
        newest_frame=121,
        reason_code=XsensReason.ARMED_FRESH,
    )
    state_harness.publish_reference(epoch=71, newest_frame=120)
    state_harness.tick()
    assert state_harness.state.link_status is state_harness.types.XsensLinkStatus.FRESH
    assert state_harness.state.hold_cause is None


def test_live_reference_epoch_mismatch_enters_hold(state_harness):
    state_harness.enter_live(epoch=71, newest_frame=110)
    state_harness.publish_reference(epoch=72, newest_frame=209)
    state_harness.tick()
    assert state_harness.state.link_status is state_harness.types.XsensLinkStatus.HOLD
    assert (
        state_harness.state.hold_cause
        is state_harness.types.XsensHoldCause.PRODUCER_STALE
    )
    assert not state_harness.policy.live_reference_gate_open


def test_changed_epoch_requires_ready_manual_rearm_and_new_yaw(state_harness):
    state_harness.enter_live(
        epoch=71, newest_frame=110, root_yaw_rad=0.1
    )
    state_harness.advance_ns(400_000_000, tick=False)
    state_harness.publish_status(
        sequence=22, epoch=71, accepted=71, newest_frame=111,
        reason_code=XsensReason.ARMED_FRESH,
    )
    state_harness.publish_reference(
        epoch=71, newest_frame=111, root_yaw_rad=0.1
    )
    state_harness.tick()
    old_yaw = state_harness.policy.active_yaw_offset

    state_harness.change_epoch(epoch=72, sequence=30)
    old_hold = state_harness.policy.held_reference.term1_local.copy()
    assert (
        state_harness.state.link_status
        is state_harness.types.XsensLinkStatus.HOLD_REARM_REQUIRED
    )
    state_harness.publish_reference(
        epoch=72, newest_frame=209, row_marker=99.0, root_yaw_rad=1.2
    )
    state_harness.tick()
    np.testing.assert_array_equal(
        state_harness.policy.selected_reference.term1_local, old_hold
    )

    state_harness.set_slot(11)
    command_count = state_harness.command_count()
    assert state_harness.state.on_action(
        state_harness.ctx, "activate_xsens"
    )
    assert state_harness.command_count() == command_count

    state_harness.publish_status(
        sequence=31, epoch=72, accepted=0, newest_frame=209,
        ready=True, reference_window_ready=True, ready_frames=30,
    )
    state_harness.publish_reference(
        epoch=72, newest_frame=209, row_marker=99.0, root_yaw_rad=1.2
    )
    state_harness.tick()
    assert state_harness.state.rearm_ready
    state_harness.release_and_press()
    arm = state_harness.last_command()
    assert arm.data[1:] == [72, 72]

    state_harness.publish_status(
        sequence=32, epoch=72, command_id=arm.data[0], target=72,
        requested=72, accepted=72, newest_frame=210,
        reason_code=XsensReason.ARMED_FRESH,
    )
    state_harness.tick()
    assert (
        state_harness.state.link_status
        is state_harness.types.XsensLinkStatus.HOLD_REARM_REQUIRED
    )
    np.testing.assert_array_equal(
        state_harness.policy.selected_reference.term1_local, old_hold
    )

    state_harness.publish_reference(
        epoch=72, newest_frame=210, row_marker=100.0, root_yaw_rad=1.2
    )
    state_harness.tick()
    assert state_harness.state.link_status is state_harness.types.XsensLinkStatus.FRESH
    assert state_harness.policy.pending_yaw_offset != pytest.approx(old_yaw)
    state_harness.advance_ns(400_000_000, tick=False)
    state_harness.publish_status(
        sequence=33, epoch=72, accepted=72, newest_frame=211,
        reason_code=XsensReason.ARMED_FRESH,
    )
    state_harness.publish_reference(
        epoch=72, newest_frame=211, row_marker=101.0, root_yaw_rad=1.2
    )
    state_harness.tick()
    assert state_harness.policy.active_yaw_offset != pytest.approx(old_yaw)


def test_second_epoch_before_rearm_preserves_original_hold(state_harness):
    state_harness.enter_live(epoch=71, newest_frame=110)
    state_harness.change_epoch(epoch=72, sequence=30)
    held_term = state_harness.policy.held_reference.term1_local.copy()
    held_root = state_harness.policy.held_reference.root_quat.copy()

    state_harness.change_epoch(epoch=73, sequence=1, newest_frame=2)
    assert state_harness.state.pending_source_epoch == 73
    assert state_harness.state.ready_frames == 2
    np.testing.assert_array_equal(
        state_harness.policy.held_reference.term1_local, held_term
    )
    np.testing.assert_array_equal(
        state_harness.policy.held_reference.root_quat, held_root
    )


@pytest.mark.parametrize("hold_before_epoch_change", [False, True])
def test_phase_ignored_live_press_consumes_latch(
    state_types, hold_before_epoch_change
):
    harness = StateHarness(state_types)
    try:
        harness.enter_live(epoch=71, newest_frame=110)
        if hold_before_epoch_change:
            harness.enter_same_epoch_hold(sequence=30)
        harness.observe_slot(0)
        before = harness.command_count()
        harness.set_slot(11)
        assert harness.state.on_action(harness.ctx, "activate_xsens")
        assert harness.command_count() == before
        assert not harness.state.neutral_latch_open

        harness.change_epoch(epoch=72, sequence=40)
        harness.publish_status(
            sequence=41, epoch=72, accepted=0, newest_frame=209,
            ready=True, reference_window_ready=True, ready_frames=30,
        )
        harness.publish_reference(epoch=72, newest_frame=209)
        harness.tick()
        assert harness.state.rearm_ready

        harness.observe_slot(2)
        harness.set_slot(11)
        assert harness.state.on_action(harness.ctx, "activate_xsens")
        assert harness.command_count() == before
        harness.observe_slot(0)
        harness.set_slot(11)
        assert harness.state.on_action(harness.ctx, "activate_xsens")
        assert harness.command_count() == before + 1
        assert harness.last_command().data[1:] == [72, 72]
    finally:
        harness.close()


def test_exit_reentry_reinitializes_latch_from_slot_snapshot(state_harness):
    state_harness.enter_with_slot(0)
    assert state_harness.state.neutral_latch_open
    state_harness.exit()

    state_harness.enter_with_slot(11)
    assert not state_harness.state.neutral_latch_open
    state_harness.exit()

    state_harness.enter_with_slot(0)
    assert state_harness.state.neutral_latch_open


def test_entry_target_uses_measured_robot_joints(state_types):
    harness = StateHarness(state_types, measured_offset=0.25)
    try:
        harness.enter_with_slot(0)
        entry = harness.state.get_entry_frame(harness.ctx)
        measured = harness.ctx.inference_frame.joints.position
        np.testing.assert_array_equal(entry.qpos, measured)
        assert not np.array_equal(
            entry.qpos, SONIC_PARAMETERS.default_position
        )
    finally:
        harness.close()


@pytest.mark.parametrize(
    "mode,expected_context",
    [
        ("waiting", None),
        ("ready", None),
        ("pending", None),
        ("active_blend", None),
        ("retained_pending_yaw", None),
        ("fresh", "fresh"),
        ("same_epoch_hold", "same_epoch_hold"),
        ("rearm_hold", "rearm_hold"),
    ],
)
def test_manual_alignment_routes_only_stable_owned_yaw_context(
    state_types, mode, expected_context
):
    harness = StateHarness(state_types)
    try:
        harness.enter_with_slot(0)
        phase = state_types.XsensPhase
        link = state_types.XsensLinkStatus
        if mode == "ready":
            harness.make_ready(epoch=71, status_sequence=20, newest_frame=109)
        elif mode == "pending":
            harness.make_ready(epoch=71, status_sequence=20, newest_frame=109)
            harness.release_and_press()
        elif mode in {
            "active_blend", "retained_pending_yaw", "fresh",
            "same_epoch_hold", "rearm_hold",
        }:
            harness.state._phase = phase.LIVE
            harness.state._link_status = (
                link.HOLD_REARM_REQUIRED
                if mode == "rearm_hold"
                else link.HOLD
                if mode in {"same_epoch_hold", "retained_pending_yaw"}
                else link.FRESH
            )
        if mode == "active_blend":
            harness.policy._source_blend_active = True
        if mode == "retained_pending_yaw":
            harness.policy.pending_yaw_offset = 0.75

        calls = []
        harness.policy.reset_xsens_alignment = (
            lambda context: calls.append(context) or True
        )
        before = (
            harness.policy.active_yaw_offset,
            harness.policy.hold_yaw_offset,
            harness.policy.pending_yaw_offset,
        )
        assert harness.state.on_action(harness.ctx, "reset_alignment") is True
        assert calls == ([] if expected_context is None else [expected_context])
        if expected_context is None:
            assert (
                harness.policy.active_yaw_offset,
                harness.policy.hold_yaw_offset,
                harness.policy.pending_yaw_offset,
            ) == before
    finally:
        harness.close()


def test_state_exit_sends_one_disarm_without_waiting_or_safety_request(
    state_harness,
):
    state_harness.enter_live(epoch=71, newest_frame=110)
    before = state_harness.command_count()
    state_harness.exit()
    assert state_harness.command_count() == before + 1
    assert state_harness.last_command().data[1:] == [71, 0]
    assert not state_harness.state.is_entered
    assert not state_harness.policy.live_reference_gate_open
    assert state_harness.ctx.requested_states == []


def test_state_exit_targets_latest_epoch_during_rearm_hold(state_harness):
    state_harness.enter_live(epoch=71, newest_frame=110)
    state_harness.change_epoch(epoch=72, sequence=30)
    before = state_harness.command_count()
    state_harness.exit()
    assert state_harness.command_count() == before + 1
    assert state_harness.last_command().data[1:] == [72, 0]


def test_operator_prompts_are_exact_and_reason_scoped(state_harness):
    initial_prompt = (
        "Xsens READY — release controls, then press LT+RT+Y to request LIVE"
    )
    rearm_prompt = (
        "Xsens new session READY — release controls, then press LT+RT+Y to re-arm"
    )
    initial_wait = (
        "Xsens enable rejected — wait for READY; "
        "reason=COLLECTING_STABILITY"
    )
    rearm_wait = (
        "Xsens re-arm rejected — wait for new-session READY; "
        "reason=SESSION_RESET"
    )

    state_harness.enter_with_slot(0)
    state_harness.publish_status(
        sequence=1, epoch=71, accepted=0, ready=False,
        reference_window_ready=False, ready_frames=0,
        reason_code=XsensReason.COLLECTING_STABILITY,
    )
    state_harness.tick()
    state_harness.set_slot(11)
    assert state_harness.state.on_action(
        state_harness.ctx, "activate_xsens"
    )
    assert initial_wait in state_harness.logger.warnings
    state_harness.complete_entry_disarm(epoch=71, sequence=2)
    state_harness.make_ready(epoch=71, status_sequence=3, newest_frame=109)
    assert state_harness.logger.infos.count(initial_prompt) == 1
    state_harness.tick()
    state_harness.tick()
    assert state_harness.logger.infos.count(initial_prompt) == 1

    state_harness.enter_live(epoch=71, newest_frame=110)
    state_harness.change_epoch(epoch=72, sequence=30)
    state_harness.observe_slot(0)
    state_harness.set_slot(11)
    assert state_harness.state.on_action(
        state_harness.ctx, "activate_xsens"
    )
    assert rearm_wait in state_harness.logger.warnings
    state_harness.publish_status(
        sequence=31, epoch=72, accepted=0, newest_frame=209,
        ready=True, reference_window_ready=True, ready_frames=30,
    )
    state_harness.publish_reference(epoch=72, newest_frame=209)
    state_harness.tick()
    assert state_harness.logger.infos.count(rearm_prompt) == 1
    state_harness.tick()
    assert state_harness.logger.infos.count(rearm_prompt) == 1
