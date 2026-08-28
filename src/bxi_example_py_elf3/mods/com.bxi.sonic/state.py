from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math
import operator
import secrets
import time
from typing import TYPE_CHECKING, Callable, Literal, Optional, Protocol

import numpy as np
import communication.msg as bxi_msg
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32, Int64MultiArray

from bxi_example_py_elf3.framework.inference import InferenceFrame, PolicyOutput
from bxi_example_py_elf3.framework.mod_api import ResourceHandle, RobotControlState
from bxi_example_py_elf3.framework.mod_api import StateBehavior
from bxi_example_py_elf3.framework.mod_api.transition import (
    EntryFrameProvider,
    MotorFrame,
    RunningFrameProvider,
)

from .gripper import BxiMotor, JointControl
from .policy import (
    ArmGateProof,
    JoinedSourceSnapshot,
    XsensStatusSnapshot,
)
from .xsens.source_core import XsensReason

if TYPE_CHECKING:
    from bxi_example_py_elf3.framework.mod_api import LoggerLike, RobotControlContext


PICO_OPERATOR_PROMPT = (
    "PICO同时按住A+B+X+Y请求校准，再按A+X切入实时POSE"
)


class SonicPolicy(Protocol):
    output: PolicyOutput
    last_status: str

    def bind_logger(self, logger: LoggerLike) -> None:
        ...

    def reset(
        self,
        frame: InferenceFrame | None = None,
        *,
        seed_target_from_robot: bool = False,
    ) -> None:
        ...

    def step(
        self,
        frame: InferenceFrame,
        dt: float,
        *,
        advance: bool = True,
    ) -> PolicyOutput:
        ...

    def configure_runtime(
        self,
        *,
        yaw_bias_rad: float,
        live_ref_timeout_s: float,
        idle_frame_start: int,
        source_blend_duration_s: float,
        source_kind: str = "legacy",
        status_timeout_s: float = 0.2,
    ) -> None:
        ...

    def poll_source_snapshot(self) -> JoinedSourceSnapshot:
        ...

    def open_live_reference_gate(
        self,
        *,
        source_epoch: int,
        minimum_source_frame_index: int,
        reset_yaw: bool,
        arm_proof: ArmGateProof | None,
    ) -> bool:
        ...

    def close_live_reference_gate(
        self,
        *,
        hold_last_reference: bool,
        rearm_required: bool,
        preserve_pending_yaw: bool = False,
    ) -> None:
        ...

    def reset_xsens_alignment(
        self,
        context: Literal["fresh", "same_epoch_hold", "rearm_hold"],
    ) -> bool:
        ...

    @property
    def source_blend_active(self) -> bool:
        ...

    @property
    def pending_yaw_offset(self) -> float | None:
        ...

    @property
    def armed_source_epoch(self) -> int | None:
        ...

    def has_fresh_live_reference(self, timeout_s: float | None = None) -> bool:
        ...

    def reset_yaw_alignment(self) -> None:
        ...


class SonicTeleopState(
    RobotControlState,
    EntryFrameProvider,
    RunningFrameProvider,
):
    """Named-joint SONIC policy state with optional PICO gripper control."""

    def __init__(
        self,
        name: str,
        state_id: int,
        policy: ResourceHandle[SonicPolicy],
        *,
        operator_prompt: str = PICO_OPERATOR_PROMPT,
        require_live_reference: bool = False,
        yaw_bias_rad: float = math.pi / 2.0,
        live_reference_timeout_s: float = 0.5,
        idle_frame_start: int = 3509,
        source_blend_seconds: float = 0.4,
        hardware_gripper: bool = False,
        gripper_input_timeout_s: float = 0.2,
        gripper_release_threshold: float = 0.05,
        gripper_left_bus: int = 5,
        gripper_right_bus: int = 6,
        gripper_can_id: int = 1,
        gripper_kp: float = 20.0,
        gripper_kd: float = 1.0,
    ) -> None:
        super().__init__(name, state_id, resources=(policy,))
        if not isinstance(operator_prompt, str) or not operator_prompt.strip():
            raise ValueError("operator_prompt must be a non-empty string")
        self.operator_prompt = operator_prompt
        if gripper_input_timeout_s <= 0.0:
            raise ValueError("gripper_input_timeout_s must be positive")
        self._policy = policy
        self.require_live_reference = bool(require_live_reference)
        self.yaw_bias_rad = float(yaw_bias_rad)
        self.live_reference_timeout_s = float(live_reference_timeout_s)
        self.idle_frame_start = int(idle_frame_start)
        self.source_blend_seconds = float(source_blend_seconds)
        self.hardware_gripper = bool(hardware_gripper)
        self.gripper_input_timeout_s = float(gripper_input_timeout_s)
        self.gripper_release_threshold = float(
            np.clip(gripper_release_threshold, 0.0, 1.0)
        )
        self._left_bus = int(gripper_left_bus)
        self._right_bus = int(gripper_right_bus)
        self._gripper_can_id = int(gripper_can_id)
        self._gripper_kp = float(gripper_kp)
        self._gripper_kd = float(gripper_kd)
        self._validate_config()
        self._last_running_frame: Optional[MotorFrame] = None
        self._policy_logger_bound = False

        self._gripper_session_active = False
        self._gripper_armed = False
        self._left_trigger = 0.0
        self._right_trigger = 0.0
        self._left_trigger_at: Optional[float] = None
        self._right_trigger_at: Optional[float] = None
        self._gripper_wait_reason: Optional[str] = None
        self._stale_sides: set[str] = set()
        self._gripper_subscriptions = []
        self._gripper_publisher = None
        self._gripper_available = not self.hardware_gripper

    @property
    def policy(self) -> SonicPolicy:
        return self._policy.get()

    def on_bind(self, ctx: RobotControlContext) -> None:
        if not self.hardware_gripper:
            return
        packet_type = getattr(
            bxi_msg,
            "CANFDPacket",
            getattr(bxi_msg, "CanfdPacket", None),
        )
        if packet_type is None:
            self.logger.error("SONIC夹爪不可用：缺少communication.msg.CANFDPacket")
            return
        qos = QoSProfile(depth=1)
        self._gripper_subscriptions = [
            ctx.ros_node.create_subscription(
                Float32,
                "pico/left_trigger",
                self._left_trigger_callback,
                qos,
            ),
            ctx.ros_node.create_subscription(
                Float32,
                "pico/right_trigger",
                self._right_trigger_callback,
                qos,
            ),
        ]
        self._gripper_publisher = ctx.ros_node.create_publisher(
            packet_type,
            "canfd_packet/tx",
            QoSProfile(depth=100),
        )
        self._gripper_available = True

    def on_unbind(self, ctx: RobotControlContext) -> None:
        for subscription in self._gripper_subscriptions:
            ctx.ros_node.destroy_subscription(subscription)
        self._gripper_subscriptions.clear()
        if self._gripper_publisher is not None:
            ctx.ros_node.destroy_publisher(self._gripper_publisher)
            self._gripper_publisher = None

    def _validate_config(self) -> None:
        if min(self._left_bus, self._right_bus, self._gripper_can_id) < 0:
            raise ValueError("gripper bus and CAN IDs must be non-negative")
        finite_values = (
            self.yaw_bias_rad,
            self.live_reference_timeout_s,
            self.source_blend_seconds,
            self._gripper_kp,
            self._gripper_kd,
        )
        if not all(math.isfinite(value) for value in finite_values):
            raise ValueError("SONIC numeric parameters must be finite")
        if self.live_reference_timeout_s <= 0.0:
            raise ValueError("live_reference_timeout_s must be positive")
        if self.idle_frame_start < 0:
            raise ValueError("idle_frame_start must be non-negative")
        if self.source_blend_seconds < 0.0:
            raise ValueError("source_blend_seconds must be non-negative")

    def is_available(self, ctx: RobotControlContext) -> bool:
        if not self._gripper_available:
            return False
        if self._policy.status != "ready":
            return True
        return not self.require_live_reference or self.policy.has_fresh_live_reference(
            self.live_reference_timeout_s
        )

    def _prepare_policy(
        self,
        ctx: RobotControlContext,
        *,
        source_kind: str = "legacy",
        status_timeout_s: float = 0.2,
        seed_target_from_robot: bool = False,
    ) -> None:
        if not self._policy_logger_bound:
            self.policy.bind_logger(self.logger)
            self._policy_logger_bound = True
        self.policy.configure_runtime(
            yaw_bias_rad=self.yaw_bias_rad,
            live_ref_timeout_s=self.live_reference_timeout_s,
            idle_frame_start=self.idle_frame_start,
            source_blend_duration_s=self.source_blend_seconds,
            source_kind=source_kind,
            status_timeout_s=status_timeout_s,
        )
        self.policy.reset(
            ctx.inference_frame,
            seed_target_from_robot=seed_target_from_robot,
        )
        self._last_running_frame = None
        self._start_gripper_session()

    def on_prepare(
        self,
        ctx: RobotControlContext,
        from_state: StateBehavior[RobotControlContext],
    ) -> None:
        self._prepare_policy(ctx)

    def get_entry_frame(self, ctx: RobotControlContext) -> MotorFrame:
        return self._motor_frame_from_target(ctx, self.policy.output.joints)

    def sample_running_frame(
        self,
        ctx: RobotControlContext,
        dt: float,
        *,
        advance: bool,
    ) -> MotorFrame:
        if not advance:
            return self._last_running_frame or self.get_entry_frame(ctx)
        output = self.policy.step(ctx.inference_frame, dt, advance=True)
        frame = self._motor_frame_from_target(ctx, output.joints)
        self._last_running_frame = frame
        return frame

    def on_enter(self, ctx: RobotControlContext) -> None:
        mode = "SONIC遥操（夹爪）" if self.hardware_gripper else "SONIC遥操"
        self.logger.info(f"{mode}已启动；{self.operator_prompt}")

    def on_exit(self, ctx: RobotControlContext) -> None:
        self._gripper_session_active = False
        self._gripper_armed = False
        self._stale_sides.clear()

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        # if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
        #     ctx.request_state(
        #         "com.bxi.basic_actions/zero_torque",
        #         trigger="sonic_orientation_safety",
        #     )
        #     return
        self._apply_frame(ctx, self.sample_running_frame(ctx, dt, advance=True))
        self._update_gripper()

    def on_action(self, ctx: RobotControlContext, action_name: str) -> bool:
        if action_name != "reset_alignment":
            return False
        self.policy.reset_yaw_alignment()
        return True

    @staticmethod
    def _valid_trigger(value: object) -> Optional[float]:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(result):
            return None
        return float(np.clip(result, 0.0, 1.0))

    def _left_trigger_callback(self, msg: Float32) -> None:
        value = self._valid_trigger(msg.data)
        if value is not None and self._gripper_session_active:
            self._left_trigger = value
            self._left_trigger_at = time.monotonic()

    def _right_trigger_callback(self, msg: Float32) -> None:
        value = self._valid_trigger(msg.data)
        if value is not None and self._gripper_session_active:
            self._right_trigger = value
            self._right_trigger_at = time.monotonic()

    def _start_gripper_session(self) -> None:
        if not self.hardware_gripper:
            return
        self._left_trigger = self._right_trigger = 0.0
        self._left_trigger_at = self._right_trigger_at = None
        self._gripper_armed = False
        self._gripper_session_active = True
        self._gripper_wait_reason = None
        self._stale_sides.clear()

    def _trigger_fresh(self, timestamp: Optional[float], now: float) -> bool:
        return (
            timestamp is not None
            and 0.0 <= now - timestamp <= self.gripper_input_timeout_s
        )

    def _try_arm_gripper(self, now: float) -> bool:
        if self._gripper_armed:
            return True
        fresh = self._trigger_fresh(self._left_trigger_at, now) and self._trigger_fresh(
            self._right_trigger_at, now
        )
        if not fresh:
            self._log_gripper_wait("input", "SONIC夹爪等待PICO trigger新数据")
            return False
        if (
            max(self._left_trigger, self._right_trigger)
            > self.gripper_release_threshold
        ):
            self._log_gripper_wait("release", "SONIC夹爪等待左右trigger松开")
            return False
        for bus in (self._left_bus, self._right_bus):
            self._gripper_publisher.publish(
                BxiMotor.build_motor_packet(
                    bus, self._gripper_can_id, BxiMotor.enter_motor_mode()
                )
            )
        self._gripper_armed = True
        self._gripper_wait_reason = None
        self.logger.info("SONIC夹爪已解锁")
        return True

    def _log_gripper_wait(self, reason: str, message: str) -> None:
        if self._gripper_wait_reason != reason:
            self._gripper_wait_reason = reason
            self.logger.warning(message)

    def _publish_gripper(self, bus: int, trigger: float) -> None:
        command = JointControl(
            p_des=float((1.0 - trigger) * 0.5 - 0.1),
            kp=self._gripper_kp,
            kd=self._gripper_kd,
        )
        data = BxiMotor.pack_cmd(
            command,
            p_range=(-12.5, 12.5),
            v_range=(-45.0, 45.0),
            t_range=(-40.0, 40.0),
            kp_range=(0.0, 500.0),
            kd_range=(0.0, 5.0),
        )
        self._gripper_publisher.publish(
            BxiMotor.build_motor_packet(bus, self._gripper_can_id, data)
        )

    def _update_gripper(self) -> None:
        if not self.hardware_gripper or not self._gripper_session_active:
            return
        now = time.monotonic()
        if not self._try_arm_gripper(now):
            return
        stale = {
            side
            for side, timestamp in (
                ("left", self._left_trigger_at),
                ("right", self._right_trigger_at),
            )
            if not self._trigger_fresh(timestamp, now)
        }
        newly_stale = stale - self._stale_sides
        if newly_stale:
            self.logger.warning(
                "SONIC夹爪trigger断流：" + ",".join(sorted(newly_stale)) + "；保持最后位置"
            )
        self._stale_sides = stale
        if stale:
            return
        self._publish_gripper(self._left_bus, self._left_trigger)
        self._publish_gripper(self._right_bus, self._right_trigger)


class XsensPhase(str, Enum):
    WAITING_FOR_DATA = "WAITING_FOR_DATA"
    READY = "READY"
    LIVE = "LIVE"


class XsensLinkStatus(str, Enum):
    FRESH = "FRESH"
    HOLD = "HOLD"
    HOLD_REARM_REQUIRED = "HOLD_REARM_REQUIRED"


class XsensHoldCause(str, Enum):
    STATUS_HEARTBEAT = "STATUS_HEARTBEAT"
    PRODUCER_STALE = "PRODUCER_STALE"


@dataclass(frozen=True)
class PendingArmRequest:
    command_id: int
    target_source_epoch: int
    requested_arm_epoch: int
    pre_status_sequence: int
    deadline_monotonic: float
    acknowledged_frame_index: int | None = None


XSENS_READY_PROMPT = (
    "Xsens READY — release controls, then press LT+RT+Y to request LIVE"
)
XSENS_REARM_READY_PROMPT = (
    "Xsens new session READY — release controls, then press LT+RT+Y to re-arm"
)


class XsensSonicTeleopState(SonicTeleopState):
    """Single framework state owning the guarded Xsens LIVE lifecycle."""

    def __init__(
        self,
        name: str,
        state_id: int,
        policy: ResourceHandle[SonicPolicy],
        *,
        operator_prompt: str,
        require_live_reference: bool = False,
        manual_live_enable: bool = True,
        manual_enable_slot: str = "btn_10",
        manual_enable_neutral_value: int = 0,
        seed_entry_from_robot: bool = True,
        hold_last_live_reference: bool = True,
        auto_resume_same_epoch: bool = True,
        rearm_on_source_epoch_change: bool = True,
        status_timeout_s: float = 0.2,
        arm_ack_timeout_s: float = 0.5,
        arm_command_topic: str = "sonic/xsens_arm_command",
        command_id_factory: Callable[[], int] | None = None,
        yaw_bias_rad: float = 1.57079632679,
        live_reference_timeout_s: float = 0.5,
        idle_frame_start: int = 3509,
        source_blend_seconds: float = 0.4,
        hardware_gripper: bool = False,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        compatibility = (
            ("require_live_reference", require_live_reference, False),
            ("manual_live_enable", manual_live_enable, True),
            ("seed_entry_from_robot", seed_entry_from_robot, True),
            ("hold_last_live_reference", hold_last_live_reference, True),
            ("auto_resume_same_epoch", auto_resume_same_epoch, True),
            (
                "rearm_on_source_epoch_change",
                rearm_on_source_epoch_change,
                True,
            ),
            ("hardware_gripper", hardware_gripper, False),
        )
        for option, actual, approved in compatibility:
            if actual is not approved:
                raise ValueError(f"{option} must be {approved!r}")
        try:
            neutral_value = operator.index(manual_enable_neutral_value)
        except TypeError as exc:
            raise ValueError(
                "manual_enable_neutral_value must be 0"
            ) from exc
        if isinstance(manual_enable_neutral_value, bool) or neutral_value != 0:
            raise ValueError("manual_enable_neutral_value must be 0")
        if not isinstance(manual_enable_slot, str) or not manual_enable_slot:
            raise ValueError("manual_enable_slot must be a non-empty string")
        if not isinstance(arm_command_topic, str) or not arm_command_topic:
            raise ValueError("arm_command_topic must be a non-empty string")
        if not callable(monotonic):
            raise ValueError("monotonic must be callable")
        if command_id_factory is not None and not callable(command_id_factory):
            raise ValueError("command_id_factory must be callable")
        if not math.isfinite(status_timeout_s) or status_timeout_s <= 0.0:
            raise ValueError("status_timeout_s must be positive and finite")
        if not math.isfinite(arm_ack_timeout_s) or arm_ack_timeout_s <= 0.0:
            raise ValueError("arm_ack_timeout_s must be positive and finite")

        super().__init__(
            name,
            state_id,
            policy,
            operator_prompt=operator_prompt,
            require_live_reference=False,
            yaw_bias_rad=yaw_bias_rad,
            live_reference_timeout_s=live_reference_timeout_s,
            idle_frame_start=idle_frame_start,
            source_blend_seconds=source_blend_seconds,
            hardware_gripper=False,
        )
        self.manual_live_enable = True
        self.manual_enable_slot = manual_enable_slot
        self.manual_enable_neutral_value = 0
        self.seed_entry_from_robot = True
        self.hold_last_live_reference = True
        self.auto_resume_same_epoch = True
        self.rearm_on_source_epoch_change = True
        self.status_timeout_s = float(status_timeout_s)
        self.arm_ack_timeout_s = float(arm_ack_timeout_s)
        self.arm_command_topic = arm_command_topic
        self._command_id_factory = command_id_factory or (
            lambda: secrets.randbits(63)
        )
        self._monotonic = monotonic
        self._issued_command_ids: set[int] = set()
        self._arm_command_publisher = None
        self._recovery_frame_index: int | None = None
        self._reset_phase_session()

    @property
    def phase(self) -> XsensPhase:
        return self._phase

    @property
    def link_status(self) -> XsensLinkStatus | None:
        return self._link_status

    @property
    def hold_cause(self) -> XsensHoldCause | None:
        return self._hold_cause

    @property
    def current_source_epoch(self) -> int | None:
        return self._current_source_epoch

    @property
    def pending_source_epoch(self) -> int | None:
        return self._pending_source_epoch

    @property
    def ready_frames(self) -> int:
        return self._ready_frames

    @property
    def rearm_ready(self) -> bool:
        return self._rearm_ready

    @property
    def arm_pending(self) -> bool:
        return (
            self._pending_request is not None
            and self._pending_request.requested_arm_epoch != 0
            and self._link_status is not XsensLinkStatus.HOLD_REARM_REQUIRED
        )

    @property
    def rearm_pending(self) -> bool:
        return (
            self._pending_request is not None
            and self._pending_request.requested_arm_epoch != 0
            and self._link_status is XsensLinkStatus.HOLD_REARM_REQUIRED
        )

    @property
    def disarm_pending(self) -> bool:
        return (
            self._disarm_retry_epoch is not None
            or (
                self._pending_request is not None
                and self._pending_request.requested_arm_epoch == 0
            )
        )

    @property
    def neutral_latch_open(self) -> bool:
        return self._neutral_latch_open

    @property
    def latest_status(self) -> XsensStatusSnapshot | None:
        return self._latest_status

    @property
    def is_entered(self) -> bool:
        return self._entered

    def on_bind(self, ctx: RobotControlContext) -> None:
        super().on_bind(ctx)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._arm_command_publisher = ctx.ros_node.create_publisher(
            Int64MultiArray,
            self.arm_command_topic,
            qos,
        )

    def on_unbind(self, ctx: RobotControlContext) -> None:
        if self._arm_command_publisher is not None:
            ctx.ros_node.destroy_publisher(self._arm_command_publisher)
            self._arm_command_publisher = None
        super().on_unbind(ctx)

    def on_prepare(
        self,
        ctx: RobotControlContext,
        from_state: StateBehavior[RobotControlContext],
    ) -> None:
        self._prepare_policy(
            ctx,
            source_kind="xsens",
            status_timeout_s=self.status_timeout_s,
            seed_target_from_robot=True,
        )
        self._reset_phase_session()

    def _reset_phase_session(self) -> None:
        self._phase = XsensPhase.WAITING_FOR_DATA
        self._link_status = None
        self._hold_cause = None
        self._current_source_epoch = None
        self._pending_source_epoch = None
        self._ready_frames = 0
        self._rearm_ready = False
        self._pending_request = None
        self._disarm_retry_epoch = None
        self._neutral_latch_open = False
        self._latest_status = None
        self._latest_status_sequence = 0
        self._entered = False
        self._diagnostic_key = None
        self._prompt_eligibility = None
        self._recovery_frame_index = None
        self._last_ack_frame_floor = None
        self._last_ack_floor_epoch = None
        self._last_recovery_frame_floor = None
        self._last_recovery_floor_epoch = None

    def on_enter(self, ctx: RobotControlContext) -> None:
        self._entered = True
        self._neutral_latch_open = self._runtime_exact_zero(
            ctx.remote_slot_value(self.manual_enable_slot)
        )
        self.logger.info(f"SONIC Xsens遥操已启动；{self.operator_prompt}")

    def on_exit(self, ctx: RobotControlContext) -> None:
        was_entered = self._entered
        self._entered = False
        try:
            if was_entered and self._current_source_epoch is not None:
                try:
                    self._publish_command(
                        target_source_epoch=self._current_source_epoch,
                        requested_arm_epoch=0,
                        pre_status_sequence=self._latest_status_sequence,
                        now=self._monotonic(),
                    )
                except Exception as exc:
                    self.logger.warning(
                        f"Xsens exit disarm failed: {exc!r}"
                    )
        finally:
            try:
                self.policy.close_live_reference_gate(
                    hold_last_reference=False,
                    rearm_required=False,
                )
            finally:
                self._pending_request = None
                self._disarm_retry_epoch = None
                self._neutral_latch_open = False
                self._prompt_eligibility = None
                self._last_ack_frame_floor = None
                self._last_ack_floor_epoch = None
                self._last_recovery_frame_floor = None
                self._last_recovery_floor_epoch = None
                super().on_exit(ctx)

    def _draw_command_id(self) -> int:
        for _ in range(32):
            candidate = self._command_id_factory()
            if isinstance(candidate, bool):
                continue
            try:
                value = operator.index(candidate)
            except TypeError:
                continue
            if 0 < value < 2**63 and value not in self._issued_command_ids:
                self._issued_command_ids.add(value)
                return value
        raise RuntimeError("failed to draw a unique command ID in 32 attempts")

    def _publish_command(
        self,
        *,
        target_source_epoch: int,
        requested_arm_epoch: int,
        pre_status_sequence: int,
        now: float,
    ) -> PendingArmRequest | None:
        command_id = self._draw_command_id()
        message = Int64MultiArray()
        message.layout.dim = []
        message.layout.data_offset = 0
        message.data = [
            command_id,
            int(target_source_epoch),
            int(requested_arm_epoch),
        ]
        try:
            self._arm_command_publisher.publish(message)
        except Exception as exc:
            self.logger.warning(
                "Xsens command publish failed "
                f"command_id={command_id} target={target_source_epoch} "
                f"requested={requested_arm_epoch}: {exc!r}"
            )
            return None
        return PendingArmRequest(
            command_id=command_id,
            target_source_epoch=int(target_source_epoch),
            requested_arm_epoch=int(requested_arm_epoch),
            pre_status_sequence=int(pre_status_sequence),
            deadline_monotonic=now + self.arm_ack_timeout_s,
        )

    def _request_disarm(
        self,
        *,
        target_source_epoch: int,
        pre_status_sequence: int,
        now: float,
    ) -> bool:
        epoch = int(target_source_epoch)
        self._pending_request = None
        self._disarm_retry_epoch = epoch
        request = self._publish_command(
            target_source_epoch=epoch,
            requested_arm_epoch=0,
            pre_status_sequence=pre_status_sequence,
            now=now,
        )
        if request is None:
            return False
        self._pending_request = request
        self._disarm_retry_epoch = None
        return True

    def _retry_disarm(
        self,
        snapshot: JoinedSourceSnapshot,
        now: float,
    ) -> None:
        epoch = self._disarm_retry_epoch
        if epoch is None:
            return
        status = snapshot.status
        pre_status_sequence = (
            self._latest_status_sequence
            if status is None
            else status.status_sequence
        )
        self._request_disarm(
            target_source_epoch=epoch,
            pre_status_sequence=pre_status_sequence,
            now=now,
        )

    @staticmethod
    def _receipt_matches(
        status: XsensStatusSnapshot,
        request: PendingArmRequest,
    ) -> bool:
        return (
            status.status_sequence > request.pre_status_sequence
            and status.last_arm_command_id == request.command_id
            and status.last_arm_target_epoch == request.target_source_epoch
            and status.last_requested_arm_epoch == request.requested_arm_epoch
            and status.accepted_arm_epoch == request.requested_arm_epoch
        )

    @staticmethod
    def _runtime_exact_zero(value: object) -> bool:
        if (
            isinstance(value, bool)
            or (
                isinstance(value, np.generic)
                and np.issubdtype(value.dtype, np.bool_)
            )
        ):
            return False
        try:
            return operator.index(value) == 0
        except TypeError:
            return False

    def _observe_manual_slot(self, value: object) -> None:
        if self._runtime_exact_zero(value):
            self._neutral_latch_open = True

    def _status_health(
        self,
        snapshot: JoinedSourceSnapshot,
        now: float,
    ) -> tuple[bool, XsensHoldCause | None]:
        status = snapshot.status
        if status is None:
            return False, XsensHoldCause.STATUS_HEARTBEAT
        local_age = now - status.received_monotonic
        if local_age < 0.0 or local_age > self.status_timeout_s:
            return False, XsensHoldCause.STATUS_HEARTBEAT
        reference = snapshot.reference
        now_ns = int(now * 1_000_000_000)
        timeout_ns = int(self.live_reference_timeout_s * 1_000_000_000)
        producer_timestamps = [status.producer_monotonic_ns]
        if reference is None or reference.producer_monotonic_ns is None:
            return False, XsensHoldCause.PRODUCER_STALE
        producer_timestamps.append(reference.producer_monotonic_ns)
        producer_ages = [now_ns - value for value in producer_timestamps]
        if (
            status.source_stale
            or any(age < 0 or age > timeout_ns for age in producer_ages)
        ):
            return False, XsensHoldCause.PRODUCER_STALE
        return True, None

    @staticmethod
    def _canonical_join(
        snapshot: JoinedSourceSnapshot,
        *,
        epoch: int,
        minimum_frame_index: int,
    ) -> bool:
        status, reference = snapshot.status, snapshot.reference
        return bool(
            status is not None
            and reference is not None
            and reference.source_epoch == epoch
            and reference.source_ready
            and reference.source_newest_frame_index is not None
            and reference.source_newest_frame_index >= minimum_frame_index
            and status.source_epoch == epoch
            and status.newest_frame_index >= reference.source_newest_frame_index
        )

    def _ready_for_manual_arm(
        self,
        snapshot: JoinedSourceSnapshot,
        epoch: int,
        now: float,
    ) -> bool:
        status = snapshot.status
        healthy, _ = self._status_health(snapshot, now)
        return bool(
            healthy
            and not self.disarm_pending
            and status is not None
            and status.source_epoch == epoch
            and status.accepted_arm_epoch == 0
            and status.ready
            and status.reference_window_ready
            and not status.source_stale
            and status.ready_frames >= 30
            and self._canonical_join(
                snapshot,
                epoch=epoch,
                minimum_frame_index=0,
            )
        )

    def _accept_epoch_edge(
        self,
        snapshot: JoinedSourceSnapshot,
        now: float,
    ) -> None:
        status = snapshot.status
        if status is None or status.source_epoch <= 0:
            return
        epoch = status.source_epoch
        self._ready_frames = status.ready_frames
        self._rearm_ready = False
        self._recovery_frame_index = None
        self._last_ack_frame_floor = None
        self._last_ack_floor_epoch = None
        self._last_recovery_frame_floor = None
        self._last_recovery_floor_epoch = None
        self._pending_request = None
        self._disarm_retry_epoch = None
        if self._current_source_epoch is None:
            self._current_source_epoch = epoch
            self._pending_source_epoch = None
            self._phase = XsensPhase.WAITING_FOR_DATA
            self._link_status = None
            self._hold_cause = None
            self._request_disarm(
                target_source_epoch=epoch,
                pre_status_sequence=status.status_sequence,
                now=now,
            )
            return
        if self._phase is XsensPhase.LIVE:
            self._current_source_epoch = epoch
            self._pending_source_epoch = epoch
            self._link_status = XsensLinkStatus.HOLD_REARM_REQUIRED
            self._hold_cause = None
            self.policy.close_live_reference_gate(
                hold_last_reference=True,
                rearm_required=True,
            )
        else:
            self._current_source_epoch = epoch
            self._pending_source_epoch = None
            self._phase = XsensPhase.WAITING_FOR_DATA
            self._link_status = None
            self._hold_cause = None
        if status.accepted_arm_epoch != 0:
            self._request_disarm(
                target_source_epoch=epoch,
                pre_status_sequence=status.status_sequence,
                now=now,
            )

    def _advance_waiting_or_ready(
        self,
        snapshot: JoinedSourceSnapshot,
        now: float,
    ) -> None:
        status = snapshot.status
        self._rearm_ready = False
        self._link_status = None
        self._hold_cause = None
        self._ready_frames = 0 if status is None else status.ready_frames
        epoch = self._current_source_epoch
        if (
            epoch is not None
            and self._ready_for_manual_arm(snapshot, epoch, now)
        ):
            self._phase = XsensPhase.READY
        else:
            self._phase = XsensPhase.WAITING_FOR_DATA

    def _pending_readiness_held(
        self,
        snapshot: JoinedSourceSnapshot,
        request: PendingArmRequest,
        now: float,
    ) -> bool:
        status = snapshot.status
        reference = snapshot.reference
        healthy, _ = self._status_health(snapshot, now)
        return bool(
            healthy
            and status is not None
            and reference is not None
            and status.source_epoch == request.target_source_epoch
            and status.accepted_arm_epoch
            in {0, request.requested_arm_epoch}
            and status.ready
            and status.reference_window_ready
            and not status.source_stale
            and status.ready_frames >= 30
            and reference.source_epoch == request.target_source_epoch
            and reference.source_ready
            and reference.source_newest_frame_index is not None
        )

    def _advance_pending_join(
        self,
        snapshot: JoinedSourceSnapshot,
        now: float,
    ) -> None:
        request = self._pending_request
        if request is None or request.requested_arm_epoch == 0:
            return
        if (
            now > request.deadline_monotonic
            or not self._pending_readiness_held(snapshot, request, now)
        ):
            self._cancel_same_epoch_request(snapshot, now)
            return

        status = snapshot.status
        assert status is not None
        if (
            request.acknowledged_frame_index is None
            and self._receipt_matches(status, request)
        ):
            request = replace(
                request,
                acknowledged_frame_index=status.newest_frame_index,
            )
            self._pending_request = request
            self._last_ack_frame_floor = status.newest_frame_index
            self._last_ack_floor_epoch = status.source_epoch
        frame_index = request.acknowledged_frame_index
        if frame_index is None:
            return
        if status.accepted_arm_epoch != request.target_source_epoch:
            return
        if not self._canonical_join(
            snapshot,
            epoch=request.target_source_epoch,
            minimum_frame_index=frame_index,
        ):
            return
        proof = ArmGateProof(
            request.command_id,
            request.target_source_epoch,
            request.requested_arm_epoch,
            request.pre_status_sequence,
        )
        if not self.policy.open_live_reference_gate(
            source_epoch=request.target_source_epoch,
            minimum_source_frame_index=frame_index,
            reset_yaw=True,
            arm_proof=proof,
        ):
            return
        self._current_source_epoch = request.target_source_epoch
        self._pending_source_epoch = None
        self._pending_request = None
        self._phase = XsensPhase.LIVE
        self._link_status = XsensLinkStatus.FRESH
        self._hold_cause = None
        self._ready_frames = status.ready_frames
        self._rearm_ready = False
        self._recovery_frame_index = None

    def _advance_live(
        self,
        snapshot: JoinedSourceSnapshot,
        now: float,
    ) -> None:
        status = snapshot.status
        if self._link_status is XsensLinkStatus.HOLD_REARM_REQUIRED:
            self._ready_frames = 0 if status is None else status.ready_frames
            epoch = self._pending_source_epoch
            self._rearm_ready = bool(
                epoch is not None
                and self._ready_for_manual_arm(snapshot, epoch, now)
            )
            return

        healthy, cause = self._status_health(snapshot, now)
        epoch = self._current_source_epoch
        current_status = bool(
            healthy
            and status is not None
            and epoch is not None
            and status.source_epoch == epoch
            and status.accepted_arm_epoch == epoch
            and status.ready
            and status.reference_window_ready
            and not status.source_stale
            and self._canonical_join(
                snapshot,
                epoch=epoch,
                minimum_frame_index=0,
            )
        )
        if self._link_status is XsensLinkStatus.FRESH:
            if current_status:
                self._ready_frames = status.ready_frames
                return
            self.policy.close_live_reference_gate(
                hold_last_reference=True,
                rearm_required=False,
            )
            self._link_status = XsensLinkStatus.HOLD
            self._hold_cause = cause or XsensHoldCause.PRODUCER_STALE
            self._recovery_frame_index = None
            self._last_recovery_frame_floor = None
            self._last_recovery_floor_epoch = None
            return

        if self._link_status is not XsensLinkStatus.HOLD:
            return
        if not current_status:
            candidate = cause or XsensHoldCause.PRODUCER_STALE
            if self._hold_cause is not XsensHoldCause.PRODUCER_STALE:
                self._hold_cause = candidate
            self._recovery_frame_index = None
            return
        self._ready_frames = status.ready_frames
        if self._hold_cause is XsensHoldCause.PRODUCER_STALE:
            if status.recovery_frames != 10:
                self._recovery_frame_index = None
                return
            if self._recovery_frame_index is None:
                self._recovery_frame_index = status.newest_frame_index
        elif self._recovery_frame_index is None:
            self._recovery_frame_index = status.newest_frame_index
        floor = self._recovery_frame_index
        assert floor is not None and epoch is not None
        self._last_recovery_frame_floor = floor
        self._last_recovery_floor_epoch = epoch
        if not self._canonical_join(
            snapshot,
            epoch=epoch,
            minimum_frame_index=floor,
        ):
            return
        if not self.policy.open_live_reference_gate(
            source_epoch=epoch,
            minimum_source_frame_index=floor,
            reset_yaw=False,
            arm_proof=None,
        ):
            return
        self._link_status = XsensLinkStatus.FRESH
        self._hold_cause = None
        self._recovery_frame_index = None

    def _advance_phase(
        self,
        snapshot: JoinedSourceSnapshot,
        now: float,
    ) -> None:
        status = snapshot.status
        expected_epoch = (
            self._pending_source_epoch
            if self._link_status is XsensLinkStatus.HOLD_REARM_REQUIRED
            else self._current_source_epoch
        )
        epoch_changed = bool(
            status is not None
            and status.source_epoch > 0
            and status.source_epoch != expected_epoch
        )
        if epoch_changed:
            self._accept_epoch_edge(snapshot, now)
        elif self._disarm_retry_epoch is not None:
            self._retry_disarm(snapshot, now)

        request = self._pending_request
        if (
            request is not None
            and request.requested_arm_epoch == 0
            and status is not None
            and self._receipt_matches(status, request)
        ):
            self._pending_request = None

        request = self._pending_request
        if request is not None and request.requested_arm_epoch != 0:
            self._advance_pending_join(snapshot, now)
            return
        if self._phase is XsensPhase.LIVE:
            self._advance_live(snapshot, now)
        else:
            self._advance_waiting_or_ready(snapshot, now)

    def _cancel_same_epoch_request(
        self,
        snapshot: JoinedSourceSnapshot,
        now: float,
    ) -> None:
        request = self._pending_request
        if request is None or request.requested_arm_epoch == 0:
            return
        status = snapshot.status
        pre_status_sequence = (
            self._latest_status_sequence
            if status is None
            else status.status_sequence
        )
        self._request_disarm(
            target_source_epoch=request.target_source_epoch,
            pre_status_sequence=pre_status_sequence,
            now=now,
        )
        self._neutral_latch_open = False
        if self._link_status is not XsensLinkStatus.HOLD_REARM_REQUIRED:
            self._phase = XsensPhase.WAITING_FOR_DATA

    def _reason_name(self, snapshot: JoinedSourceSnapshot) -> str:
        if snapshot.status is None:
            return XsensReason.NO_DATA.name
        return XsensReason(snapshot.status.reason_code).name

    def _alignment_context(
        self,
    ) -> Literal["fresh", "same_epoch_hold", "rearm_hold"] | None:
        if (
            self._phase is not XsensPhase.LIVE
            or self.arm_pending
            or self.rearm_pending
            or self.policy.source_blend_active
            or self.policy.pending_yaw_offset is not None
        ):
            return None
        return {
            XsensLinkStatus.FRESH: "fresh",
            XsensLinkStatus.HOLD: "same_epoch_hold",
            XsensLinkStatus.HOLD_REARM_REQUIRED: "rearm_hold",
        }.get(self._link_status)

    def on_action(
        self,
        ctx: RobotControlContext,
        action_name: str,
    ) -> bool:
        if action_name not in {"activate_xsens", "reset_alignment"}:
            return False
        if action_name == "reset_alignment":
            context = self._alignment_context()
            if context is None or not self.policy.reset_xsens_alignment(context):
                self.logger.warning(
                    "Xsens alignment reset rejected in "
                    f"{self._phase.value}/"
                    f"{self._link_status.value if self._link_status else 'IDLE'}"
                )
            return True

        snapshot = self.policy.poll_source_snapshot()
        now = self._monotonic()
        latch_was_open = self._neutral_latch_open
        self._neutral_latch_open = False
        if not latch_was_open:
            self.logger.warning(
                "Xsens enable rejected — observe exact btn_10=0 before pressing again"
            )
            return True

        if self._phase is XsensPhase.READY and not self.arm_pending:
            epoch = self._current_source_epoch
            rearm = False
        elif (
            self._phase is XsensPhase.LIVE
            and self._link_status is XsensLinkStatus.HOLD_REARM_REQUIRED
            and self._rearm_ready
            and not self.rearm_pending
        ):
            epoch = self._pending_source_epoch
            rearm = True
        else:
            reason = self._reason_name(snapshot)
            if (
                self._phase is XsensPhase.LIVE
                and self._link_status is XsensLinkStatus.HOLD_REARM_REQUIRED
            ):
                self.logger.warning(
                    "Xsens re-arm rejected — wait for new-session READY; "
                    f"reason={reason}"
                )
            elif self._phase is not XsensPhase.LIVE:
                self.logger.warning(
                    f"Xsens enable rejected — wait for READY; reason={reason}"
                )
            return True

        if epoch is None or not self._ready_for_manual_arm(snapshot, epoch, now):
            if rearm:
                self._rearm_ready = False
                self.logger.warning(
                    "Xsens re-arm rejected — wait for new-session READY; "
                    f"reason={self._reason_name(snapshot)}"
                )
            else:
                self._phase = XsensPhase.WAITING_FOR_DATA
                self.logger.warning(
                    "Xsens enable rejected — wait for READY; "
                    f"reason={self._reason_name(snapshot)}"
                )
            return True

        assert snapshot.status is not None
        request = self._publish_command(
            target_source_epoch=epoch,
            requested_arm_epoch=epoch,
            pre_status_sequence=snapshot.status.status_sequence,
            now=now,
        )
        if request is not None:
            self._last_ack_frame_floor = None
            self._last_ack_floor_epoch = None
            self._pending_request = request
        return True

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        now = self._monotonic()
        self._observe_manual_slot(
            ctx.remote_slot_value(self.manual_enable_slot)
        )
        snapshot = self.policy.poll_source_snapshot()
        self._latest_status = snapshot.status
        if snapshot.status is not None:
            self._latest_status_sequence = max(
                self._latest_status_sequence,
                snapshot.status.status_sequence,
            )
        self._advance_phase(snapshot, now)
        output = self.policy.step(ctx.inference_frame, dt, advance=True)
        frame = self._motor_frame_from_target(ctx, output.joints)
        self._last_running_frame = frame
        self._apply_frame(ctx, frame)
        self._emit_phase_diagnostic(snapshot, now)

    def _emit_phase_diagnostic(
        self,
        snapshot: JoinedSourceSnapshot,
        now: float,
    ) -> None:
        eligibility = None
        if (
            self._phase is XsensPhase.READY
            and self._pending_request is None
            and not self.disarm_pending
        ):
            eligibility = ("initial", self._current_source_epoch)
        elif (
            self._phase is XsensPhase.LIVE
            and self._link_status is XsensLinkStatus.HOLD_REARM_REQUIRED
            and self._rearm_ready
            and self._pending_request is None
            and not self.disarm_pending
        ):
            eligibility = ("rearm", self._pending_source_epoch)
        if eligibility != self._prompt_eligibility:
            self._prompt_eligibility = eligibility
            if eligibility is not None:
                self.logger.info(
                    XSENS_READY_PROMPT
                    if eligibility[0] == "initial"
                    else XSENS_REARM_READY_PROMPT
                )

        request = self._pending_request
        pending_kind = (
            "disarm"
            if self._disarm_retry_epoch is not None
            else None
            if request is None
            else "disarm"
            if request.requested_arm_epoch == 0
            else "rearm"
            if self._link_status is XsensLinkStatus.HOLD_REARM_REQUIRED
            else "arm"
        )
        reason = self._reason_name(snapshot)
        key = (
            self._phase,
            self._link_status,
            reason,
            pending_kind,
            self.policy.source_blend_active,
        )
        if key == self._diagnostic_key:
            return
        self._diagnostic_key = key

        status = snapshot.status
        status_age = (
            math.inf
            if status is None
            else now - status.received_monotonic
        )
        hold_cause = (
            "none" if self._hold_cause is None else self._hold_cause.value
        )
        echoed_command_id = (
            0 if status is None else status.last_arm_command_id
        )
        echoed_target_epoch = (
            0 if status is None else status.last_arm_target_epoch
        )
        pending_pre_status_sequence = (
            0 if request is None else request.pre_status_sequence
        )
        ack_floor = (
            self._last_ack_frame_floor
            if request is None or request.acknowledged_frame_index is None
            else request.acknowledged_frame_index
        )
        ack_floor_epoch = (
            self._last_ack_floor_epoch
            if request is None or request.acknowledged_frame_index is None
            else None if status is None else status.source_epoch
        )
        recovery_floor = (
            self._last_recovery_frame_floor
            if self._recovery_frame_index is None
            else self._recovery_frame_index
        )
        recovery_floor_epoch = (
            self._last_recovery_floor_epoch
            if self._recovery_frame_index is None
            else None if status is None else status.source_epoch
        )
        if self._link_status in {
            XsensLinkStatus.HOLD,
            XsensLinkStatus.HOLD_REARM_REQUIRED,
        }:
            yaw_owner = "hold"
        elif self.policy.pending_yaw_offset is not None:
            yaw_owner = "pending"
        elif self.policy.armed_source_epoch is not None:
            yaw_owner = "active"
        else:
            yaw_owner = "none"
        if (
            (request is not None and request.requested_arm_epoch != 0)
            or self._link_status is XsensLinkStatus.HOLD_REARM_REQUIRED
            or self.policy.pending_yaw_offset is not None
        ):
            yaw_mode = "reset"
        elif self._phase is XsensPhase.LIVE:
            yaw_mode = "preserve"
        else:
            yaw_mode = "none"
        alignment_context = self._alignment_context()
        self.logger.info(
            "Xsens phase="
            f"{self._phase.value} link="
            f"{self._link_status.value if self._link_status else 'IDLE'} "
            f"ready_frames={self._ready_frames} "
            f"recovery_frames={0 if status is None else status.recovery_frames} "
            f"status_age={status_age:.3f} "
            f"status_sequence={0 if status is None else status.status_sequence} "
            f"reason={reason} source_epoch="
            f"{0 if status is None else status.source_epoch} "
            f"requested_epoch="
            f"{0 if status is None else status.last_requested_arm_epoch} "
            f"accepted_epoch="
            f"{0 if status is None else status.accepted_arm_epoch} "
            f"armed_epoch={self.policy.armed_source_epoch} "
            f"pending={pending_kind or 'none'} "
            f"joined_frame="
            f"{None if snapshot.reference is None else snapshot.reference.source_newest_frame_index} "
            f"blend={self.policy.source_blend_active} "
            f"hold_cause={hold_cause} "
            f"echoed_command_id={echoed_command_id} "
            f"echoed_target_epoch={echoed_target_epoch} "
            f"pending_pre_status_sequence={pending_pre_status_sequence} "
            f"ack_floor={'none' if ack_floor is None else ack_floor} "
            f"ack_floor_epoch="
            f"{'none' if ack_floor_epoch is None else ack_floor_epoch} "
            f"recovery_floor="
            f"{'none' if recovery_floor is None else recovery_floor} "
            f"recovery_floor_epoch="
            f"{'none' if recovery_floor_epoch is None else recovery_floor_epoch} "
            f"yaw_owner={yaw_owner} yaw_mode={yaw_mode} "
            f"alignment_context={alignment_context or 'ineligible'}"
        )


__all__ = [
    "PendingArmRequest",
    "SonicTeleopState",
    "XsensHoldCause",
    "XsensLinkStatus",
    "XsensPhase",
    "XsensSonicTeleopState",
]
