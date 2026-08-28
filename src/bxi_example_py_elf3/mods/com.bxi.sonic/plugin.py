from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
    StateBuildContext,
)

from .policy import SonicTeleopPolicy
from .state import PICO_OPERATOR_PROMPT, SonicTeleopState, XsensSonicTeleopState


SONIC_POLICY = ResourceKey[SonicTeleopPolicy]("com.bxi.sonic/policy")


def _load_policy(context: ResourceLoadContext) -> SonicTeleopPolicy:
    return SonicTeleopPolicy(
        str(context.asset("assets/sonic.onnx")),
        str(context.asset("assets/stream_reference.npz")),
    )


def _build_state(
    state: StateBuildContext,
    policy,
) -> SonicTeleopState:
    return SonicTeleopState(
        state.name,
        state.state_id,
        policy,
        operator_prompt=state.string_param(
            "operator_prompt", PICO_OPERATOR_PROMPT
        ),
        require_live_reference=state.bool_param(
            "require_live_reference",
            False,
        ),
        yaw_bias_rad=state.float_param("yaw_bias_rad", 1.57079632679),
        live_reference_timeout_s=state.float_param("live_reference_timeout_s", 0.5),
        idle_frame_start=state.int_param("idle_frame_start", 3509),
        source_blend_seconds=state.float_param("source_blend_seconds", 0.4),
        hardware_gripper=state.bool_param("hardware_gripper", False),
        gripper_input_timeout_s=state.float_param(
            "gripper_input_timeout_s",
            0.2,
        ),
        gripper_release_threshold=state.float_param(
            "gripper_release_threshold",
            0.05,
        ),
        gripper_left_bus=state.int_param("gripper_left_bus", 5),
        gripper_right_bus=state.int_param("gripper_right_bus", 6),
        gripper_can_id=state.int_param("gripper_can_id", 1),
        gripper_kp=state.float_param("gripper_kp", 20.0),
        gripper_kd=state.float_param("gripper_kd", 1.0),
    )


def _build_xsens_state(
    state: StateBuildContext,
    policy,
) -> XsensSonicTeleopState:
    return XsensSonicTeleopState(
        state.name,
        state.state_id,
        policy,
        operator_prompt=state.string_param(
            "operator_prompt", "保持近似中立姿势，等待 Xsens READY"
        ),
        require_live_reference=state.bool_param("require_live_reference", False),
        manual_live_enable=state.bool_param("manual_live_enable", True),
        manual_enable_slot=state.string_param("manual_enable_slot", "btn_10"),
        manual_enable_neutral_value=state.int_param(
            "manual_enable_neutral_value", 0
        ),
        seed_entry_from_robot=state.bool_param("seed_entry_from_robot", True),
        hold_last_live_reference=state.bool_param(
            "hold_last_live_reference", True
        ),
        auto_resume_same_epoch=state.bool_param("auto_resume_same_epoch", True),
        rearm_on_source_epoch_change=state.bool_param(
            "rearm_on_source_epoch_change", True
        ),
        status_timeout_s=state.float_param("status_timeout_s", 0.2),
        arm_ack_timeout_s=state.float_param("arm_ack_timeout_s", 0.5),
        arm_command_topic=state.string_param(
            "arm_command_topic", "sonic/xsens_arm_command"
        ),
        yaw_bias_rad=state.float_param("yaw_bias_rad", 1.57079632679),
        live_reference_timeout_s=state.float_param("live_reference_timeout_s", 0.5),
        idle_frame_start=state.int_param("idle_frame_start", 3509),
        source_blend_seconds=state.float_param("source_blend_seconds", 0.4),
        hardware_gripper=state.bool_param("hardware_gripper", False),
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(SONIC_POLICY, _load_policy, policy="startup")
    policy = context.resource(SONIC_POLICY)
    return ModDefinition(
        state_factories={
            "sonic_teleop": lambda state: _build_state(state, policy),
            "sonic_zerolab": lambda state: _build_state(state, policy),
            "sonic_xsens": lambda state: _build_xsens_state(state, policy),
        }
    )


__all__ = ["SONIC_POLICY", "create_mod"]
