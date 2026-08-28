from pathlib import Path

import yaml

from bxi_example_py_elf3.framework.mod_api import ResourceKey, StateBuildContext
from bxi_example_py_elf3.framework.runtime.mod_loader import (
    _discover_mods,
    _load_definition,
    _remove_module_prefixes,
    load_process_node_spec,
)
from bxi_example_py_elf3.framework.runtime.resource_manager import (
    ResourceManager,
)


MOD_ROOT = Path(__file__).resolve().parents[1] / "mods" / "com.bxi.sonic"


def load_manifest():
    with (MOD_ROOT / "mod.yaml").open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_zerolab_nodes_have_distinct_upstream_and_mutually_exclusive_states():
    manifest = load_manifest()
    nodes = manifest["nodes"]
    assert set(nodes["pico_manager"]["states"]) == {"sonic_teleop"}
    assert set(nodes["smpl_bridge"]["states"]) == {"sonic_teleop"}
    assert set(nodes["zerolab_source"]["states"]) == {"sonic_zerolab"}
    assert set(nodes["zerolab_bridge"]["states"]) == {"sonic_zerolab"}
    assert nodes["zerolab_source"]["runtime"] == "python"
    assert nodes["zerolab_source"]["execution"] == "process"
    assert nodes["zerolab_source"]["runtime_profile"] == "host_ros"
    assert nodes["zerolab_source"]["params"]["udp_port"] == 18000
    assert nodes["zerolab_source"]["params"]["pose_host"] == "127.0.0.1"
    assert nodes["zerolab_source"]["params"]["pose_port"] == 5558
    assert (
        nodes["zerolab_bridge"]["entrypoint"]
        == "pico.pose_to_smpl_ref_bridge:create_node"
    )
    assert nodes["zerolab_bridge"]["depends_on"] == ["zerolab_source"]
    assert nodes["zerolab_bridge"]["params"]["pico_port"] == 5558
    assert nodes["zerolab_bridge"]["params"]["out_port"] == 5557
    assert nodes["smpl_bridge"]["params"]["out_port"] == 5557
    assert set(nodes["smpl_bridge"]["states"]).isdisjoint(
        nodes["zerolab_bridge"]["states"]
    )


def test_zerolab_event_state_and_routes_are_safe():
    manifest = load_manifest()
    assert manifest["events"]["activate_zerolab"] == {
        "slot": "btn_10",
        "value": 4,
    }
    params = manifest["states"]["sonic_zerolab"]["params"]
    assert params["require_live_reference"] is False
    assert params["hardware_gripper"] is False
    assert "T-pose" in params["operator_prompt"]
    assert "两秒" in params["operator_prompt"]
    routes = {(r["from"], r["event"], r["to"]) for r in manifest["routes"]}
    assert (
        "com.bxi.basic_actions/normal",
        "activate_zerolab",
        "sonic_zerolab",
    ) in routes
    assert (
        "sonic_zerolab",
        "com.bxi.basic_actions/normal",
        "com.bxi.basic_actions/normal",
    ) in routes
    forbidden = {
        ("sonic_teleop", "sonic_zerolab"),
        ("sonic_zerolab", "sonic_teleop"),
    }
    assert not any(
        (route["from"], route["to"]) in forbidden
        for route in manifest["routes"]
    )


class CaptureLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


def test_source_prompts_and_zerolab_availability_without_live_data():
    resources = ResourceManager()
    module_prefix = None
    try:
        discovered = _discover_mods((MOD_ROOT,))
        definition, module = _load_definition(
            discovered["com.bxi.sonic"], resources
        )
        module_prefix = module.__name__.split(".", 1)[0]
        policy_key = ResourceKey[object]("com.bxi.sonic/policy")
        assert resources.status(policy_key) == "unloaded"
        assert set(definition.state_factories) == {
            "sonic_teleop",
            "sonic_zerolab",
            "sonic_xsens",
        }

        pico_context = StateBuildContext("com.bxi.sonic/sonic_teleop", 1, {})
        pico = definition.state_factories["sonic_teleop"](pico_context)
        pico_context.finish()
        assert pico.operator_prompt == (
            "PICO同时按住A+B+X+Y请求校准，再按A+X切入实时POSE"
        )

        prompt = "请保持T-pose两秒，等待ZeroLab校准完成后再开始动作"
        zero_context = StateBuildContext(
            "com.bxi.sonic/sonic_zerolab",
            2,
            {
                "operator_prompt": prompt,
                "require_live_reference": False,
                "hardware_gripper": False,
            },
        )
        zero = definition.state_factories["sonic_zerolab"](zero_context)
        zero_context.finish()
        assert zero._policy is pico._policy
        assert zero.require_live_reference is False
        assert zero.hardware_gripper is False
        assert zero.is_available(None) is True
        logger = CaptureLogger()
        zero._bind_logger(logger)
        zero.on_enter(None)
        assert logger.messages == [f"SONIC遥操已启动；{prompt}"]
    finally:
        resources.close()
        if module_prefix is not None:
            _remove_module_prefixes((module_prefix,))


def test_process_loader_imports_zerolab_source_with_dynamic_package():
    spec, module_prefix = load_process_node_spec(
        MOD_ROOT / "mod.yaml", "zerolab_source"
    )
    try:
        assert callable(spec.factory)
        assert spec.execution == "process"
        assert spec.states == ("com.bxi.sonic/sonic_zerolab",)
        assert spec.params["udp_port"] == 18000
        assert spec.params["pose_port"] == 5558
    finally:
        _remove_module_prefixes((module_prefix,))


LEGACY_MANIFEST_SNAPSHOT = yaml.safe_load(
    r"""
nodes:
  pico_manager:
    runtime: command
    entrypoint: pico/manager_launcher.py
    interpreter: python3
    runtime_profile: pico_bootstrap
    execution: process
    lifecycle: state
    states: [sonic_teleop]
    arguments: [--manager, --num_frames_to_send, "10", --target_fps, "50"]
    environment: {PYTHONUNBUFFERED: "1"}
    manifest: {label: SONIC PICO管理器}
    runtime_requirements: {python: [], ros: [], system: []}
    restart:
      max_attempts: 3
      delay: 3.0
      non_retryable_exit_codes: [78]
    shutdown: {signal: SIGINT, terminate_after: 3.0, kill_after: 5.0}
  smpl_bridge:
    runtime: python
    entrypoint: pico.pose_to_smpl_ref_bridge:create_node
    execution: in_process
    runtime_profile: host_ros
    lifecycle: state
    states: [sonic_teleop]
    depends_on: [pico_manager]
    params:
      pico_host: 127.0.0.1
      pico_port: 5556
      pico_topic: pose
      out_host: 127.0.0.1
      out_port: 5557
      out_topic: smpl_ref
      rate_hz: 50.0
      history_frames: 5
      max_gap_frames: 200
      catch_up_enabled: true
      stale_warning_seconds: 0.5
    manifest: {label: SONIC SMPL参考桥}
    runtime_requirements:
      python: [{import: zmq}]
      ros: [{package: rclpy}, {package: std_msgs}]
      system: []
  zerolab_source:
    runtime: python
    entrypoint: zerolab.source_node:create_node
    execution: process
    runtime_profile: host_ros
    lifecycle: state
    states: [sonic_zerolab]
    params:
      udp_bind_host: 0.0.0.0
      udp_port: 18000
      allowed_sender: ""
      pose_host: 127.0.0.1
      pose_port: 5558
      pose_topic: pose
      rate_hz: 50.0
      window_frames: 10
      stale_seconds: 0.5
      record_path: ""
    manifest: {label: ZeroLab姿态源}
    runtime_requirements:
      python: [{import: numpy}, {import: scipy}, {import: zmq}]
      ros: [{package: rclpy}]
      system: []
    shutdown: {signal: SIGINT, terminate_after: 3.0, kill_after: 5.0}
  zerolab_bridge:
    runtime: python
    entrypoint: pico.pose_to_smpl_ref_bridge:create_node
    execution: in_process
    runtime_profile: host_ros
    lifecycle: state
    states: [sonic_zerolab]
    depends_on: [zerolab_source]
    params:
      pico_host: 127.0.0.1
      pico_port: 5558
      pico_topic: pose
      out_host: 127.0.0.1
      out_port: 5557
      out_topic: smpl_ref
      rate_hz: 50.0
      history_frames: 5
      max_gap_frames: 200
      catch_up_enabled: true
      stale_warning_seconds: 0.5
    manifest: {label: ZeroLab SMPL参考桥}
    runtime_requirements:
      python: [{import: zmq}]
      ros: [{package: rclpy}, {package: std_msgs}]
      system: []
events:
  activate: {slot: btn_10, value: 9}
  reset_alignment: {slot: btn_9, value: 1}
  activate_zerolab: {slot: btn_10, value: 4}
states:
  sonic_teleop:
    manifest:
      label: SONIC遥操
      priority: 840
      group: Advanced
      icon: sports_esports
      confirm: true
      confirm_message: 使用前务必查看官方WIKI，进入后先保持idle站立；若启用夹爪，请先松开左右trigger并确认周围安全
    params:
      require_live_reference: false
      yaw_bias_rad: 1.57079632679
      live_reference_timeout_s: 0.5
      idle_frame_start: 3509
      source_blend_seconds: 0.4
      hardware_gripper: false
      gripper_input_timeout_s: 0.2
      gripper_release_threshold: 0.05
      gripper_left_bus: 5
      gripper_right_bus: 6
      gripper_can_id: 1
      gripper_kp: 20.0
      gripper_kd: 1.0
  sonic_zerolab:
    manifest:
      label: SONIC ZeroLab遥操
      priority: 839
      group: Advanced
      icon: sports_esports
      confirm: true
      confirm_message: 请保持T-pose两秒，等待ZeroLab校准完成后再开始动作
    params:
      operator_prompt: 请保持T-pose两秒，等待ZeroLab校准完成后再开始动作
      require_live_reference: false
      yaw_bias_rad: 1.57079632679
      live_reference_timeout_s: 0.5
      idle_frame_start: 3509
      source_blend_seconds: 0.4
      hardware_gripper: false
      gripper_input_timeout_s: 0.2
      gripper_release_threshold: 0.05
      gripper_left_bus: 5
      gripper_right_bus: 6
      gripper_can_id: 1
      gripper_kp: 20.0
      gripper_kd: 1.0
routes:
  - {from: com.bxi.basic_actions/normal, event: activate, to: sonic_teleop, transition: soft_switch}
  - {from: sonic_teleop, event: com.bxi.basic_actions/normal, to: com.bxi.basic_actions/normal, transition: soft_switch}
  - {from: sonic_teleop, event: com.bxi.basic_actions/zero_torque, to: com.bxi.basic_actions/zero_torque}
  - {from: sonic_teleop, event: com.bxi.basic_actions/pd_brake, to: com.bxi.basic_actions/pd_brake}
  - {from: sonic_teleop, event: com.bxi.basic_actions/recover, to: com.bxi.basic_actions/recover, transition: soft_switch}
  - {from: com.bxi.basic_actions/normal, event: activate_zerolab, to: sonic_zerolab, transition: soft_switch}
  - {from: sonic_zerolab, event: com.bxi.basic_actions/normal, to: com.bxi.basic_actions/normal, transition: soft_switch}
  - {from: sonic_zerolab, event: com.bxi.basic_actions/zero_torque, to: com.bxi.basic_actions/zero_torque}
  - {from: sonic_zerolab, event: com.bxi.basic_actions/pd_brake, to: com.bxi.basic_actions/pd_brake}
  - {from: sonic_zerolab, event: com.bxi.basic_actions/recover, to: com.bxi.basic_actions/recover, transition: soft_switch}
actions:
  - from: sonic_teleop
    event: reset_alignment
    action: reset_alignment
    manifest: {label: 重置朝向对齐, ui: refresh}
  - from: sonic_zerolab
    event: reset_alignment
    action: reset_alignment
    manifest: {label: 重置朝向对齐, ui: refresh}
"""
)


def test_existing_pico_and_zerolab_manifest_sections_are_unchanged():
    manifest = load_manifest()
    xsens_routes = [
        route for route in manifest["routes"]
        if "sonic_xsens" in (route["from"], route["to"])
        or route["event"] == "activate_xsens"
    ]
    xsens_actions = [
        action for action in manifest["actions"]
        if action["from"] == "sonic_xsens"
        or action["event"] == "activate_xsens"
        or action["action"] == "activate_xsens"
    ]
    actual = {
        "nodes": {
            name: manifest["nodes"][name]
            for name in (
                "pico_manager", "smpl_bridge", "zerolab_source", "zerolab_bridge",
            )
        },
        "events": {
            name: manifest["events"][name]
            for name in ("activate", "reset_alignment", "activate_zerolab")
        },
        "states": {
            name: manifest["states"][name]
            for name in ("sonic_teleop", "sonic_zerolab")
        },
        "routes": [route for route in manifest["routes"] if route not in xsens_routes],
        "actions": [action for action in manifest["actions"] if action not in xsens_actions],
    }
    assert actual == LEGACY_MANIFEST_SNAPSHOT
    assert {yaml.safe_dump(route, sort_keys=True) for route in xsens_routes} == {
        yaml.safe_dump(route, sort_keys=True)
        for route in [
            {
                "from": "com.bxi.basic_actions/normal",
                "event": "activate_xsens",
                "to": "sonic_xsens",
                "transition": "soft_switch",
            },
            {
                "from": "sonic_xsens",
                "event": "com.bxi.basic_actions/normal",
                "to": "com.bxi.basic_actions/normal",
                "transition": "soft_switch",
            },
            {
                "from": "sonic_xsens",
                "event": "com.bxi.basic_actions/zero_torque",
                "to": "com.bxi.basic_actions/zero_torque",
            },
            {
                "from": "sonic_xsens",
                "event": "com.bxi.basic_actions/pd_brake",
                "to": "com.bxi.basic_actions/pd_brake",
            },
            {
                "from": "sonic_xsens",
                "event": "com.bxi.basic_actions/recover",
                "to": "com.bxi.basic_actions/recover",
                "transition": "soft_switch",
            },
        ]
    }
    assert {yaml.safe_dump(action, sort_keys=True) for action in xsens_actions} == {
        yaml.safe_dump(action, sort_keys=True)
        for action in [
            {
                "from": "sonic_xsens",
                "event": "activate_xsens",
                "action": "activate_xsens",
                "manifest": {"label": "请求Xsens实时控制", "ui": "play_arrow"},
            },
            {
                "from": "sonic_xsens",
                "event": "reset_alignment",
                "action": "reset_alignment",
                "manifest": {"label": "重置朝向对齐", "ui": "refresh"},
            },
        ]
    }
