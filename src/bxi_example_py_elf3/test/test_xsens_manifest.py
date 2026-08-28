from __future__ import annotations

from copy import deepcopy
import importlib
import runpy
from pathlib import Path

import pytest
import setuptools
import yaml

from bxi_example_py_elf3.framework.mod_api import StateBuildContext
from bxi_example_py_elf3.framework.runtime.mod_loader import (
    _discover_mods,
    _load_definition,
    _remove_module_prefixes,
    load_process_node_spec,
)
from bxi_example_py_elf3.framework.runtime.resource_manager import ResourceManager


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MOD_ROOT = PACKAGE_ROOT / "mods" / "com.bxi.sonic"
CONFIG_PATH = PACKAGE_ROOT / "config" / "elf3_state_machine.yaml"
LIVE_SOURCE_STATES = {"sonic_teleop", "sonic_zerolab", "sonic_xsens"}
NORMAL_STATE = "com.bxi.basic_actions/normal"
APPROVED_XSENS_ROUTES = frozenset({
    (("event", "activate_xsens"), ("from", NORMAL_STATE),
     ("to", "sonic_xsens"), ("transition", "soft_switch")),
    (("event", NORMAL_STATE), ("from", "sonic_xsens"),
     ("to", NORMAL_STATE), ("transition", "soft_switch")),
    (("event", "com.bxi.basic_actions/zero_torque"),
     ("from", "sonic_xsens"),
     ("to", "com.bxi.basic_actions/zero_torque")),
    (("event", "com.bxi.basic_actions/pd_brake"),
     ("from", "sonic_xsens"),
     ("to", "com.bxi.basic_actions/pd_brake")),
    (("event", "com.bxi.basic_actions/recover"),
     ("from", "sonic_xsens"),
     ("to", "com.bxi.basic_actions/recover"), ("transition", "soft_switch")),
})
APPROVED_XSENS_ACTIONS = frozenset({
    (("action", "activate_xsens"), ("event", "activate_xsens"),
     ("from", "sonic_xsens"),
     ("manifest", (("label", "请求Xsens实时控制"), ("ui", "play_arrow")))),
    (("action", "reset_alignment"), ("event", "reset_alignment"),
     ("from", "sonic_xsens"),
     ("manifest", (("label", "重置朝向对齐"), ("ui", "refresh")))),
})


@pytest.fixture
def manifest():
    with (MOD_ROOT / "mod.yaml").open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_xsens_nodes_are_state_scoped_and_use_exact_ports(manifest):
    source = manifest["nodes"]["xsens_source"]
    bridge = manifest["nodes"]["xsens_bridge"]
    assert source["states"] == ["sonic_xsens"]
    assert source["params"]["udp_port"] == 9763
    assert source["params"]["pose_port"] == 5559
    assert bridge["states"] == ["sonic_xsens"]
    assert bridge["params"]["out_port"] == 5557
    assert bridge["params"]["source_kind"] == "xsens"
    assert bridge["params"]["authoritative_input_window"] is True


def test_xsens_event_routes_and_actions_use_btn_10_11(manifest):
    assert manifest["events"]["activate_xsens"] == {"slot": "btn_10", "value": 11}
    assert any(
        route.get("from") == "com.bxi.basic_actions/normal"
        and route.get("event") == "activate_xsens"
        and route.get("to") == "sonic_xsens"
        for route in manifest["routes"]
    )
    assert any(
        action.get("from") == "sonic_xsens"
        and action.get("event") == "activate_xsens"
        and action.get("action") == "activate_xsens"
        for action in manifest["actions"]
    )


def test_plugin_registers_xsens_state_with_shared_policy(manifest):
    resources = ResourceManager()
    package_name = None
    try:
        discovered = _discover_mods((MOD_ROOT,))
        definition, package = _load_definition(
            discovered["com.bxi.sonic"], resources
        )
        package_name = package.__name__
        assert set(definition.state_factories) == {
            "sonic_teleop",
            "sonic_zerolab",
            "sonic_xsens",
        }

        built = {}
        for index, local_name in enumerate(
            ("sonic_teleop", "sonic_zerolab", "sonic_xsens"), start=1
        ):
            context = StateBuildContext(
                f"com.bxi.sonic/{local_name}",
                index,
                manifest["states"][local_name]["params"],
            )
            built[local_name] = definition.state_factories[local_name](context)
            context.finish()

        assert type(built["sonic_xsens"]).__name__ == "XsensSonicTeleopState"
        assert (
            built["sonic_teleop"]._policy
            is built["sonic_zerolab"]._policy
            is built["sonic_xsens"]._policy
        )
    finally:
        resources.close()
        if package_name is not None:
            _remove_module_prefixes((package_name,))


def test_xsens_state_has_every_exact_behavior_parameter(manifest):
    assert manifest["states"]["sonic_xsens"]["params"] == {
        "operator_prompt": "保持近似中立姿势，等待 Xsens READY",
        "require_live_reference": False,
        "manual_live_enable": True,
        "manual_enable_slot": "btn_10",
        "manual_enable_neutral_value": 0,
        "seed_entry_from_robot": True,
        "hold_last_live_reference": True,
        "auto_resume_same_epoch": True,
        "rearm_on_source_epoch_change": True,
        "hardware_gripper": False,
        "yaw_bias_rad": 1.57079632679,
        "live_reference_timeout_s": 0.5,
        "idle_frame_start": 3509,
        "source_blend_seconds": 0.4,
        "status_timeout_s": 0.2,
        "arm_ack_timeout_s": 0.5,
        "arm_command_topic": "sonic/xsens_arm_command",
    }


def test_xsens_source_and_bridge_have_every_exact_transport_parameter(manifest):
    assert manifest["nodes"]["xsens_source"]["params"] == {
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
    assert manifest["nodes"]["xsens_bridge"]["params"] == {
        "pico_host": "127.0.0.1",
        "pico_port": 5559,
        "input_pose_topic": "pose",
        "input_status_topic": "xsens_status",
        "out_host": "127.0.0.1",
        "out_port": 5557,
        "output_reference_topic": "smpl_ref",
        "output_status_topic": "xsens_status",
        "source_kind": "xsens",
        "authoritative_input_window": True,
        "readiness_debounce_messages": 1,
        "rate_hz": 50.0,
        "history_frames": 5,
        "max_gap_frames": 200,
        "catch_up_enabled": True,
        "stale_warning_seconds": 0.5,
    }


@pytest.mark.filterwarnings(
    "ignore:The distutils.sysconfig module is deprecated:DeprecationWarning"
)
def test_xsens_runtime_requirements_and_dynamic_entrypoints_are_importable(
    manifest, monkeypatch
):
    source = manifest["nodes"]["xsens_source"]
    bridge = manifest["nodes"]["xsens_bridge"]
    assert source["entrypoint"] == "xsens.source_node:create_node"
    assert source["runtime"] == "python"
    assert source["execution"] == "process"
    assert source["runtime_profile"] == "host_ros"
    assert source["lifecycle"] == "state"
    assert source["shutdown"] == {
        "signal": "SIGINT", "terminate_after": 3.0, "kill_after": 5.0,
    }
    assert {item["import"] for item in source["runtime_requirements"]["python"]} == {
        "numpy", "scipy", "zmq",
    }
    assert {item["package"] for item in source["runtime_requirements"]["ros"]} == {
        "rclpy", "std_msgs",
    }
    assert bridge["entrypoint"] == "pico.pose_to_smpl_ref_bridge:create_node"
    assert bridge["runtime"] == "python"
    assert bridge["execution"] == "in_process"
    assert bridge["runtime_profile"] == "host_ros"
    assert bridge["lifecycle"] == "state"
    assert bridge["depends_on"] == ["xsens_source"]
    assert {item["import"] for item in bridge["runtime_requirements"]["python"]} == {
        "zmq",
    }
    assert {item["package"] for item in bridge["runtime_requirements"]["ros"]} == {
        "rclpy", "std_msgs",
    }

    for module_name in ("numpy", "scipy", "zmq", "rclpy", "std_msgs"):
        assert importlib.import_module(module_name) is not None
    source_module = importlib.import_module("xsens.source_node")
    bridge_module = importlib.import_module("pico.pose_to_smpl_ref_bridge")
    assert callable(source_module.create_node)
    assert callable(bridge_module.create_node)

    spec, package_name = load_process_node_spec(MOD_ROOT / "mod.yaml", "xsens_source")
    try:
        assert callable(spec.factory)
        assert spec.execution == "process"
        assert spec.states == ("com.bxi.sonic/sonic_xsens",)
        assert spec.params["udp_port"] == 9763
        assert spec.params["pose_port"] == 5559
    finally:
        _remove_module_prefixes((package_name,))

    bridge_spec, package_name = load_process_node_spec(MOD_ROOT / "mod.yaml", "xsens_bridge")
    try:
        assert callable(bridge_spec.factory)
        assert bridge_spec.execution == "in_process"
        assert bridge_spec.states == ("com.bxi.sonic/sonic_xsens",)
        assert bridge_spec.params["pico_port"] == 5559
        assert bridge_spec.params["out_port"] == 5557
    finally:
        _remove_module_prefixes((package_name,))

    captured = {}
    monkeypatch.setattr(setuptools, "setup", lambda **kwargs: captured.update(kwargs))
    monkeypatch.chdir(PACKAGE_ROOT)
    runpy.run_path(str(PACKAGE_ROOT / "setup.py"), run_name="xsens_setup_probe")
    packaged = {
        Path(source).as_posix()
        for _, sources in captured["data_files"]
        for source in sources
    }
    assert {
        "mods/com.bxi.sonic/xsens/__init__.py",
        "mods/com.bxi.sonic/xsens/protocol.py",
        "mods/com.bxi.sonic/xsens/udp_receiver.py",
        "mods/com.bxi.sonic/xsens/converter.py",
        "mods/com.bxi.sonic/xsens/source_core.py",
        "mods/com.bxi.sonic/xsens/source_node.py",
    }.issubset(packaged)


def test_xsens_has_normal_zero_torque_pd_brake_and_recover_exits(manifest):
    exits = {
        (route["event"], route["to"], route.get("transition"))
        for route in manifest["routes"]
        if route["from"] == "sonic_xsens"
    }
    assert ("com.bxi.basic_actions/normal", "com.bxi.basic_actions/normal", "soft_switch") in exits
    assert ("com.bxi.basic_actions/zero_torque", "com.bxi.basic_actions/zero_torque", None) in exits
    assert ("com.bxi.basic_actions/pd_brake", "com.bxi.basic_actions/pd_brake", None) in exits
    assert ("com.bxi.basic_actions/recover", "com.bxi.basic_actions/recover", "soft_switch") in exits


def test_xsens_has_reset_alignment_action(manifest):
    matching = [
        action for action in manifest["actions"]
        if action["from"] == "sonic_xsens" and action["event"] == "reset_alignment"
    ]
    assert matching == [{
        "from": "sonic_xsens", "event": "reset_alignment", "action": "reset_alignment",
        "manifest": {"label": "重置朝向对齐", "ui": "refresh"},
    }]


def test_live_source_states_have_no_direct_cross_routes(manifest):
    assert not [
        route for route in manifest["routes"]
        if route["from"] in LIVE_SOURCE_STATES
        and route["to"] in LIVE_SOURCE_STATES
        and route["from"] != route["to"]
    ]
    entries = {
        (route["event"], route["to"])
        for route in manifest["routes"]
        if route["from"] == NORMAL_STATE
        and route["to"] in LIVE_SOURCE_STATES
    }
    assert entries == {
        ("activate", "sonic_teleop"),
        ("activate_zerolab", "sonic_zerolab"),
        ("activate_xsens", "sonic_xsens"),
    }


def _freeze_manifest_value(value):
    if isinstance(value, dict):
        return tuple(sorted(
            (key, _freeze_manifest_value(item)) for key, item in value.items()
        ))
    if isinstance(value, list):
        return tuple(_freeze_manifest_value(item) for item in value)
    return value


def _xsens_routes(manifest):
    return [
        route for route in manifest["routes"]
        if "sonic_xsens" in (route["from"], route["to"])
        or route["event"] == "activate_xsens"
    ]


def _xsens_actions(manifest):
    return [
        action for action in manifest["actions"]
        if action["from"] == "sonic_xsens"
        or action["event"] == "activate_xsens"
        or action["action"] == "activate_xsens"
    ]


def _assert_approved_xsens_wiring(manifest):
    live_state_entries = [
        route for route in manifest["routes"]
        if route["to"] in LIVE_SOURCE_STATES
    ]
    assert live_state_entries
    assert {route["from"] for route in live_state_entries} == {NORMAL_STATE}
    assert {
        _freeze_manifest_value(route) for route in _xsens_routes(manifest)
    } == APPROVED_XSENS_ROUTES
    assert {
        _freeze_manifest_value(action) for action in _xsens_actions(manifest)
    } == APPROVED_XSENS_ACTIONS


def test_xsens_wiring_routes_and_actions_match_the_approved_manifest(manifest):
    _assert_approved_xsens_wiring(manifest)


def _legacy_live_source_route_predicates_accept(manifest):
    no_direct_cross_routes = not [
        route for route in manifest["routes"]
        if route["from"] in LIVE_SOURCE_STATES
        and route["to"] in LIVE_SOURCE_STATES
        and route["from"] != route["to"]
    ]
    normal_entries = {
        (route["event"], route["to"])
        for route in manifest["routes"]
        if route["from"] == NORMAL_STATE
        and route["to"] in LIVE_SOURCE_STATES
    }
    return no_direct_cross_routes and normal_entries == {
        ("activate", "sonic_teleop"),
        ("activate_zerolab", "sonic_zerolab"),
        ("activate_xsens", "sonic_xsens"),
    }


def test_exact_xsens_wiring_invariants_reject_reviewer_mutations(manifest):
    mutated = deepcopy(manifest)
    mutated["routes"].append({
        "from": "com.bxi.basic_actions/zero_torque",
        "event": "activate_xsens",
        "to": "sonic_xsens",
        "transition": "soft_switch",
    })
    assert _legacy_live_source_route_predicates_accept(mutated)
    with pytest.raises(AssertionError):
        _assert_approved_xsens_wiring(mutated)

    extra_route = deepcopy(manifest)
    extra_route["routes"].append({
        "from": NORMAL_STATE,
        "event": "activate_xsens_fallback",
        "to": "sonic_xsens",
        "transition": "soft_switch",
    })
    with pytest.raises(AssertionError):
        _assert_approved_xsens_wiring(extra_route)

    extra_action = deepcopy(manifest)
    extra_action["actions"].append({
        "from": "sonic_xsens",
        "event": "activate_xsens_diagnostic",
        "action": "activate_xsens_diagnostic",
        "manifest": {"label": "Xsens诊断", "ui": "bug_report"},
    })
    with pytest.raises(AssertionError):
        _assert_approved_xsens_wiring(extra_action)


def test_xsens_cannot_be_the_initial_state_without_prepare_seed(manifest):
    with CONFIG_PATH.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    assert config["initial_state"] == "com.bxi.basic_actions/zero_torque"
    assert config["initial_state"] != "com.bxi.sonic/sonic_xsens"
    assert any(
        route["from"] == "com.bxi.basic_actions/normal"
        and route["to"] == "sonic_xsens"
        and route["event"] == "activate_xsens"
        for route in manifest["routes"]
    )
