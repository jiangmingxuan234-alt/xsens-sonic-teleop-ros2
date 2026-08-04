from pathlib import Path
import socket

import pytest
import rclpy
import zmq

from bxi_example_py_elf3.framework.mod_api import NodeBuildContext
from pico.pose_to_smpl_ref_bridge import SmplRefBridgeNode
from zerolab.source_node import ZeroLabSourceNode, validate_source_params


def free_port(sock_type):
    sock = socket.socket(socket.AF_INET, sock_type)
    try:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]
    finally:
        sock.close()


@pytest.fixture
def rclpy_runtime():
    started_here = not rclpy.ok()
    if started_here:
        rclpy.init(args=[])
    yield
    if started_here and rclpy.ok():
        rclpy.shutdown()


def source_context(udp_port, pose_port, node_name):
    root = Path(__file__).resolve().parents[1] / "mods" / "com.bxi.sonic"
    return NodeBuildContext(
        mod_id="com.bxi.sonic",
        node_id=f"com.bxi.sonic/{node_name}",
        node_name=node_name,
        mod_root=root,
        params={
            "udp_bind_host": "127.0.0.1",
            "udp_port": udp_port,
            "allowed_sender": "",
            "pose_host": "127.0.0.1",
            "pose_port": pose_port,
            "pose_topic": "pose",
            "rate_hz": 50.0,
            "window_frames": 10,
            "stale_seconds": 0.5,
            "record_path": "",
        },
    )


@pytest.mark.parametrize(
    ("params", "message"),
    [
        ({"mystery": 1}, "unknown"),
        ({"udp_port": 0}, "udp_port"),
        ({"udp_port": True}, "udp_port"),
        ({"pose_port": 65536}, "pose_port"),
        ({"pose_port": False}, "pose_port"),
        ({"rate_hz": 49.0}, "rate_hz"),
        ({"rate_hz": True}, "rate_hz"),
        ({"window_frames": 9}, "window_frames"),
        ({"window_frames": True}, "window_frames"),
        ({"stale_seconds": 0.0}, "stale_seconds"),
        ({"stale_seconds": False}, "stale_seconds"),
        ({"udp_bind_host": ""}, "udp_bind_host"),
        ({"pose_host": "0.0.0.0"}, "pose_host"),
        ({"pose_topic": ""}, "pose_topic"),
        ({"allowed_sender": 7}, "allowed_sender"),
        ({"record_path": Path("capture")}, "record_path"),
    ],
)
def test_source_rejects_unknown_or_unsafe_params(params, message):
    with pytest.raises(ValueError, match=message):
        validate_source_params(params)


def test_source_rejects_recording_path_inside_mod_root():
    root = Path(__file__).resolve().parents[1] / "mods" / "com.bxi.sonic"
    with pytest.raises(ValueError, match="outside"):
        validate_source_params(
            {"record_path": str(root / "capture")}, mod_root=root
        )


def test_repeated_source_enter_exit_releases_udp_and_pose_ports(rclpy_runtime):
    udp_port = free_port(socket.SOCK_DGRAM)
    pose_port = free_port(socket.SOCK_STREAM)
    first = ZeroLabSourceNode(source_context(udp_port, pose_port, "zerolab_a"))
    first.destroy_node()
    second = ZeroLabSourceNode(
        source_context(udp_port, pose_port, "zerolab_b")
    )
    second.destroy_node()


def test_pose_bind_failure_immediately_releases_udp_port(rclpy_runtime):
    udp_port = free_port(socket.SOCK_DGRAM)
    pose_port = free_port(socket.SOCK_STREAM)
    blocker_context = zmq.Context()
    blocker = blocker_context.socket(zmq.PUB)
    blocker.setsockopt(zmq.LINGER, 0)
    blocker.bind(f"tcp://127.0.0.1:{pose_port}")
    try:
        with pytest.raises(zmq.ZMQError):
            ZeroLabSourceNode(
                source_context(udp_port, pose_port, "zerolab_fail")
            )
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind(("127.0.0.1", udp_port))
        finally:
            probe.close()
    finally:
        blocker.close(linger=0)
        blocker_context.term()


def test_existing_bridge_releases_its_output_port(rclpy_runtime):
    input_port = free_port(socket.SOCK_STREAM)
    output_port = free_port(socket.SOCK_STREAM)
    params = {
        "pico_host": "127.0.0.1",
        "pico_port": input_port,
        "pico_topic": "pose",
        "out_host": "127.0.0.1",
        "out_port": output_port,
        "out_topic": "smpl_ref",
        "rate_hz": 50.0,
        "history_frames": 5,
        "max_gap_frames": 200,
        "catch_up_enabled": True,
        "stale_warning_seconds": 0.5,
    }
    root = Path(__file__).resolve().parents[1] / "mods" / "com.bxi.sonic"
    first = SmplRefBridgeNode(
        NodeBuildContext("com.bxi.sonic", "bridge_a", "bridge_a", root, params)
    )
    first.destroy_node()
    second = SmplRefBridgeNode(
        NodeBuildContext("com.bxi.sonic", "bridge_b", "bridge_b", root, params)
    )
    second.destroy_node()
