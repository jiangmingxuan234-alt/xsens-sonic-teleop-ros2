from types import SimpleNamespace

import pytest

from bxi_example_py_elf3.framework.runtime.controller import RobotControlFramework
from bxi_example_py_elf3.framework.runtime.state_machine import RemoteEventAdapter


EVENTS = {
    "enter_xsens": {"slot": "btn_10", "value": 11},
    "enter_zerolab": {"slot": "btn_10", "value": 4},
}


def test_remote_slot_snapshot_updates_even_when_sync_only_suppresses_events():
    adapter = RemoteEventAdapter(EVENTS)
    assert adapter.extract_events(SimpleNamespace(btn_10=11)) == ["enter_xsens"]
    assert adapter.extract_events(SimpleNamespace(btn_10=0), sync_only=True) == []
    assert adapter.remote_slot_value("btn_10") == 0


def test_remote_slot_snapshot_preserves_nonzero_intermediate_values():
    adapter = RemoteEventAdapter(EVENTS)
    adapter.extract_events(SimpleNamespace(btn_10=11))
    adapter.extract_events(SimpleNamespace(btn_10=2))
    assert adapter.remote_slot_value("btn_10") == 2
    assert adapter.extract_events(SimpleNamespace(btn_10=11)) == ["enter_xsens"]


def test_remote_slot_value_rejects_undeclared_slots():
    adapter = RemoteEventAdapter(EVENTS)
    with pytest.raises(KeyError, match="undeclared remote slot: btn_9"):
        adapter.remote_slot_value("btn_9")


def test_robot_control_framework_exposes_read_only_remote_slot_value():
    adapter = RemoteEventAdapter(EVENTS)
    adapter.extract_events(SimpleNamespace(btn_10=0), sync_only=True)
    framework = RobotControlFramework.__new__(RobotControlFramework)
    framework.remote_event_adapter = adapter
    assert framework.remote_slot_value("btn_10") == 0


def test_initial_values_remain_event_keyed_while_slot_snapshot_starts_zero():
    adapter = RemoteEventAdapter(EVENTS, initial_values={"enter_xsens": 11})
    assert adapter.remote_slot_value("btn_10") == 0
    assert adapter.extract_events(SimpleNamespace(btn_10=11)) == []
    assert adapter.extract_events(SimpleNamespace(btn_10=4)) == ["enter_zerolab"]


def test_shared_slot_keeps_independent_event_edge_predecessors():
    adapter = RemoteEventAdapter(EVENTS)
    assert adapter.extract_events(SimpleNamespace(btn_10=4)) == ["enter_zerolab"]
    assert adapter.extract_events(SimpleNamespace(btn_10=11)) == ["enter_xsens"]
    assert adapter.remote_slot_value("btn_10") == 11


def test_release_seen_during_transition_handoff_is_retained():
    adapter = RemoteEventAdapter(EVENTS)
    adapter.extract_events(SimpleNamespace(btn_10=11))
    assert adapter.extract_events(SimpleNamespace(btn_10=0), sync_only=True) == []
    assert adapter.remote_slot_value("btn_10") == 0
