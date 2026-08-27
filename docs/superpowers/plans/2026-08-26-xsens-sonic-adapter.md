# Xsens MVN to SONIC/ELF3 Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a state-scoped Xsens MVN `MXTP02` input that safely drives the existing SONIC/ELF3 retargeting policy with manual initial arm, same-epoch hold/automatic recovery, and new-epoch manual re-arm.

**Architecture:** A new Xsens process parses UDP, converts 23 segment transforms into the existing 10-row SONIC pose contract, and publishes pose plus an independently heartbeating status stream. The existing bridge gains an explicit authoritative-Xsens branch, while SONIC gains joined status/reference gating, a static canonical hold, phase-owned yaw offsets, and a dedicated `XsensSonicTeleopState`; legacy PICO and ZeroLab paths stay on their current branch. A physical-slot level snapshot and a driver-gated keyboard binding provide the exact-zero two-press operator contract.

**Tech Stack:** Python 3, NumPy/SciPy, ROS 2 `rclpy`, `std_msgs/msg/Int64MultiArray`, ZeroMQ packed messages, pytest, the repository's existing C++14 remote-controller target, yaml-cpp, ROS 2 `colcon`/CTest.

## Global Constraints

- The approved design is `docs/superpowers/specs/2026-08-25-xsens-sonic-adapter-design.md`; implementation must not weaken or silently reinterpret it.
- MVN input is UDP `127.0.0.1:9763`, 60 Hz, exactly one `MXTP02` datagram of 760 bytes, 23 FullBody segments, no props/fingers, big-endian payload, quaternion order `wxyz`.
- Xsens publishes on ZMQ `5559`; the state-scoped bridge publishes on shared SONIC ZMQ `5557`; sockets use `LINGER=0` and release deterministically.
- SONIC consumes a newest complete, strictly progressing 10-frame window at 50 Hz. The existing ONNX/RKNN models and 1770-dimensional observation contract do not change.
- Readiness requires 30 accepted frames, pelvis diameter `<= 0.15 m`, and every segment's hemisphere-aligned 95th-percentile angular deviation `<= 20 deg`; it is deliberately not a T-pose, body-angle, symmetry, height, or image similarity check.
- `sonic_xsens` remains one framework state with `WAITING_FOR_DATA`, `READY`, and `LIVE`; live link detail is `FRESH`, `HOLD`, or `HOLD_REARM_REQUIRED`.
- Initial live and each changed source epoch require an exact command/status/reference join. An ordinary same-epoch outage holds the newest human pose, keeps current robot proprioception in inference, and resumes automatically after ten post-edge frames.
- All source/alignment transitions use the existing 0.4-second smoothstep target blend. `soft_switch` remains the separate 20 ms framework transition.
- `LT + RT + Y` with `LB/RB` released emits `btn_10=11`; a second accepted press requires a physically observed exact `btn_10=0`. Keyboard `r` exists only with explicit `--keyboard` or `--driver keyboard`.
- Staleness never requests PD brake, zero torque, or a frozen raw 29-DOF command. Existing normal, recovery, PD-brake, zero-torque, PICO, ZeroLab, watchdog, and gripper behavior remains unchanged.
- Tests use synthetic anonymous packets and motion. Do not copy, hardcode, stage, or commit `/home/fazepurple/桌面/zhengbu_boy ceshi.mvnx` or any raw operator recording.
- Preserve and do not stage the pre-existing user changes listed below:

```text
src/bxi_example_py_elf3/mods/com.bxi.sonic/ZEROLAB_F2_PRO.md
src/bxi_example_py_elf3/test/test_zerolab_record_cli.py
docs/reports/2026-08-12-embodied-ai-algorithm-interview-guide-zh.md
docs/superpowers/plans/2026-08-04-zerolab-sonic-bxi-adapter.md
docs/superpowers/plans/2026-08-07-pair-body-002-temporary-60-frame-calibration.md
docs/superpowers/plans/2026-08-07-zerolab-realtime-mujoco-teleoperation.md
src/bxi_example_py_elf3/mods/com.bxi.sonic/zerolab/paired_record_cli.py
```

## Canonical Cross-Task Interfaces

The following names and types are fixed for every task in this plan.

```python
# xsens/protocol.py
@dataclass(frozen=True)
class Mxtp02Header:
    sample_counter: int
    datagram_counter: int
    item_count: int
    time_code: int
    character_id: int
    body_segment_count: int
    prop_count: int
    finger_segment_count: int
    reserved: int
    payload_size: int

@dataclass(frozen=True)
class XsensPacket:
    receive_timestamp_ns: int
    sender_address: tuple[str, int]
    header: Mxtp02Header
    segment_positions_xsens: np.ndarray       # float32 (23, 3), row = ID - 1
    segment_quat_wxyz_xsens: np.ndarray      # float32 (23, 4), row = ID - 1
    raw_payload: bytes

# xsens/converter.py
@dataclass(frozen=True)
class ConvertedXsensFrame:
    frame_index: int
    receive_timestamp_ns: int
    segment_positions_xrt: np.ndarray        # float32 (23, 3)
    segment_quat_xrt_xyzw: np.ndarray        # float32 (23, 4)
    smpl_body_pose: np.ndarray               # float32 (21, 3)
    smpl_joints: np.ndarray                  # float32 (24, 3)
    body_quat_w: np.ndarray                  # float32 (4,), wxyz
    joint_pos: np.ndarray                    # float32 (29,)

# xsens/source_core.py
class CounterKind(str, Enum):
    DUPLICATE = "duplicate"
    FORWARD = "forward"
    AMBIGUOUS = "ambiguous"
    BACKWARD = "backward"

class TimeCodeMode(str, Enum):
    UNKNOWN_OR_CONSTANT = "unknown_or_constant"
    ADVANCING = "advancing"

class AcceptClassification(str, Enum):
    INITIAL = "initial"
    FORWARD = "forward"
    DUPLICATE = "duplicate"
    CANDIDATE_STARTED = "candidate_started"
    CANDIDATE_REPLACED = "candidate_replaced"
    SENDER_INELIGIBLE = "sender_ineligible"
    EPOCH_COMMITTED = "epoch_committed"
    CONVERSION_REJECTED = "conversion_rejected"

@dataclass(frozen=True)
class CounterDelta:
    kind: CounterKind
    modular_delta: int
    missing_frames: int

@dataclass(frozen=True)
class ArmCommand:
    command_id: int
    target_source_epoch: int
    requested_arm_epoch: int

class ArmCommandClassification(str, Enum):
    ARMED = "armed"
    DISARMED = "disarmed"
    TARGET_MISMATCH = "target_mismatch"
    NOT_READY = "not_ready"
    DUPLICATE = "duplicate"
    REUSED_ID = "reused_id"
    INVALID = "invalid"

@dataclass(frozen=True)
class AcceptResult:
    accepted: bool
    classification: AcceptClassification
    epoch_changed: bool = False
    missing_frames: int = 0
    counter_kind: CounterKind | None = None

@dataclass(frozen=True)
class ArmCommandResult:
    classification: ArmCommandClassification
    first_seen: bool
    receipt_changed: bool
    publish_immediately: bool

@dataclass(frozen=True)
class SourceCoreStats:
    accepted: int = 0
    duplicates: int = 0
    ambiguous: int = 0
    backwards: int = 0
    sender_ineligible: int = 0
    reset_candidates: int = 0
    epoch_changes: int = 0
    inferred_missing_frames: int = 0
    conversion_failures: int = 0

# policy.py
@dataclass(frozen=True)
class SmplReferenceFrame:
    term1_local: np.ndarray
    root_quat: np.ndarray
    wrist: np.ndarray
    anchor_quat: np.ndarray | None = None
    frame_index: int = -1
    sequence: int = 0
    source_ready: bool = True
    producer_monotonic_ns: int | None = None
    source_epoch: int | None = None
    source_newest_frame_index: int | None = None
    received_monotonic: float = 0.0

@dataclass(frozen=True)
class XsensStatusSnapshot:
    status_sequence: int
    status_monotonic_ns: int
    source_epoch: int
    last_arm_command_id: int
    last_arm_target_epoch: int
    last_requested_arm_epoch: int
    accepted_arm_epoch: int
    producer_monotonic_ns: int
    newest_frame_index: int
    ready: bool
    reference_window_ready: bool
    source_stale: bool
    ready_frames: int
    recovery_frames: int
    reason_code: int
    received_monotonic: float

@dataclass(frozen=True)
class JoinedSourceSnapshot:
    status: XsensStatusSnapshot | None
    reference: SmplReferenceFrame | None

@dataclass(frozen=True)
class ArmGateProof:
    command_id: int
    target_source_epoch: int
    requested_arm_epoch: int
    pre_status_sequence: int

# Every component that makes a timing decision receives these callables.
# Production defaults are time.monotonic/time.monotonic_ns; tests pass one
# FakeClock so second- and nanosecond-based boundaries share one timeline.
MonotonicSeconds = Callable[[], float]
MonotonicNanoseconds = Callable[[], int]
```

The arm topic is reliable and volatile `sonic/xsens_arm_command`, type `std_msgs/msg/Int64MultiArray`, empty layout, `data=[command_id, target_source_epoch, requested_arm_epoch]`. Status and reference joining always compares exact command ID, target epoch, requested epoch, accepted epoch, strictly newer `status_sequence`, source epoch, and `source_newest_frame_index`.

Test-only helpers are created in the first task that uses them and reused by later Xsens tests:

| Helper | Exact test-only contract |
|---|---|
| `FakeClock` | Holds `now_ns`, returns `monotonic_ns()` and `monotonic()` from the same value, and advances only through `advance_ns(delta)`; no wall clock is used. |
| `FakeDatagramSocket` | Returns queued `(payload, sender)` pairs from `recvfrom`, raises `BlockingIOError` when empty, records bind/nonblocking/close calls, and never truncates its queued payload. |
| `reserve_udp_port()` | Binds a temporary localhost UDP socket to port zero, reads the assigned nonzero port, closes the temporary socket, and returns that integer for one immediate lifecycle test. |
| `reserve_tcp_port()` | Uses the same short-lived localhost reservation pattern for a nonzero TCP port; exact production ports are asserted in manifest tests and exercised only after live listeners are stopped. |
| `make_packet(sample_counter=1, time_code=100, sender=("127.0.0.1", 4000), receive_timestamp_ns=0, positions=None, quaternions_wxyz=None)` | Builds bytes with `build_mxtp02_packet`, then calls `parse_mxtp02_packet` with those exact values. |
| `make_core(...)` | Returns `tuple[XsensSourceCore, FakeClock]` with real converter, deterministic epoch draws, and approved default thresholds. |
| `feed_stable_frames(core, clock, count, epoch_counter_start=None)` | Feeds strictly increasing identity-orientation packets 1/60 second apart with fixed relative pelvis/segments; `None` continues after the core's newest frame. |
| `ready_core(epoch)` | Uses `make_core` plus exactly 30 stable frames and returns `(core, clock)` with a fresh, complete window and the supplied deterministic epoch. |
| `NodeHarness` | Injects fake receiver/core/publisher/context into `XsensSourceNode`, records parsed commands plus ordered `(topic, fields)` sends, and lets `publisher.fail_next_topic` make the next matching send return false without binding real ports. |
| `make_arm_message(command_id, target_epoch, requested_epoch)` | Returns `Int64MultiArray` with `layout.dim=[]`, `data_offset=0`, and exactly those three data values. |
| `make_xsens_pose_fields(...)` | Returns all eight exact Task 9 pose fields with ten finite rows, caller-selected indices/epoch/readiness/timestamp, identity root quaternions, and zero wrists. |
| `make_status(...)` | Returns all fifteen exact status scalars with valid dtypes and caller-selected sequence/epoch/command/authorization/freshness/progress values. |
| `BridgeHarness` | Builds `SmplRefBridgeNode` with injected ordered input messages and a fake output whose `fail_next_topic` returns false once; exposes `input_status`, `input_pose`, `tick`, and ordered `output.topics`. |
| `make_reference(...)` | Returns a finite `SmplReferenceFrame` with ten rows and caller-selected epoch/newest frame/source-ready/row marker. |
| `TestableSonicPolicy` | Test subclass that bypasses model execution and exposes `inject_status(fields: Mapping[str, np.ndarray], received_mono: float | None = None)` and `inject_reference(reference: SmplReferenceFrame, received_mono: float | None = None)` by appending status fields and the already-decoded canonical frame to the two planned receiver queues; omitted receive time uses its `FakeClock`. |
| `PolicyHarness` | Builds `TestableSonicPolicy` with `FakeClock`, synthetic offline reference arrays, and deterministic inference frames; `authorize_reference` injects matching status/reference then opens the gate. |
| `authorize_reference(policy, reference)` | Injects a fresh matching authorized status/reference, constructs the exact `ArmGateProof` for that receipt, polls once, and asserts `open_live_reference_gate` succeeds. |
| `enter_hold(policy)` | Calls `authorize_reference` with a deterministic live frame, then closes the gate with `hold_last_reference=True` and `rearm_required=False`. |
| `make_inference_frame(qpos_offset)` | Returns a complete ELF3 `InferenceFrame` whose 29 bound joint positions equal the packaged defaults plus the supplied scalar offset and whose velocities/base motion are finite zeros. |
| `StateHarness` | Supplies fake ROS publisher/context plus `TestableSonicPolicy`; can set the physical slot, inject status/reference, tick once, and return the last published three-int command. |

No helper above is installed as runtime API.

All timing APIs in this plan are dependency-injected. `SonicTeleopPolicy`
stores `_monotonic` and `_monotonic_ns` from constructor arguments and never
calls `time.monotonic()`/`time.monotonic_ns()` elsewhere. `XsensSonicTeleopState`
stores `_monotonic` from its constructor, `XsensSourceCore` stores `_clock_ns`,
and the source/bridge nodes pass one captured `now` into a tick. This is a
contract, not optional test scaffolding: exact `0.2 s`, `0.25 s`, `0.5 s`, and
`0.4 s` boundaries must all be reproducible without patching the Python `time`
module.

---

### Task 1: Add a Physical Remote-Slot Level Snapshot

**Files:**

- Modify: `src/bxi_example_py_elf3/bxi_example_py_elf3/framework/runtime/state_machine.py:35-80`
- Modify: `src/bxi_example_py_elf3/bxi_example_py_elf3/framework/runtime/controller.py:245-260`
- Modify: `src/bxi_example_py_elf3/bxi_example_py_elf3/framework/mod_api/context.py:38-104`
- Create: `src/bxi_example_py_elf3/test/test_framework_remote_slots.py`

**Interfaces:**

- Consumes: Existing `RemoteEventAdapter.extract_events(msg, sync_only=False) -> list[str]` edge behavior.
- Produces: `RemoteEventAdapter.remote_slot_value(slot: str) -> int`, `RobotControlFramework.remote_slot_value(slot: str) -> int`, and the same method on `RobotControlContext`.
- Invariant: `_last_values` remains keyed by event name; `_latest_slot_values` is independently keyed by declared physical slot.
- Compatibility: `initial_values` remains event-name keyed and initializes only edge predecessors; every declared physical slot snapshot starts at zero until the first message or `sync_only` synchronization updates it.

- [ ] **Step 1: Write failing snapshot and forwarding tests**

```python
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
```

- [ ] **Step 2: Run the tests and verify the new API is missing**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_framework_remote_slots.py -v
```

Expected: FAIL with `AttributeError: 'RemoteEventAdapter' object has no attribute 'remote_slot_value'`.

- [ ] **Step 3: Implement one snapshot per declared physical slot**

```python
self._latest_slot_values = {slot: 0 for slot in declared_slots}

def remote_slot_value(self, slot: str) -> int:
    if slot not in self._latest_slot_values:
        raise KeyError(f"undeclared remote slot: {slot}")
    return self._latest_slot_values[slot]
```

Parse every event configuration during construction, reject non-string slots exactly as today, preserve per-event `_last_values`, read every unique physical slot once at the start of `extract_events()`, and update the slot snapshot even under `sync_only=True`. Add the controller forwarding method and the `RobotControlContext` protocol declaration with the exact signature above.

- [ ] **Step 4: Run focused and framework regressions**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_framework_remote_slots.py -v
```

Expected: PASS, including unknown-slot rejection and shared-slot edge behavior.

- [ ] **Step 5: Commit the framework contract**

```bash
git add src/bxi_example_py_elf3/bxi_example_py_elf3/framework/runtime/state_machine.py src/bxi_example_py_elf3/bxi_example_py_elf3/framework/runtime/controller.py src/bxi_example_py_elf3/bxi_example_py_elf3/framework/mod_api/context.py src/bxi_example_py_elf3/test/test_framework_remote_slots.py
git commit -m "feat(framework): expose remote slot level snapshots"
```

### Task 2: Gate the Development Keyboard Binding by Explicit Driver Selection

**Files:**

- Modify: `src/remote_controller/include/remote_controller/config.hpp:90-95,198`
- Modify: `src/remote_controller/src/config.cpp:800-835,1318-1338`
- Modify: `src/remote_controller/src/main.cpp:18-30`
- Modify: `src/remote_controller/test/control_rules_test.cpp`

**Interfaces:**

- Consumes: Existing CLI `driver_filter`, where `--keyboard` becomes `keyboard` and `--driver <type>` uses the explicit type.
- Produces: `Binding.requires_driver_filter: std::string` and `load_remote_config(const std::string &path, const std::string &driver_filter = "")`.
- Valid filters: configured `InputDeviceConfig.type` values (`crsf`, `joystick`, `keyboard`); missing/empty means ungated.

- [ ] **Step 1: Add failing filter-selection tests**

```cpp
std::string write_filter_fixture(const std::string &required_filter)
{
    YAML::Node root = YAML::LoadFile(REMOTE_CONTROLLER_TEST_CONFIG_PATH);
    YAML::Node binding;
    binding["output"] = "btn_10=11";
    binding["requires_driver_filter"] = required_filter;
    binding["when"].push_back("keyboard.sonic_event");
    root["outputs"]["level"].push_back(binding);
    const std::string path = "/tmp/remote_controller_driver_filter_test.yaml";
    std::ofstream stream(path);
    stream << root;
    stream.close();
    return path;
}

void test_binding_driver_filter_selection()
{
    const std::string path = write_filter_fixture("keyboard");
    const RemoteConfig hardware = remote_controller::load_remote_config(
        path);
    const RemoteConfig keyboard = remote_controller::load_remote_config(
        path, "keyboard");
    const RemoteConfig joystick = remote_controller::load_remote_config(
        path, "joystick");

    const auto has_keyboard_xsens = [](const RemoteConfig &config) {
        return std::any_of(
            config.bindings.begin(), config.bindings.end(),
            [](const remote_controller::Binding &binding) {
                return binding.output == "btn_10=11" &&
                    binding.requires_driver_filter == "keyboard";
            });
    };
    expect(!has_keyboard_xsens(hardware));
    expect(has_keyboard_xsens(keyboard));
    expect(!has_keyboard_xsens(joystick));

    const auto ungated_outputs = [](const RemoteConfig &config) {
        std::vector<std::string> outputs;
        for (const auto &binding : config.bindings) {
            if (binding.requires_driver_filter.empty()) {
                outputs.push_back(binding.output);
            }
        }
        return outputs;
    };
    expect(ungated_outputs(hardware) == ungated_outputs(keyboard));
    std::remove(path.c_str());
}

void test_unknown_binding_driver_filter_is_rejected()
{
    const std::string path = write_filter_fixture("unknown_driver");
    bool threw = false;
    try {
        remote_controller::load_remote_config(path, "keyboard");
    } catch (const std::runtime_error &) {
        threw = true;
    }
    std::remove(path.c_str());
    expect(threw);
}
```

Include `<algorithm>`, `<cstdio>`, `<fstream>`, and `<yaml-cpp/yaml.h>`, and call both new functions from this file's explicit `main()`.

- [ ] **Step 2: Build the focused test and verify the overload/field is missing**

Run:

```bash
colcon build --packages-select remote_controller --cmake-args -DBUILD_TESTING=ON
```

Expected: compilation FAIL because `Binding::requires_driver_filter` and the two-argument `load_remote_config` do not exist.

- [ ] **Step 3: Implement schema validation and load-time filtering**

```cpp
struct Binding {
    std::string output;
    std::string mode;
    std::string requires_driver_filter;
    ConditionConfig when;
};

RemoteConfig load_remote_config(
    const std::string &path,
    const std::string &driver_filter = "");
```

Parse and validate every gated binding before filtering it. A nonempty required filter must occur among configured device `type` values. Load ungated bindings always; load a gated binding only when the explicit `driver_filter` equals it; an empty runtime filter excludes every gated binding. Pass `driver_filter` to `load_remote_config(config_path, driver_filter)` in `COMPublisher` while retaining the same value for `InputDeviceManager`.

- [ ] **Step 4: Run the C++ filter tests**

Run:

```bash
colcon build --packages-select remote_controller --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/remote_controller -R remote_controller_control_rules_test --output-on-failure
```

Expected: PASS; existing ungated mapping tests remain green.

- [ ] **Step 5: Commit the driver-filter schema**

```bash
git add src/remote_controller/include/remote_controller/config.hpp src/remote_controller/src/config.cpp src/remote_controller/src/main.cpp src/remote_controller/test/control_rules_test.cpp
git commit -m "feat(remote): gate bindings by explicit driver filter"
```

### Task 3: Add the `LT + RT + Y` Physical and Explicit-Keyboard Mappings

**Files:**

- Modify: `src/remote_controller/config/xbox_default.yaml`
- Modify: `src/remote_controller/test/control_rules_test.cpp`

**Interfaces:**

- Consumes: Task 2 `requires_driver_filter` semantics.
- Produces: gamepad and CRSF `btn_10=11`, exact release back to zero, and explicit-keyboard `r -> btn_10=11`.

- [ ] **Step 1: Add failing gamepad, CRSF, collision, release, and keyboard tests**

```cpp
void test_xsens_gamepad_chord_emits_btn_10_11()
{
    InputMapper mapper(remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH));
    mapper.set_signals({
        {"js.axis.5", 1.0},
        {"js.axis.4", 1.0},
        {"js.button.4", 1.0},
        {"js.button.6", 0.0},
        {"js.button.7", 0.0},
    });
    communication::msg::MotionCommands message;
    mapper.fill_message(message);
    expect(message.btn_10 == 11);
}

void test_xsens_crsf_chord_emits_btn_10_11()
{
    InputMapper mapper(remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH));
    mapper.set_signals({
        {"crsf.channel.7", 1.0},
        {"crsf.channel.3", 1.0},
        {"crsf.channel.8", -0.2},
        {"crsf.channel.9", 0.0},
    });
    communication::msg::MotionCommands message;
    mapper.fill_message(message);
    expect(message.btn_10 == 11);
}


int xsens_gamepad_value(double lt, double rt, double y, double lb, double rb)
{
    InputMapper mapper(remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH));
    mapper.set_signals({
        {"js.axis.5", lt}, {"js.axis.4", rt}, {"js.button.4", y},
        {"js.button.6", lb}, {"js.button.7", rb},
    });
    communication::msg::MotionCommands message;
    mapper.fill_message(message);
    return message.btn_10;
}

int xsens_crsf_value(
    double lt, double rt, double button_group_a, double button_group_b)
{
    InputMapper mapper(remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH));
    mapper.set_signals({
        {"crsf.channel.7", lt}, {"crsf.channel.3", rt},
        {"crsf.channel.8", button_group_a},
        {"crsf.channel.9", button_group_b},
    });
    communication::msg::MotionCommands message;
    mapper.fill_message(message);
    return message.btn_10;
}

void test_xsens_chord_guards_and_release()
{
    expect(xsens_gamepad_value(0.0, 1.0, 1.0, 0.0, 0.0) != 11);
    expect(xsens_gamepad_value(1.0, 0.0, 1.0, 0.0, 0.0) != 11);
    expect(xsens_gamepad_value(1.0, 1.0, 0.0, 0.0, 0.0) != 11);
    expect(xsens_gamepad_value(1.0, 1.0, 1.0, 1.0, 0.0) != 11);
    expect(xsens_gamepad_value(1.0, 1.0, 1.0, 0.0, 1.0) != 11);
    expect(xsens_gamepad_value(0.0, 0.0, 0.0, 0.0, 0.0) == 0);
}

void test_xsens_crsf_guards_and_release()
{
    constexpr double y = -0.2;
    constexpr double idle = 0.0;
    expect(xsens_crsf_value(1.0, 1.0, y, idle) == 11);
    expect(xsens_crsf_value(0.0, 1.0, y, idle) != 11);
    expect(xsens_crsf_value(1.0, 0.0, y, idle) != 11);
    expect(xsens_crsf_value(1.0, 1.0, idle, idle) != 11);
    expect(xsens_crsf_value(1.0, 1.0, y, -0.9) != 11); // LB
    expect(xsens_crsf_value(1.0, 1.0, y, -0.7) != 11); // RB
    expect(xsens_crsf_value(0.0, 0.0, idle, idle) == 0);
}

void test_xsens_keyboard_requires_explicit_filter_and_expires()
{
    communication::msg::MotionCommands message;
    InputMapper hardware(remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH));
    hardware.handle_keyboard_key('r');
    hardware.fill_message(message);
    expect(message.btn_10 == 0);

    InputMapper keyboard(remote_controller::load_remote_config(
        REMOTE_CONTROLLER_TEST_CONFIG_PATH, "keyboard"));
    keyboard.handle_keyboard_key('r');
    keyboard.fill_message(message);
    expect(message.btn_10 == 11);
    std::this_thread::sleep_for(std::chrono::milliseconds(210));
    keyboard.tick();
    keyboard.fill_message(message);
    expect(message.btn_10 == 0);
}
```

Include `<chrono>` and `<thread>` and call all five Xsens mapping tests from `main()`.

- [ ] **Step 2: Run the focused binary and verify value 11 is absent**

Run:

```bash
colcon build --packages-select remote_controller --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/remote_controller -R remote_controller_control_rules_test --output-on-failure
```

Expected: FAIL in `test_xsens_gamepad_chord_emits_btn_10_11` with `btn_10 == 0`.

- [ ] **Step 3: Add separate physical and gated keyboard bindings**

```yaml
sources:
  keyboard:
    signals:
      keyboard.xsens: {from: keyboard.key, key: r, hold_ms: 200}

controls:
  keyboard.xsens_event:
    type: bool
    inputs: [{source: keyboard.xsens}]

outputs:
  level:
    - output: btn_10=11
      when:
        - trigger.left_event
        - trigger.right_event
        - button.y_event
        - released: shoulder.left_event
        - released: shoulder.right_event

    - output: btn_10=11
      requires_driver_filter: keyboard
      when: [keyboard.xsens_event]
```

Keep the two value-11 bindings separate so only the keyboard branch is gated. Do not alter existing values 1 through 10; their opposite-trigger release conditions are the collision guard.

- [ ] **Step 4: Run mapping and existing chord regressions**

Run:

```bash
colcon build --packages-select remote_controller --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/remote_controller -R remote_controller_control_rules_test --output-on-failure
```

Expected: PASS for gamepad, CRSF, release, keyboard gating, and prior three-button-chord tests.

- [ ] **Step 5: Commit the operator mapping**

```bash
git add src/remote_controller/config/xbox_default.yaml src/remote_controller/test/control_rules_test.cpp
git commit -m "feat(remote): map xsens live-enable chord"
```

### Task 4: Parse and Validate the Exact `MXTP02` Wire Contract

**Files:**

- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/__init__.py`
- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/protocol.py`
- Create: `src/bxi_example_py_elf3/test/xsens_test_helpers.py`
- Create: `src/bxi_example_py_elf3/test/test_xsens_protocol.py`

**Interfaces:**

- Consumes: raw UDP bytes, monotonic receive timestamp, and `(source_ip, source_port)`.
- Produces: `parse_mxtp02_packet(payload: bytes, *, receive_timestamp_ns: int, sender_address: tuple[str, int]) -> XsensPacket`.
- Constants: `PACKET_SIZE=760`, `HEADER_SIZE=24`, `SEGMENT_COUNT=23`, `PAYLOAD_SIZE=736`, `HEADER_STRUCT=struct.Struct(">6sIBBIBBBBHH")`, `SEGMENT_STRUCT=struct.Struct(">I7f")`.

- [ ] **Step 1: Create a synthetic packet builder and failing parser tests**

```python
from collections import deque
import socket
import struct

import numpy as np


class FakeClock:
    def __init__(self, now_ns: int = 0) -> None:
        self.now_ns = int(now_ns)

    def monotonic_ns(self) -> int:
        return self.now_ns

    def monotonic(self) -> float:
        return self.now_ns / 1_000_000_000.0

    def advance_ns(self, delta: int) -> None:
        if delta < 0:
            raise ValueError("delta must be non-negative")
        self.now_ns += int(delta)


class FakeDatagramSocket:
    def __init__(self, datagrams=()) -> None:
        self.datagrams = deque(datagrams)
        self.bound = None
        self.blocking = None
        self.closed = False
        self.close_calls = 0
        self.recv_sizes = []

    def bind(self, address) -> None:
        self.bound = address

    def setblocking(self, value: bool) -> None:
        self.blocking = bool(value)

    def recvfrom(self, size: int):
        self.recv_sizes.append(size)
        if not self.datagrams:
            raise BlockingIOError
        payload, sender = self.datagrams.popleft()
        return payload, sender

    def close(self) -> None:
        self.close_calls += 1
        self.closed = True


def _reserve_port(sock_type: int) -> int:
    with socket.socket(socket.AF_INET, sock_type) as reservation:
        reservation.bind(("127.0.0.1", 0))
        return int(reservation.getsockname()[1])


def reserve_udp_port() -> int:
    return _reserve_port(socket.SOCK_DGRAM)


def reserve_tcp_port() -> int:
    return _reserve_port(socket.SOCK_STREAM)


def build_mxtp02_packet(
    *,
    sample_counter: int = 1,
    time_code: int = 100,
    identifier: bytes = b"MXTP02",
    datagram_counter: int = 0,
    item_count: int = 23,
    character_id: int = 0,
    body_segment_count: int = 23,
    prop_count: int = 0,
    finger_segment_count: int = 0,
    reserved: int = 0,
    payload_size: int = 736,
    segment_order: tuple[int, ...] = tuple(range(1, 24)),
    positions: np.ndarray | None = None,
    quaternions_wxyz: np.ndarray | None = None,
) -> bytes:
    pos = (
        np.zeros((23, 3), dtype=np.float32)
        if positions is None else np.asarray(positions, dtype=np.float32)
    )
    quat = (
        np.tile(np.array([1, 0, 0, 0], dtype=np.float32), (23, 1))
        if quaternions_wxyz is None
        else np.asarray(quaternions_wxyz, dtype=np.float32)
    )
    header = struct.pack(
        ">6sIBBIBBBBHH", identifier, sample_counter, datagram_counter,
        item_count, time_code, character_id, body_segment_count, prop_count,
        finger_segment_count, reserved, payload_size,
    )
    rows = []
    for row_index, segment_id in enumerate(segment_order):
        source_index = segment_id - 1 if 1 <= segment_id <= 23 else row_index % 23
        rows.append(struct.pack(
            ">I7f", segment_id, *pos[source_index], *quat[source_index]
        ))
    body = b"".join(rows)
    return header + body


def make_packet(
    sample_counter=1,
    time_code=100,
    sender=("127.0.0.1", 4000),
    receive_timestamp_ns=0,
    positions=None,
    quaternions_wxyz=None,
):
    payload = build_mxtp02_packet(
        sample_counter=sample_counter,
        time_code=time_code,
        positions=positions,
        quaternions_wxyz=quaternions_wxyz,
    )
    return parse_mxtp02_packet(
        payload,
        receive_timestamp_ns=receive_timestamp_ns,
        sender_address=sender,
    )
```

Put these helpers in `test/xsens_test_helpers.py`. In
`test_xsens_protocol.py`, use the following concrete validation matrix; each
row mutates exactly one wire fact and must raise `XsensProtocolError`:

```python
@pytest.mark.parametrize(
    "payload",
    [
        build_mxtp02_packet()[:-1],
        build_mxtp02_packet() + b"x",
        build_mxtp02_packet(identifier=b"BAD002"),
        build_mxtp02_packet(item_count=22),
        build_mxtp02_packet(character_id=1),
        build_mxtp02_packet(body_segment_count=22),
        build_mxtp02_packet(prop_count=1),
        build_mxtp02_packet(finger_segment_count=1),
        build_mxtp02_packet(payload_size=735),
        build_mxtp02_packet(segment_order=tuple(range(1, 23))),
        build_mxtp02_packet(segment_order=tuple(range(1, 23)) + (22,)),
        build_mxtp02_packet(segment_order=tuple(range(1, 23)) + (24,)),
    ],
)
def test_rejects_malformed_header_or_segment_set(payload):
    with pytest.raises(XsensProtocolError):
        parse_mxtp02_packet(
            payload, receive_timestamp_ns=5, sender_address=("127.0.0.1", 4000)
        )


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
def test_rejects_nonfinite_position_or_quaternion(bad):
    positions = np.zeros((23, 3), dtype=np.float32)
    positions[4, 2] = bad
    quats = np.tile(np.array([1, 0, 0, 0], np.float32), (23, 1))
    quats[7, 1] = bad
    for payload in (
        build_mxtp02_packet(positions=positions),
        build_mxtp02_packet(quaternions_wxyz=quats),
    ):
        with pytest.raises(XsensProtocolError):
            parse_mxtp02_packet(
                payload, receive_timestamp_ns=5,
                sender_address=("127.0.0.1", 4000),
            )


def test_norm_boundaries_are_closed_at_float32_wire_precision():
    for accepted in (np.float32(0.95), np.float32(1.05)):
        quats = np.zeros((23, 4), dtype=np.float32)
        quats[:, 0] = accepted
        parse_mxtp02_packet(
            build_mxtp02_packet(quaternions_wxyz=quats),
            receive_timestamp_ns=5, sender_address=("127.0.0.1", 4000),
        )
    for rejected in (
        np.nextafter(np.nextafter(np.float32(0.95), np.float32(-np.inf)),
                     np.float32(-np.inf)),
        np.nextafter(np.nextafter(np.float32(1.05), np.float32(np.inf)),
                     np.float32(np.inf)),
    ):
        quats = np.zeros((23, 4), dtype=np.float32)
        quats[:, 0] = rejected
        with pytest.raises(XsensProtocolError):
            parse_mxtp02_packet(
                build_mxtp02_packet(quaternions_wxyz=quats),
                receive_timestamp_ns=5, sender_address=("127.0.0.1", 4000),
            )
```

The two positive tests are:

```python
def test_decodes_all_header_metadata_and_freezes_arrays():
    positions = np.arange(69, dtype=np.float32).reshape(23, 3) / 10.0
    quats = np.tile(np.array([1, 0, 0, 0], np.float32), (23, 1))
    payload = build_mxtp02_packet(
        sample_counter=0x01020304, time_code=0x11223344,
        datagram_counter=7, reserved=0x1234,
        positions=positions, quaternions_wxyz=quats,
    )
    packet = parse_mxtp02_packet(
        payload, receive_timestamp_ns=987_654_321,
        sender_address=("127.0.0.1", 43123),
    )
    assert packet.header == Mxtp02Header(
        sample_counter=0x01020304, datagram_counter=7, item_count=23,
        time_code=0x11223344, character_id=0, body_segment_count=23,
        prop_count=0, finger_segment_count=0, reserved=0x1234,
        payload_size=736,
    )
    assert packet.receive_timestamp_ns == 987_654_321
    assert packet.sender_address == ("127.0.0.1", 43123)
    assert packet.raw_payload == payload
    np.testing.assert_array_equal(packet.segment_positions_xsens, positions)
    np.testing.assert_array_equal(packet.segment_quat_wxyz_xsens, quats)
    assert not packet.segment_positions_xsens.flags.writeable
    assert not packet.segment_quat_wxyz_xsens.flags.writeable
    with pytest.raises(ValueError, match="read-only"):
        packet.segment_positions_xsens[0, 0] = 9.0


def test_segment_rows_are_id_indexed_independent_of_arrival_order():
    positions = np.arange(69, dtype=np.float32).reshape(23, 3)
    quats = np.tile(np.array([1, 0, 0, 0], np.float32), (23, 1))
    packet = parse_mxtp02_packet(
        build_mxtp02_packet(
            segment_order=tuple(reversed(range(1, 24))),
            positions=positions, quaternions_wxyz=quats,
        ),
        receive_timestamp_ns=1, sender_address=("127.0.0.1", 4000),
    )
    np.testing.assert_array_equal(packet.segment_positions_xsens, positions)
    np.testing.assert_array_equal(packet.segment_quat_wxyz_xsens, quats)
```

- [ ] **Step 2: Run the parser tests and verify the module is missing**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_protocol.py -v
```

Expected: collection FAIL with `ModuleNotFoundError: No module named 'xsens'`.

- [ ] **Step 3: Implement immutable parsing with all-or-nothing validation**

```python
def parse_mxtp02_packet(payload, *, receive_timestamp_ns, sender_address):
    if len(payload) != PACKET_SIZE:
        raise XsensProtocolError(f"MXTP02 packet must be {PACKET_SIZE} bytes")
    values = HEADER_STRUCT.unpack_from(payload, 0)
    if values[0] != b"MXTP02":
        raise XsensProtocolError("identifier must be MXTP02")
```

Decode 23 items into preallocated float32 arrays indexed by ID. Reject before
constructing `XsensPacket` unless IDs are exactly `1..23`, every value is
finite, and every raw quaternion norm lies in the closed interval `[0.95,
1.05]` at wire precision. Compare float64-computed norms against one outward
float32 ULP (`nextafter(float32(0.95), -inf)` through
`nextafter(float32(1.05), +inf)`) solely to represent the approved inclusive
boundaries after float32 encoding; values two ULPs outside must fail. Call
`setflags(write=False)` on both decoded arrays before constructing the frozen
dataclass. Preserve `datagram_counter` without inventing an unverified rejection
rule; do not normalize in the parser.

- [ ] **Step 4: Run the complete protocol matrix**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_protocol.py -v
```

Expected: PASS for the exact 760-byte fixture and every malformed-packet case.

- [ ] **Step 5: Commit the wire protocol**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/__init__.py src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/protocol.py src/bxi_example_py_elf3/test/xsens_test_helpers.py src/bxi_example_py_elf3/test/test_xsens_protocol.py
git commit -m "feat(xsens): parse MXTP02 datagrams"
```

### Task 5: Own UDP 9763 and Timestamp Complete Datagrams

**Files:**

- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/udp_receiver.py`
- Create: `src/bxi_example_py_elf3/test/test_xsens_udp_receiver.py`

**Interfaces:**

- Consumes: Task 4 `PACKET_SIZE` and an injectable `clock_ns: Callable[[], int]`.
- Produces: `ReceivedDatagram`, `ReceiverStats`, and `XsensUdpReceiver.poll()`, `.drain(limit=256)`, `.close()`.
- Boundary: this layer filters only allowed sender IP and exact length; parser-valid sender-tuple locking belongs to `XsensSourceCore`.

```text
XsensUdpReceiver(
  bind_host: str = "0.0.0.0", port: int = 9763,
  allowed_sender_host: str | None = "127.0.0.1", *,
  clock_ns: Callable[[], int] = time.monotonic_ns,
  sock: socket.socket | None = None
)
```

- [ ] **Step 1: Write failing fake-socket and real-port lifecycle tests**

```python
def test_receiver_filters_ip_and_uses_monotonic_receive_timestamp():
    sock = FakeDatagramSocket([
        (b"x" * 760, ("10.0.0.4", 4000)),
        (b"y" * 760, ("127.0.0.1", 4001)),
    ])
    receiver = XsensUdpReceiver(
        allowed_sender_host="127.0.0.1",
        clock_ns=lambda: 123_456_789,
        sock=sock,
    )
    received = receiver.poll()
    assert received is not None
    assert received.receive_timestamp_ns == 123_456_789
    assert received.sender_address == ("127.0.0.1", 4001)
    assert receiver.stats.unexpected_sender == 1


def test_receiver_does_not_invent_a_local_frame_index():
    fields = dataclasses.fields(ReceivedDatagram)
    assert [field.name for field in fields] == [
        "payload", "receive_timestamp_ns", "sender_address"
    ]
```

Add these concrete boundary and lifecycle tests:

```python
@pytest.mark.parametrize("size", [PACKET_SIZE - 1, PACKET_SIZE + 1])
def test_poll_skips_wrong_sizes_and_uses_oversize_receive_buffer(size):
    sock = FakeDatagramSocket([
        (b"x" * size, ("127.0.0.1", 4000)),
        (b"y" * PACKET_SIZE, ("127.0.0.1", 4001)),
    ])
    receiver = XsensUdpReceiver(sock=sock, clock_ns=lambda: 9)
    assert receiver.poll().sender_address == ("127.0.0.1", 4001)
    assert receiver.stats.invalid_size == 1
    assert set(sock.recv_sizes) == {PACKET_SIZE + 1}


def test_drain_preserves_order_and_limit():
    queued = [
        (bytes([value]) * PACKET_SIZE, ("127.0.0.1", 4100 + value))
        for value in range(3)
    ]
    receiver = XsensUdpReceiver(sock=FakeDatagramSocket(queued), clock_ns=lambda: 1)
    assert [item.sender_address[1] for item in receiver.drain(limit=2)] == [4100, 4101]
    assert [item.sender_address[1] for item in receiver.drain(limit=2)] == [4102]


@pytest.mark.parametrize("port", [True, False, -1, 0, 65536])
def test_rejects_unsafe_ports(port):
    with pytest.raises(ValueError, match="port"):
        XsensUdpReceiver(port=port, sock=FakeDatagramSocket())


def test_close_is_idempotent_and_real_port_can_be_rebound():
    fake = FakeDatagramSocket()
    receiver = XsensUdpReceiver(sock=fake)
    receiver.close()
    receiver.close()
    assert fake.closed
    assert fake.close_calls == 1
    assert fake.bound == ("0.0.0.0", 9763)
    assert fake.blocking is False

    port = reserve_udp_port()
    for _ in range(2):
        live = XsensUdpReceiver(bind_host="127.0.0.1", port=port)
        live.close()
```

The fake assertions above fix the injected-socket setup contract as well as
the owned-socket lifecycle.

- [ ] **Step 2: Run the receiver tests and verify the class is missing**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_udp_receiver.py -v
```

Expected: collection FAIL importing `xsens.udp_receiver`.

- [ ] **Step 3: Implement nonblocking ownership and statistics**

```python
@dataclass(frozen=True)
class ReceivedDatagram:
    payload: bytes
    receive_timestamp_ns: int
    sender_address: tuple[str, int]


@dataclass
class ReceiverStats:
    received: int = 0
    delivered: int = 0
    invalid_size: int = 0
    unexpected_sender: int = 0
```

Create a nonblocking `AF_INET/SOCK_DGRAM` socket when none is injected; validate `1 <= port <= 65535`; bind the requested address; count every received datagram; filter IP before delivery; timestamp only a delivered complete datagram with the injected monotonic clock. `drain()` repeatedly calls `poll()` until `BlockingIOError`/limit while preserving arrival order. `close()` is idempotent.

- [ ] **Step 4: Run receiver and protocol tests**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_udp_receiver.py test/test_xsens_protocol.py -v
```

Expected: PASS with no lingering UDP socket.

- [ ] **Step 5: Commit UDP ownership**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/udp_receiver.py src/bxi_example_py_elf3/test/test_xsens_udp_receiver.py
git commit -m "feat(xsens): receive complete UDP datagrams"
```

### Task 6: Convert Xsens 23-Segment Transforms to the SONIC Pose Contract

**Files:**

- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/converter.py`
- Create: `src/bxi_example_py_elf3/test/test_xsens_converter.py`

**Interfaces:**

- Consumes: Task 4 `XsensPacket`; shared `compute_from_body_poses()` and `build_elf3_joint_pos()`.
- Produces: `XsensMotionConverter.convert(packet, *, frame_index) -> ConvertedXsensFrame`, transactional `.convert_many(items: Iterable[tuple[XsensPacket, int]], *, reset_epoch: bool = False) -> tuple[ConvertedXsensFrame, ...]`, and `.reset_epoch() -> None`.
- Pure helpers: `xsens_positions_to_xrt`, `xsens_world_quaternions_to_xrt`, `align_quaternion_signs`, `shortest_path_slerp`, `synthesize_smpl_world_quats`, and `world_to_parent_local_quats`.
- Diagnostic property: `previous_raw_quats_xyzw -> np.ndarray | None` returns a copy; callers cannot mutate converter state.

- [ ] **Step 1: Write failing basis, mapping, sign, FK, and wrist tests**

```python
def test_positions_apply_exact_x_z_negative_y_basis():
    source = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    np.testing.assert_array_equal(
        xsens_positions_to_xrt(source),
        np.array([[1.0, 3.0, -2.0]], dtype=np.float32),
    )


def test_wxyz_quaternions_are_normalized_and_sign_continuous():
    converter = XsensMotionConverter()
    identity = np.tile(np.array([1, 0, 0, 0], dtype=np.float32), (23, 1))
    first = converter.convert(
        make_packet(quaternions_wxyz=identity), frame_index=1
    )
    second = converter.convert(
        make_packet(quaternions_wxyz=-identity), frame_index=2
    )
    dots = np.einsum(
        "ij,ij->i", first.segment_quat_xrt_xyzw,
        second.segment_quat_xrt_xyzw,
    )
    assert np.all(dots >= 0.0)
```

Use direct virtual-world-quaternion tests for every mapping row, then one real-FK
contract test. The test file defines this local helper:

```python
def axis_angle_xyzw(axis: int, degrees: float) -> np.ndarray:
    value = np.zeros(4, dtype=np.float64)
    value[axis] = np.sin(np.deg2rad(degrees) / 2.0)
    value[3] = np.cos(np.deg2rad(degrees) / 2.0)
    return value


@pytest.mark.parametrize("smpl_index,segment_id", sorted(DIRECT_SMPL_SEGMENT_IDS.items()))
def test_each_direct_mapping_row_uses_declared_segment(smpl_index, segment_id):
    world = np.tile(np.array([0, 0, 0, 1], np.float64), (23, 1))
    marker = axis_angle_xyzw((segment_id - 1) % 3, 7.0 + segment_id)
    world[segment_id - 1] = marker
    virtual = synthesize_smpl_world_quats(world)
    assert abs(float(np.dot(virtual[smpl_index], marker))) == pytest.approx(1.0)


def test_basis_rotation_uses_matrix_conjugation():
    source = Rotation.from_euler("xyz", [17.0, -23.0, 41.0], degrees=True).as_matrix()
    source_wxyz = Rotation.from_matrix(source).as_quat(scalar_first=True).reshape(1, 4)
    converted = xsens_world_quaternions_to_xrt(source_wxyz)
    actual = Rotation.from_quat(converted[0]).as_matrix()
    np.testing.assert_allclose(actual, XSENS_TO_XRT @ source @ XSENS_TO_XRT.T, atol=1e-6)


def test_spine_midpoint_uses_shortest_path_and_endpoints_copy_wrists():
    world = np.tile(np.array([0, 0, 0, 1], np.float64), (23, 1))
    world[2] = axis_angle_xyzw(2, 20.0)       # L3, ID 3
    world[3] = -axis_angle_xyzw(2, 60.0)      # T12, ID 4, opposite sign
    world[14] = axis_angle_xyzw(0, 31.0)      # left hand, ID 15
    world[10] = axis_angle_xyzw(1, -29.0)     # right hand, ID 11
    virtual = synthesize_smpl_world_quats(world)
    expected_midpoint = axis_angle_xyzw(2, 40.0)
    assert abs(float(np.dot(virtual[6], expected_midpoint))) == pytest.approx(1.0)
    np.testing.assert_array_equal(virtual[22], virtual[20])
    np.testing.assert_array_equal(virtual[23], virtual[21])


def test_toes_remain_left_right_distinct_and_hand_endpoint_locals_are_identity():
    world = np.tile(np.array([0, 0, 0, 1], np.float64), (23, 1))
    world[22] = axis_angle_xyzw(0, 12.0)  # left toe ID 23
    world[18] = axis_angle_xyzw(1, 19.0)  # right toe ID 19
    virtual = synthesize_smpl_world_quats(world)
    assert not np.allclose(virtual[10], virtual[11])
    local = world_to_parent_local_quats(virtual, SMPL24_PARENTS)
    np.testing.assert_allclose(local[22], [0, 0, 0, 1], atol=1e-6)
    np.testing.assert_allclose(local[23], [0, 0, 0, 1], atol=1e-6)


def test_real_fk_output_has_exact_shapes_dtypes_and_only_six_wrist_slots():
    frame = XsensMotionConverter().convert(make_packet(), frame_index=17)
    assert frame.smpl_joints.shape == (24, 3)
    assert frame.body_quat_w.shape == (4,)
    assert frame.smpl_body_pose.shape == (21, 3)
    assert frame.joint_pos.shape == (29,)
    assert all(array.dtype == np.float32 for array in (
        frame.segment_positions_xrt, frame.segment_quat_xrt_xyzw,
        frame.smpl_body_pose, frame.smpl_joints, frame.body_quat_w,
        frame.joint_pos,
    ))
    assert np.isfinite(frame.smpl_joints).all()
    assert set(np.flatnonzero(frame.joint_pos)) <= {19, 20, 21, 26, 27, 28}


def test_xsens_positions_do_not_rescale_or_translate_fixed_skeleton():
    base = np.arange(69, dtype=np.float32).reshape(23, 3) / 100.0
    first = XsensMotionConverter().convert(make_packet(positions=base), frame_index=1)
    second = XsensMotionConverter().convert(
        make_packet(positions=base * 1.8 + np.array([3, -2, 7], np.float32)),
        frame_index=2,
    )
    np.testing.assert_allclose(first.smpl_joints, second.smpl_joints, atol=1e-6)


def test_reset_epoch_clears_sign_history():
    converter = XsensMotionConverter()
    positive = np.tile(np.array([1, 0, 0, 0], np.float32), (23, 1))
    negative = -positive
    converter.convert(make_packet(quaternions_wxyz=positive), frame_index=1)
    continuous = converter.convert(make_packet(quaternions_wxyz=negative), frame_index=2)
    converter.reset_epoch()
    reset = converter.convert(make_packet(quaternions_wxyz=negative), frame_index=3)
    assert np.all(continuous.segment_quat_xrt_xyzw[:, 3] > 0)
    assert np.all(reset.segment_quat_xrt_xyzw[:, 3] < 0)


def test_convert_many_does_not_commit_first_frame_when_second_fk_fails(monkeypatch):
    converter = XsensMotionConverter()
    before = converter.previous_raw_quats_xyzw
    real_fk = converter_module.compute_from_body_poses
    calls = 0

    def fail_second(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise RuntimeError("synthetic FK failure")
        return real_fk(*args, **kwargs)

    monkeypatch.setattr(converter_module, "compute_from_body_poses", fail_second)
    with pytest.raises(RuntimeError, match="synthetic FK failure"):
        converter.convert_many(
            [(make_packet(sample_counter=1), 1), (make_packet(sample_counter=2), 2)],
            reset_epoch=True,
        )
    assert converter.previous_raw_quats_xyzw is before
```

Expose `previous_raw_quats_xyzw` as a read-only, copied diagnostic property so
the transaction test does not inspect a mutable private array. Task 9's armed
stale test spies on `converter.reset_epoch` and asserts its call count remains
zero; only a committed epoch change may reset converter sign history.

- [ ] **Step 2: Run conversion tests and verify the converter is missing**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_converter.py -v
```

Expected: collection FAIL importing `xsens.converter`.

- [ ] **Step 3: Implement the exact basis and 24-joint mapping**

```python
SMPL24_PARENTS = [
    -1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8,
    9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21,
]

XSENS_TO_XRT = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, -1.0, 0.0],
], dtype=np.float64)

DIRECT_SMPL_SEGMENT_IDS = {
    0: 1, 1: 20, 2: 16, 3: 2, 4: 21, 5: 17,
    7: 22, 8: 18, 9: 5, 10: 23, 11: 19, 12: 6,
    13: 12, 14: 8, 15: 7, 16: 13, 17: 9, 18: 14,
    19: 10, 20: 15, 21: 11,
}
```

Set SMPL index 6 to half-Slerp of segment IDs 3 and 4; copy indices
20/21 to endpoints 22/23. Normalize and sign-align each segment before the
basis conjugation. Build `(24,7)` `body_poses`, put converted pelvis
translation only in row 0, call fixed-skeleton FK, emit
`smpl_pose[:63].reshape(21,3)`, root-local joints, scalar-first root quaternion,
and the shared 29-joint wrist output. Implement conversion through the pure
helper
`_convert_with_previous(packet: XsensPacket, frame_index: int, previous_quats: np.ndarray | None) -> tuple[ConvertedXsensFrame, np.ndarray]`.
`convert()` or `convert_many()` replaces `_previous_raw_quats_xyzw` only after
every derived array in the entire call passes shape/finite/unit validation. A
failed single frame or failed second candidate frame leaves sign history
unchanged. No T-pose calibration, rest subtraction, body-dimension scaling, or
low-pass filter is permitted.

- [ ] **Step 4: Run conversion plus ZeroLab math regressions**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_converter.py test/test_zerolab_converter.py -v
```

Expected: PASS; ZeroLab calibration behavior is unchanged.

- [ ] **Step 5: Commit conversion**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/converter.py src/bxi_example_py_elf3/test/test_xsens_converter.py
git commit -m "feat(xsens): convert segments to SONIC pose"
```

### Task 7: Implement Counter Unwrapping and Observable Source-Epoch Changes

**Files:**

- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/source_core.py`
- Modify: `src/bxi_example_py_elf3/test/xsens_test_helpers.py`
- Create: `src/bxi_example_py_elf3/test/test_xsens_source_epoch.py`

**Interfaces:**

- Consumes: Task 4 `XsensPacket`, Task 6 `XsensMotionConverter`, injectable monotonic clock and epoch factory.
- Produces: `CounterKind`, `CounterDelta`, `TimeCodeMode`, `AcceptClassification`, `classify_uint32_delta(previous, current)`, `draw_new_epoch(epoch_factory, *, current_epoch=0)`, `XsensSourceCore.accept(packet) -> AcceptResult`, `expire(now_ns: int) -> bool`, read-only `stats: SourceCoreStats`, `source_epoch`, `newest_frame_index`, `newest_producer_monotonic_ns`, `time_code_mode`, `candidate_frame_count`, `candidate_sender`, and `locked_sender: tuple[str, int, int] | None`.
- Epoch candidate defaults: two coherent frames within 0.25 seconds; changed sender is eligible only when active producer age is strictly greater than 0.5 seconds.

```text
XsensSourceCore(
  converter: XsensMotionConverter,
  *, clock_ns: Callable[[], int], epoch_factory: Callable[[], int],
  window_frames: int = 10, same_epoch_resume_frames: int = 10,
  ready_frames: int = 30,
  stale_seconds: float = 0.5, epoch_candidate_frames: int = 2,
  epoch_candidate_timeout_s: float = 0.25,
  max_pelvis_span_m: float = 0.15,
  max_segment_deviation_deg: float = 20.0
)
```

- [ ] **Step 1: Write the half-range, sender-lock, time-code, and candidate tests**

```python
@pytest.mark.parametrize(
    ("previous", "current", "kind", "missing"),
    [
        (10, 10, CounterKind.DUPLICATE, 0),
        (10, 11, CounterKind.FORWARD, 0),
        (10, 14, CounterKind.FORWARD, 2),
        (0xFFFFFFFF, 0, CounterKind.FORWARD, 0),
        (0, 0x80000000, CounterKind.AMBIGUOUS, 0),
        (1000, 900, CounterKind.BACKWARD, 0),
    ],
)
def test_uint32_counter_classifies_half_range(previous, current, kind, missing):
    result = classify_uint32_delta(previous, current)
    assert result.kind is kind
    assert result.missing_frames == missing


def test_epoch_draw_retries_zero_and_current_epoch():
    draws = iter([-1, 0, 1 << 63, 41, 42])
    assert draw_new_epoch(lambda: next(draws), current_epoch=41) == 42
```

Add these source-core factories to `xsens_test_helpers.py` in this task:

```python
def make_core(
    *,
    epoch_draws: tuple[int, ...] = (91, 92, 93),
    converter: XsensMotionConverter | None = None,
    now_ns: int = 0,
    window_frames: int = 10,
    same_epoch_resume_frames: int = 10,
    ready_frames: int = 30,
    stale_seconds: float = 0.5,
    epoch_candidate_frames: int = 2,
    epoch_candidate_timeout_s: float = 0.25,
    max_pelvis_span_m: float = 0.15,
    max_segment_deviation_deg: float = 20.0,
) -> tuple[XsensSourceCore, FakeClock]:
    clock = FakeClock(now_ns)
    draws = iter(epoch_draws)
    core = XsensSourceCore(
        converter or XsensMotionConverter(),
        clock_ns=clock.monotonic_ns,
        epoch_factory=lambda: next(draws),
        window_frames=window_frames,
        same_epoch_resume_frames=same_epoch_resume_frames,
        ready_frames=ready_frames,
        stale_seconds=stale_seconds,
        epoch_candidate_frames=epoch_candidate_frames,
        epoch_candidate_timeout_s=epoch_candidate_timeout_s,
        max_pelvis_span_m=max_pelvis_span_m,
        max_segment_deviation_deg=max_segment_deviation_deg,
    )
    return core, clock


def accept_packet(core, counter, *, timestamp_ns, time_code=0, sender=("127.0.0.1", 4000)):
    return core.accept(make_packet(
        sample_counter=counter,
        time_code=time_code,
        sender=sender,
        receive_timestamp_ns=timestamp_ns,
    ))
```

The reset-candidate tests use exact packet sequences rather than descriptive
fixtures:

```python
def test_sender_lock_forward_gap_and_same_epoch_post_gap_recovery():
    core, _ = make_core(epoch_draws=(91, 92))
    first = accept_packet(core, 100, timestamp_ns=0)
    gap = accept_packet(core, 104, timestamp_ns=2_000_000_000)
    assert first.classification is AcceptClassification.INITIAL
    assert gap.classification is AcceptClassification.FORWARD
    assert gap.missing_frames == 3
    assert core.source_epoch == 91
    assert core.locked_sender == ("127.0.0.1", 4000, 0)
    assert core.newest_producer_monotonic_ns == 2_000_000_000


def test_active_packet_or_duplicate_cancels_pending_candidate():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 1000, timestamp_ns=0, time_code=1000)
    started = accept_packet(core, 900, timestamp_ns=10_000_000, time_code=900)
    assert started.classification is AcceptClassification.CANDIDATE_STARTED
    assert core.candidate_frame_count == 1
    accepted = accept_packet(core, 1001, timestamp_ns=20_000_000, time_code=1001)
    assert accepted.classification is AcceptClassification.FORWARD
    assert core.candidate_frame_count == 0

    accept_packet(core, 800, timestamp_ns=30_000_000, time_code=800)
    producer_before = core.newest_producer_monotonic_ns
    duplicate = accept_packet(core, 1001, timestamp_ns=40_000_000, time_code=1001)
    assert duplicate.classification is AcceptClassification.DUPLICATE
    assert core.candidate_frame_count == 0
    assert core.newest_producer_monotonic_ns == producer_before


def test_candidate_duplicate_replacement_and_strict_expiry():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 1000, timestamp_ns=0)
    accept_packet(core, 900, timestamp_ns=1_000_000_000)
    duplicate = accept_packet(core, 900, timestamp_ns=1_010_000_000)
    assert duplicate.classification is AcceptClassification.DUPLICATE
    assert core.candidate_frame_count == 1
    replacement = accept_packet(core, 800, timestamp_ns=1_020_000_000)
    assert replacement.classification is AcceptClassification.CANDIDATE_REPLACED
    assert core.candidate_frame_count == 1
    assert not core.expire(1_270_000_000)
    assert core.candidate_frame_count == 1
    assert core.expire(1_270_000_001)
    assert core.candidate_frame_count == 0


def test_two_candidate_frames_commit_atomically_with_original_time():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 1000, timestamp_ns=0)
    first = accept_packet(core, 900, timestamp_ns=600_000_000)
    second = accept_packet(core, 901, timestamp_ns=610_000_000)
    assert first.classification is AcceptClassification.CANDIDATE_STARTED
    assert second == AcceptResult(
        accepted=True,
        classification=AcceptClassification.EPOCH_COMMITTED,
        epoch_changed=True,
        missing_frames=0,
    )
    assert core.source_epoch == 92
    assert core.newest_frame_index == 901
    assert core.newest_producer_monotonic_ns == 610_000_000
    assert core.locked_sender == ("127.0.0.1", 4000, 0)
    assert core.stats.epoch_changes == 1


def test_time_code_modes_wrap_and_regression_evidence():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 10, timestamp_ns=0, time_code=0)
    accept_packet(core, 11, timestamp_ns=1, time_code=0)
    accept_packet(core, 12, timestamp_ns=2, time_code=0xFFFFFFFE)
    accept_packet(core, 13, timestamp_ns=3, time_code=0xFFFFFFFF)
    assert core.time_code_mode is TimeCodeMode.ADVANCING
    wrapped = accept_packet(core, 14, timestamp_ns=4, time_code=0)
    assert wrapped.classification is AcceptClassification.FORWARD
    half_range = accept_packet(core, 15, timestamp_ns=5, time_code=0x80000000)
    assert half_range.classification is AcceptClassification.CANDIDATE_STARTED

    # An active-compatible sample still wins over the pending candidate.
    resumed = accept_packet(core, 15, timestamp_ns=6, time_code=1)
    assert resumed.classification is AcceptClassification.FORWARD
    assert core.source_epoch == 91


def test_constant_to_advancing_does_not_change_epoch_but_regression_can_commit():
    core, _ = make_core(epoch_draws=(91, 92))
    for counter, time_code in ((100, 0), (101, 0), (102, 50), (103, 51)):
        accept_packet(core, counter, timestamp_ns=counter, time_code=time_code)
    assert core.time_code_mode is TimeCodeMode.ADVANCING
    assert core.source_epoch == 91
    accept_packet(core, 104, timestamp_ns=104, time_code=0)
    committed = accept_packet(core, 105, timestamp_ns=105, time_code=1)
    assert committed.classification is AcceptClassification.EPOCH_COMMITTED
    assert core.source_epoch == 92


def test_changed_sender_requires_strictly_stale_active_producer():
    core, _ = make_core(epoch_draws=(91, 92))
    accept_packet(core, 1, timestamp_ns=0, sender=("127.0.0.1", 4000))
    at_boundary = accept_packet(
        core, 1, timestamp_ns=500_000_000, sender=("127.0.0.1", 5000)
    )
    assert at_boundary.classification is AcceptClassification.SENDER_INELIGIBLE
    first = accept_packet(
        core, 1, timestamp_ns=500_000_001, sender=("127.0.0.1", 5000)
    )
    second = accept_packet(
        core, 2, timestamp_ns=510_000_000, sender=("127.0.0.1", 5000)
    )
    assert first.classification is AcceptClassification.CANDIDATE_STARTED
    assert second.classification is AcceptClassification.EPOCH_COMMITTED
    assert core.locked_sender == ("127.0.0.1", 5000, 0)


def test_process_restart_changes_epoch_but_fully_forward_same_sender_restart_is_unobservable():
    first, _ = make_core(epoch_draws=(91,))
    second, _ = make_core(epoch_draws=(92,))
    accept_packet(first, 700, timestamp_ns=0, time_code=700)
    accept_packet(second, 701, timestamp_ns=0, time_code=701)
    assert (first.source_epoch, second.source_epoch) == (91, 92)

    before = first.source_epoch
    result = accept_packet(first, 701, timestamp_ns=9_000_000_000, time_code=701)
    assert result.classification is AcceptClassification.FORWARD
    assert first.source_epoch == before  # MXTP02 exposes no restart evidence.


def test_failed_second_candidate_conversion_does_not_partially_commit(monkeypatch):
    converter = XsensMotionConverter()
    core, _ = make_core(epoch_draws=(91, 92), converter=converter)
    accept_packet(core, 1000, timestamp_ns=0)
    accept_packet(core, 900, timestamp_ns=600_000_000)
    before = (
        core.source_epoch, core.locked_sender, core.newest_frame_index,
        core.newest_producer_monotonic_ns,
    )
    monkeypatch.setattr(
        converter, "convert_many",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("FK failed")),
    )
    result = accept_packet(core, 901, timestamp_ns=610_000_000)
    assert result.classification is AcceptClassification.CONVERSION_REJECTED
    assert (
        core.source_epoch, core.locked_sender, core.newest_frame_index,
        core.newest_producer_monotonic_ns,
    ) == before
    assert core.stats.conversion_failures == 1
```

The malformed-byte-between-candidates case remains in Task 10 because that is
the first layer that receives bytes and invokes the parser.

- [ ] **Step 2: Run epoch tests and verify the core is missing**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_source_epoch.py -v
```

Expected: collection FAIL importing `xsens.source_core`.

- [ ] **Step 3: Implement modular classification and atomic epoch commit**

```python
class CounterKind(str, Enum):
    DUPLICATE = "duplicate"
    FORWARD = "forward"
    AMBIGUOUS = "ambiguous"
    BACKWARD = "backward"


def classify_uint32_delta(previous: int, current: int) -> CounterDelta:
    delta = (int(current) - int(previous)) & 0xFFFFFFFF
    if delta == 0:
        return CounterDelta(CounterKind.DUPLICATE, 0, 0)
    if delta < 0x80000000:
        return CounterDelta(CounterKind.FORWARD, delta, delta - 1)
    if delta == 0x80000000:
        return CounterDelta(CounterKind.AMBIGUOUS, delta, 0)
    return CounterDelta(CounterKind.BACKWARD, delta, 0)
```

Keep active-compatible matching-sender classification higher priority than a
pending candidate. Candidate packets never update active freshness/windows.
`expire(now_ns)` clears a candidate only when its first receive age is strictly
greater than 0.25 seconds and returns whether it changed state; the source node
calls it every 50 Hz tick. On commit, first draw a valid replacement epoch into
a local variable without mutating core state, then call the converter's
transactional `convert_many(..., reset_epoch=True)`. If drawing or either
conversion/FK step fails, no core field changes and `convert_many` leaves sign
history unchanged; a conversion failure increments only `conversion_failures`.
After conversion succeeds, the remainder is a no-fail assignment block under
the core lock: install the drawn epoch, zero authorization/receipt, clear
unwrap/readiness/output state, retain processed command IDs, and append the two
already-converted candidate frames in receive order with their original
timestamps. This final block invokes no callbacks or conversion code. Time code
is diagnostic evidence only and never a freshness clock.

Validate positive integer window sizes and require
`same_epoch_resume_frames == window_frames`; the approved defaults are both
exactly ten.

`draw_new_epoch()` accepts only non-bool integers in `1..2**63-1` that differ from `current_epoch`; it redraws negative, zero, `>=2**63`, current, and repeated collision values rather than masking them.

- [ ] **Step 4: Run epoch, protocol, and converter tests**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_source_epoch.py test/test_xsens_protocol.py test/test_xsens_converter.py -v
```

Expected: PASS across counter wrap, reset candidates, sender replacement, and the unobservable-restart limitation.

- [ ] **Step 5: Commit epoch tracking**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/source_core.py src/bxi_example_py_elf3/test/xsens_test_helpers.py src/bxi_example_py_elf3/test/test_xsens_source_epoch.py
git commit -m "feat(xsens): track sample counters and source epochs"
```

### Task 8: Add Loose Readiness, Unarmed Staleness, and Rolling Pose Windows

**Files:**

- Modify: `src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/source_core.py`
- Modify: `src/bxi_example_py_elf3/test/xsens_test_helpers.py`
- Create: `src/bxi_example_py_elf3/test/test_xsens_source_core.py`

**Interfaces:**

- Consumes: accepted current-epoch frames from Task 7.
- Produces: read-only `ready: bool`, `ready_frames: int`, `reference_window_ready: bool`, `newest_producer_monotonic_ns: int`, `check_stale(now_ns: int) -> bool`, and `current_pose_window() -> tuple[ConvertedXsensFrame, ...] | None`.
- Timing boundaries: exactly 0.5 seconds remains fresh; only age `> 0.5` is stale.

- [ ] **Step 1: Add failing readiness, boundary, and rolling-window tests**

```python
def test_readiness_requires_thirty_stable_frames_and_complete_window():
    core, clock = make_core()
    feed_stable_frames(core, clock, count=29)
    assert core.ready is False
    assert core.ready_frames == 29
    feed_stable_frames(core, clock, count=1)
    assert core.ready_frames == 30
    assert core.reference_window_ready is True
    assert core.ready is True
    assert [frame.frame_index for frame in core.current_pose_window()] == list(range(21, 31))


def test_stale_boundary_is_strictly_greater_than_half_second():
    core, clock = ready_core(epoch=91)
    newest = core.newest_producer_monotonic_ns
    assert not core.check_stale(newest + 500_000_000)
    assert core.check_stale(newest + 500_000_001)
    assert core.ready is False
    assert core.ready_frames == 0
    assert core.current_pose_window() is None
```

Add the frame-sequence helpers in `xsens_test_helpers.py`:

```python
SOURCE_PERIOD_NS = 1_000_000_000 // 60


def feed_frame_sequence(core, clock, positions, quaternions, *, counter_start=None):
    count = len(positions)
    assert len(quaternions) == count
    if counter_start is None:
        counter_start = 1 if core.newest_frame_index < 0 else (
            (core.newest_frame_index + 1) & 0xFFFFFFFF
        )
    results = []
    for offset, (position, quat) in enumerate(zip(positions, quaternions)):
        clock.advance_ns(SOURCE_PERIOD_NS)
        results.append(core.accept(make_packet(
            sample_counter=(counter_start + offset) & 0xFFFFFFFF,
            time_code=0,
            receive_timestamp_ns=clock.now_ns,
            positions=position,
            quaternions_wxyz=quat,
        )))
    return results


def feed_stable_frames(core, clock, count, epoch_counter_start=None):
    positions = np.zeros((count, 23, 3), dtype=np.float32)
    quats = np.tile(
        np.array([1, 0, 0, 0], dtype=np.float32), (count, 23, 1)
    )
    return feed_frame_sequence(
        core, clock, positions, quats, counter_start=epoch_counter_start
    )


def ready_core(epoch):
    core, clock = make_core(epoch_draws=(epoch, epoch + 1))
    feed_stable_frames(core, clock, 30)
    assert core.source_epoch == epoch and core.ready
    return core, clock


def axis_angle_wxyz(axis: int, degrees: float) -> np.ndarray:
    value = np.zeros(4, dtype=np.float32)
    value[0] = np.cos(np.deg2rad(degrees) / 2.0)
    value[axis + 1] = np.sin(np.deg2rad(degrees) / 2.0)
    return value
```

Use these exact stability/window tests:

```python
def test_approximate_neutral_need_not_match_t_pose():
    core, clock = make_core()
    positions = np.zeros((30, 23, 3), np.float32)
    pose = np.stack([axis_angle_wxyz(i % 3, 5.0 + i) for i in range(23)])
    quats = np.repeat(pose[None, :, :], 30, axis=0)
    feed_frame_sequence(core, clock, positions, quats)
    assert core.ready


def test_global_pelvis_origin_does_not_affect_stability():
    for origin in ([0, 0, 0], [100, -40, 8]):
        core, clock = make_core()
        positions = np.zeros((30, 23, 3), np.float32)
        positions += np.asarray(origin, np.float32)
        quats = np.tile(np.array([1, 0, 0, 0], np.float32), (30, 23, 1))
        feed_frame_sequence(core, clock, positions, quats)
        assert core.ready


@pytest.mark.parametrize(
    "span,expected",
    [
        (np.float32(0.15), True),
        (np.nextafter(np.float32(0.15), np.float32(np.inf)), False),
    ],
)
def test_pelvis_diameter_boundary_is_inclusive(span, expected):
    core, clock = make_core(max_pelvis_span_m=float(np.float32(0.15)))
    positions = np.zeros((30, 23, 3), np.float32)
    positions[15:, 0, 0] = span
    quats = np.tile(np.array([1, 0, 0, 0], np.float32), (30, 23, 1))
    feed_frame_sequence(core, clock, positions, quats)
    assert core.ready is expected


@pytest.mark.parametrize("angle,expected", [(20.0, True), (20.01, False)])
def test_segment_p95_boundary_is_inclusive(angle, expected):
    core, clock = make_core(max_segment_deviation_deg=20.0)
    positions = np.zeros((30, 23, 3), np.float32)
    quats = np.tile(np.array([1, 0, 0, 0], np.float32), (30, 23, 1))
    quats[:15, 5] = axis_angle_wxyz(2, angle)
    quats[15:, 5] = axis_angle_wxyz(2, -angle)
    feed_frame_sequence(core, clock, positions, quats)
    assert core.ready is expected


def test_motion_above_either_threshold_blocks_ready():
    pelvis_core, pelvis_clock = make_core()
    positions = np.zeros((30, 23, 3), np.float32)
    positions[15:, 0, 0] = 0.151
    identity = np.tile(np.array([1, 0, 0, 0], np.float32), (30, 23, 1))
    feed_frame_sequence(pelvis_core, pelvis_clock, positions, identity)
    assert not pelvis_core.ready

    segment_core, segment_clock = make_core()
    moving = identity.copy()
    moving[:15, 9] = axis_angle_wxyz(1, 21.0)
    moving[15:, 9] = axis_angle_wxyz(1, -21.0)
    feed_frame_sequence(
        segment_core, segment_clock, np.zeros_like(positions), moving
    )
    assert not segment_core.ready


def test_forward_counter_gap_does_not_reset_readiness():
    core, clock = make_core()
    feed_stable_frames(core, clock, 15, epoch_counter_start=1)
    feed_stable_frames(core, clock, 15, epoch_counter_start=20)
    assert core.ready_frames == 30
    assert core.stats.inferred_missing_frames == 4
    assert core.ready


def test_window_contains_ten_strictly_increasing_current_epoch_frames():
    core, clock = make_core()
    feed_stable_frames(core, clock, 30)
    indices = [frame.frame_index for frame in core.current_pose_window()]
    assert indices == list(range(21, 31))
    assert all(right > left for left, right in zip(indices, indices[1:]))
```

`axis_angle_wxyz(axis, degrees)` is the scalar-first counterpart of Task 6's
test helper. The implementation compares the float32 pelvis span to the
float32-configured limit so the closed wire boundary is deterministic; angular
comparison allows only numerical roundoff (`1e-5 deg`), not a semantic margin.

- [ ] **Step 2: Run the source-core tests and verify readiness is absent**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_source_core.py -v
```

Expected: FAIL because readiness/window fields and stale-edge behavior are not implemented.

- [ ] **Step 3: Implement exact loose-stability and post-edge window rules**

```python
def angular_deviation_degrees(samples_xyzw: np.ndarray) -> np.ndarray:
    aligned = samples_xyzw.copy()
    aligned[np.einsum("fsc,sc->fs", aligned, aligned[0]) < 0.0] *= -1.0
    mean = aligned.mean(axis=0)
    mean /= np.linalg.norm(mean, axis=1, keepdims=True)
    dots = np.abs(np.einsum("fsc,sc->fs", aligned, mean))
    return np.degrees(2.0 * np.arccos(np.clip(dots, 0.0, 1.0)))
```

Maintain independent deques for 30-frame readiness evidence and the current 10-frame output window. Pelvis stability is the maximum pairwise distance in the 30-frame window; segment stability is per-segment p95 after hemisphere alignment. Append each accepted source sample at most once. Before authorization exists, a stale edge clears readiness and output but never resets converter sign continuity or source epoch. A counter gap is counted but accepted. Task 9 extends this same edge with armed recovery semantics and turns the window into packed pose fields.

- [ ] **Step 4: Run source-core and epoch regressions**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_source_core.py test/test_xsens_source_epoch.py -v
```

Expected: PASS at exact readiness/freshness thresholds and with strictly increasing ten-row windows.

- [ ] **Step 5: Commit readiness and windows**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/source_core.py src/bxi_example_py_elf3/test/xsens_test_helpers.py src/bxi_example_py_elf3/test/test_xsens_source_core.py
git commit -m "feat(xsens): gate readiness and pose windows"
```

### Task 9: Implement Arm Authorization and Exact Pose/Status Contracts

**Files:**

- Modify: `src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/source_core.py`
- Modify: `src/bxi_example_py_elf3/test/xsens_test_helpers.py`
- Create: `src/bxi_example_py_elf3/test/test_xsens_source_contract.py`

**Interfaces:**

- Consumes: Task 8 readiness/freshness/windows and canonical `ArmCommand`.
- Produces: `handle_arm_command(command: ArmCommand) -> ArmCommandResult`, `build_pose_fields()`, `build_status_fields(now_ns)`, and read-only `accepted_arm_epoch: int`.
- Receipt rule: only first-seen structurally valid commands replace receipt fields; an exact duplicate is idempotent; an ID reused with different payload is invalid.

- [ ] **Step 1: Write failing authorization, replay, dtype, and status-sequence tests**

```python
def make_xsens_pose_fields(
    *, indices=None, source_epoch=71, calibration_ready=True,
    producer_monotonic_ns=1_000_000_000,
):
    frame_index = (
        np.arange(100, 110, dtype=np.int64)
        if indices is None else np.asarray(indices, dtype=np.int64)
    )
    rows = int(frame_index.size)
    roots = np.zeros((rows, 4), dtype=np.float32)
    roots[:, 0] = 1.0
    return {
        "frame_index": frame_index,
        "smpl_joints": np.zeros((rows, 24, 3), dtype=np.float32),
        "body_quat_w": roots,
        "joint_pos": np.zeros((rows, 29), dtype=np.float32),
        "stream_mode": np.array([1], dtype=np.int32),
        "calibration_ready": np.array([calibration_ready], dtype=np.bool_),
        "producer_monotonic_ns": np.array([producer_monotonic_ns], dtype=np.int64),
        "source_epoch": np.array([source_epoch], dtype=np.int64),
    }


def make_status(
    *, status_sequence=1, status_monotonic_ns=1_000_000_000,
    source_epoch=71, last_arm_command_id=0, last_arm_target_epoch=0,
    last_requested_arm_epoch=0, accepted_arm_epoch=0,
    producer_monotonic_ns=1_000_000_000, newest_frame_index=109,
    ready=True, reference_window_ready=True, source_stale=False,
    ready_frames=30, recovery_frames=0, reason_code=XsensReason.READY,
):
    return {
        "status_sequence": np.array([status_sequence], dtype=np.int64),
        "status_monotonic_ns": np.array([status_monotonic_ns], dtype=np.int64),
        "source_epoch": np.array([source_epoch], dtype=np.int64),
        "last_arm_command_id": np.array([last_arm_command_id], dtype=np.int64),
        "last_arm_target_epoch": np.array([last_arm_target_epoch], dtype=np.int64),
        "last_requested_arm_epoch": np.array([last_requested_arm_epoch], dtype=np.int64),
        "accepted_arm_epoch": np.array([accepted_arm_epoch], dtype=np.int64),
        "producer_monotonic_ns": np.array([producer_monotonic_ns], dtype=np.int64),
        "newest_frame_index": np.array([newest_frame_index], dtype=np.int64),
        "ready": np.array([ready], dtype=np.bool_),
        "reference_window_ready": np.array([reference_window_ready], dtype=np.bool_),
        "source_stale": np.array([source_stale], dtype=np.bool_),
        "ready_frames": np.array([ready_frames], dtype=np.int32),
        "recovery_frames": np.array([recovery_frames], dtype=np.int32),
        "reason_code": np.array([int(reason_code)], dtype=np.int32),
    }


def test_valid_arm_requires_current_ready_fresh_complete_epoch():
    core, clock = ready_core(epoch=91)
    result = core.handle_arm_command(ArmCommand(7, 91, 91))
    assert result == ArmCommandResult(
        ArmCommandClassification.ARMED, True, True, True
    )
    status = core.build_status_fields(clock.now_ns)
    assert status["last_arm_command_id"].tolist() == [7]
    assert status["last_arm_target_epoch"].tolist() == [91]
    assert status["last_requested_arm_epoch"].tolist() == [91]
    assert status["accepted_arm_epoch"].tolist() == [91]


def test_disarm_clears_readiness_even_when_already_unarmed():
    core, clock = ready_core(epoch=91)
    assert core.handle_arm_command(ArmCommand(8, 91, 0)) == ArmCommandResult(
        ArmCommandClassification.DISARMED, True, True, True
    )
    status = core.build_status_fields(clock.now_ns)
    assert status["accepted_arm_epoch"].tolist() == [0]
    assert status["ready_frames"].tolist() == [0]
    assert not status["reference_window_ready"][0]


def test_armed_stale_retains_authorization_and_requires_ten_recovery_frames():
    core, clock = ready_core(epoch=91)
    core.handle_arm_command(ArmCommand(9, 91, 91))
    newest = core.newest_producer_monotonic_ns
    clock.advance_ns(newest + 500_000_001 - clock.now_ns)
    assert core.check_stale(clock.now_ns)
    assert core.accepted_arm_epoch == 91
    assert core.current_pose_window() is None

    feed_stable_frames(core, clock, count=9, epoch_counter_start=31)
    status = core.build_status_fields(clock.now_ns)
    assert status["recovery_frames"].tolist() == [9]
    assert core.build_pose_fields() is None

    feed_stable_frames(core, clock, count=1, epoch_counter_start=40)
    status = core.build_status_fields(clock.now_ns)
    assert status["recovery_frames"].tolist() == [10]
    assert core.build_pose_fields()["frame_index"].tolist() == list(range(31, 41))


@pytest.mark.parametrize(
    "command",
    [
        ArmCommand(0, 91, 91),
        ArmCommand(-1, 91, 91),
        ArmCommand(True, 91, 91),
        ArmCommand(1.0, 91, 91),
        ArmCommand(2**63, 91, 91),
        ArmCommand(1, 0, 0),
        ArmCommand(1, -1, 0),
        ArmCommand(1, True, 0),
        ArmCommand(1, 91.0, 0),
        ArmCommand(1, 2**63, 0),
        ArmCommand(1, 91, True),
        ArmCommand(1, 91, 91.0),
        ArmCommand(1, 91, 90),
    ],
)
def test_structurally_invalid_commands_do_not_replace_receipt(command):
    core, clock = ready_core(epoch=91)
    result = core.handle_arm_command(command)
    assert result.classification is ArmCommandClassification.INVALID
    assert result.publish_immediately is False
    assert core.build_status_fields(clock.now_ns)["last_arm_command_id"].tolist() == [0]


def test_duplicate_and_reused_command_id_are_distinct():
    core, clock = ready_core(epoch=91)
    command = ArmCommand(12, 91, 91)
    assert core.handle_arm_command(command).classification is ArmCommandClassification.ARMED
    assert core.handle_arm_command(command) == ArmCommandResult(
        ArmCommandClassification.DUPLICATE, False, False, False
    )
    changed = core.handle_arm_command(ArmCommand(12, 91, 0))
    assert changed == ArmCommandResult(
        ArmCommandClassification.REUSED_ID, False, False, False
    )
    assert core.accepted_arm_epoch == 91
```

Add these exact authorization and serialization tests:

```python
POSE_KEYS = {
    "frame_index", "smpl_joints", "body_quat_w", "joint_pos",
    "stream_mode", "calibration_ready", "producer_monotonic_ns",
    "source_epoch",
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


def test_first_seen_current_target_arm_while_not_ready_is_echoed_but_not_accepted():
    core, clock = make_core(epoch_draws=(91, 92))
    feed_stable_frames(core, clock, 10)
    result = core.handle_arm_command(ArmCommand(19, 91, 91))
    assert result == ArmCommandResult(
        ArmCommandClassification.NOT_READY, True, True, True
    )
    status = core.build_status_fields(clock.now_ns)
    assert status["last_arm_command_id"].tolist() == [19]
    assert status["accepted_arm_epoch"].tolist() == [0]


def test_accepted_arm_bypasses_later_stillness_but_not_freshness_or_complete_window():
    core, clock = ready_core(91)
    assert core.handle_arm_command(ArmCommand(20, 91, 91)).classification is ArmCommandClassification.ARMED

    positions = np.zeros((30, 23, 3), np.float32)
    positions[:, 0, 0] = np.linspace(0.0, 1.0, 30, dtype=np.float32)
    quats = np.tile(np.array([1, 0, 0, 0], np.float32), (30, 23, 1))
    feed_frame_sequence(core, clock, positions, quats)
    status = core.build_status_fields(clock.now_ns)
    assert status["ready"].tolist() == [True]
    assert status["reason_code"].tolist() == [XsensReason.ARMED_FRESH]

    newest = core.newest_producer_monotonic_ns
    assert core.check_stale(newest + 500_000_001)
    stale = core.build_status_fields(newest + 500_000_001)
    assert stale["ready"].tolist() == [False]
    assert stale["reference_window_ready"].tolist() == [False]
    assert stale["reason_code"].tolist() == [XsensReason.ARMED_STALE]


def test_target_mismatch_is_echoed_without_mutating_authorization_or_windows():
    core, clock = ready_core(91)
    before_indices = [frame.frame_index for frame in core.current_pose_window()]
    result = core.handle_arm_command(ArmCommand(21, 90, 90))
    assert result == ArmCommandResult(
        ArmCommandClassification.TARGET_MISMATCH, True, True, True
    )
    status = core.build_status_fields(clock.now_ns)
    assert status["last_arm_command_id"].tolist() == [21]
    assert status["last_arm_target_epoch"].tolist() == [90]
    assert status["last_requested_arm_epoch"].tolist() == [90]
    assert status["accepted_arm_epoch"].tolist() == [0]
    assert status["reason_code"].tolist() == [XsensReason.ARM_COMMAND_MISMATCH]
    assert [frame.frame_index for frame in core.current_pose_window()] == before_indices


def test_old_target_disarm_after_epoch_commit_preserves_two_replayed_frames():
    core, clock = ready_core(91)
    core.handle_arm_command(ArmCommand(22, 91, 91))
    clock.advance_ns(600_000_000)
    accept_packet(core, 5, timestamp_ns=clock.now_ns)
    clock.advance_ns(1)
    accept_packet(core, 6, timestamp_ns=clock.now_ns)
    assert core.source_epoch == 92
    assert core.ready_frames == 2
    result = core.handle_arm_command(ArmCommand(23, 91, 0))
    assert result.classification is ArmCommandClassification.TARGET_MISMATCH
    assert core.source_epoch == 92
    assert core.accepted_arm_epoch == 0
    assert core.ready_frames == 2


def test_pose_fields_have_exact_shapes_dtypes_and_metadata():
    core, clock = ready_core(91)
    fields = core.build_pose_fields()
    assert set(fields) == POSE_KEYS
    assert {key: (value.dtype, value.shape) for key, value in fields.items()} == POSE_SCHEMA
    assert fields["stream_mode"].tolist() == [1]
    assert fields["source_epoch"].tolist() == [91]
    assert fields["producer_monotonic_ns"].tolist() == [core.newest_producer_monotonic_ns]


def test_status_fields_have_exact_shapes_dtypes_sentinels_and_ranges():
    empty, clock = make_core(epoch_draws=(91, 92))
    fields = empty.build_status_fields(clock.now_ns)
    assert set(fields) == STATUS_KEYS
    assert fields["producer_monotonic_ns"].tolist() == [0]
    assert fields["newest_frame_index"].tolist() == [-1]
    assert fields["last_arm_command_id"].tolist() == [0]
    assert fields["accepted_arm_epoch"].tolist() == [0]
    assert {key: (value.dtype, value.shape) for key, value in fields.items()} == STATUS_SCHEMA
    assert 0 <= int(fields["ready_frames"][0]) <= 30
    assert 0 <= int(fields["recovery_frames"][0]) <= 10
    assert int(fields["reason_code"][0]) in set(XsensReason)


def test_status_sequence_increases_for_start_change_command_and_heartbeat():
    core, clock = make_core(epoch_draws=(91, 92))
    start = int(core.build_status_fields(clock.now_ns)["status_sequence"][0])
    accept_packet(core, 1, timestamp_ns=1)
    change = int(core.build_status_fields(1)["status_sequence"][0])
    core.handle_arm_command(ArmCommand(24, 91, 0))
    receipt = int(core.build_status_fields(2)["status_sequence"][0])
    heartbeat = int(core.build_status_fields(3)["status_sequence"][0])
    assert [start, change, receipt, heartbeat] == list(range(start, start + 4))
    assert [1, 2, 3] == [
        int(core.build_status_fields(now)["status_monotonic_ns"][0])
        for now in (1, 2, 3)
    ]


def test_packed_pose_and_status_round_trip_through_existing_decoder():
    core, clock = ready_core(91)
    for topic, fields in (
        ("pose", core.build_pose_fields()),
        ("xsens_status", core.build_status_fields(clock.now_ns)),
    ):
        decoded = _decode_packed_message(pack_pose_message(fields, topic=topic), topic)
        assert set(decoded) == set(fields)
        for name in fields:
            np.testing.assert_array_equal(decoded[name], fields[name])


def test_stale_never_resets_converter_sign_history():
    class CountingConverter(XsensMotionConverter):
        def __init__(self):
            super().__init__()
            self.reset_calls = 0

        def reset_epoch(self):
            self.reset_calls += 1
            return super().reset_epoch()

    converter = CountingConverter()
    core, clock = make_core(epoch_draws=(91, 92), converter=converter)
    feed_stable_frames(core, clock, 30)
    core.handle_arm_command(ArmCommand(25, 91, 91))
    assert core.check_stale(core.newest_producer_monotonic_ns + 500_000_001)
    assert converter.reset_calls == 0


def test_reason_lifecycle_has_deterministic_precedence():
    core, clock = ready_core(91)
    core.handle_arm_command(ArmCommand(26, 90, 90))
    assert core.build_status_fields(clock.now_ns)["reason_code"].tolist() == [
        XsensReason.ARM_COMMAND_MISMATCH
    ]
    core.note_invalid_input()
    assert core.build_status_fields(clock.now_ns)["reason_code"].tolist() == [
        XsensReason.INVALID_INPUT
    ]
    feed_stable_frames(core, clock, 1)
    assert core.build_status_fields(clock.now_ns)["reason_code"].tolist() == [
        XsensReason.ARM_COMMAND_MISMATCH
    ]
    core.handle_arm_command(ArmCommand(27, 91, 0))
    assert core.build_status_fields(clock.now_ns)["reason_code"].tolist() != [
        XsensReason.ARM_COMMAND_MISMATCH
    ]
    clock.advance_ns(600_000_000)
    accept_packet(core, 5, timestamp_ns=clock.now_ns)
    clock.advance_ns(1)
    accept_packet(core, 6, timestamp_ns=clock.now_ns)
    assert core.build_status_fields(clock.now_ns)["reason_code"].tolist() == [
        XsensReason.SESSION_RESET
    ]
    clock.advance_ns(1)
    accept_packet(core, 7, timestamp_ns=clock.now_ns)
    assert core.build_status_fields(clock.now_ns)["reason_code"].tolist() != [
        XsensReason.SESSION_RESET
    ]
```

The parameter table above fixes bool, non-integer, and signed-int64 overflow
handling for all three fields.

- [ ] **Step 2: Run contract tests and verify authorization/status is incomplete**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_source_contract.py -v
```

Expected: FAIL on missing `handle_arm_command` or missing receipt/status fields.

- [ ] **Step 3: Implement stable status reasons and atomic command handling**

```python
class XsensReason(IntEnum):
    NO_DATA = 0
    COLLECTING_WINDOW = 1
    COLLECTING_STABILITY = 2
    PELVIS_UNSTABLE = 3
    SEGMENT_UNSTABLE = 4
    READY = 5
    ARMED_FRESH = 6
    ARMED_STALE = 7
    ARMED_RECOVERING = 8
    ARM_COMMAND_MISMATCH = 9
    SESSION_RESET = 10
    INVALID_INPUT = 11
```

Convert each command field with `operator.index`, rejecting booleans and any
non-integral value. Require command ID and target in `1..2**63-1`; requested
epoch must be integer zero or exactly that target. A matching arm is accepted
only with current epoch, current ready/fresh state, and a complete window. A
matching disarm always sets authorization to zero and clears
stability/readiness/output evidence before recording its receipt, even when
already unarmed. A mismatched target is echoed with reason
`ARM_COMMAND_MISMATCH` but cannot touch the current epoch's two replayed frames
or windows. `ArmCommandResult.publish_immediately` is true for every first-seen
structurally valid receipt, including mismatch and not-ready; it is false for
malformed commands, an exact duplicate, and an ID reused with changed payload.
Exact duplicates remain visible only through the normal heartbeat and never
restart readiness.

Construct pose fields with exact dtypes/shapes:

```text
frame_index int64[10]                 smpl_joints float32[10,24,3]
body_quat_w float32[10,4]             joint_pos float32[10,29]
stream_mode int32[1] = 1              calibration_ready bool[1]
producer_monotonic_ns int64[1]        source_epoch int64[1]
```

Construct all fifteen status fields from the approved contract, using `newest_frame_index=-1` and producer timestamp `0` before data. Increment `status_sequence` every time `build_status_fields()` produces startup, change, command-receipt, or heartbeat output; never use Xsens time code as `status_monotonic_ns` or freshness. Reason precedence is deterministic: invalid parser/conversion/command input since the last accepted packet reports `INVALID_INPUT`; a committed epoch reports `SESSION_RESET` until the next active-compatible packet after its two replayed frames; a latest target-mismatch receipt reports `ARM_COMMAND_MISMATCH` until a later first-seen valid command or epoch; otherwise derive the reason from no-data/window/stability/ready/armed stale/recovering/fresh state. Expose `note_invalid_input()` for Task 10 and clear only the corresponding transient flag on the stated recovery event.

- [ ] **Step 4: Run all pure Xsens source tests**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_source_epoch.py test/test_xsens_source_core.py test/test_xsens_source_contract.py -v
```

Expected: PASS, including delayed old-target disarm and exact duplicate-command behavior.

- [ ] **Step 5: Commit authorization and contracts**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/source_core.py src/bxi_example_py_elf3/test/xsens_test_helpers.py src/bxi_example_py_elf3/test/test_xsens_source_contract.py
git commit -m "feat(xsens): publish arm and status contracts"
```

### Task 10: Add the State-Scoped ROS/ZMQ Xsens Source Node

**Files:**

- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/source_node.py`
- Modify: `src/bxi_example_py_elf3/test/xsens_test_helpers.py`
- Create: `src/bxi_example_py_elf3/test/test_xsens_source_node.py`

**Interfaces:**

- Consumes: Tasks 4-9 parser, receiver, converter, source core, `pack_pose_message`, and ROS `Int64MultiArray`.
- Produces: `validate_source_params(raw)`, `XsensPoseStatusPublisher`, `XsensSourceNode`, and `create_node(context)`.
- Lifecycle: source node exclusively binds UDP 9763 and ZMQ 5559 while `sonic_xsens` is active.

```text
XsensPoseStatusPublisher(
  host: str, port: int, *, zmq_context: zmq.Context | None = None
)
send(topic: str, fields: Mapping[str, np.ndarray]) -> bool
close() -> None

XsensSourceNode(
  context: NodeBuildContext, *,
  clock_ns: Callable[[], int] = time.monotonic_ns,
  epoch_factory: Callable[[], int] = random_epoch_candidate,
  receiver_factory: Callable[..., XsensUdpReceiver] = XsensUdpReceiver,
  converter_factory: Callable[[], XsensMotionConverter] = XsensMotionConverter,
  core_factory: Callable[..., XsensSourceCore] = XsensSourceCore,
  publisher_factory: Callable[..., XsensPoseStatusPublisher] = XsensPoseStatusPublisher
)
```

`random_epoch_candidate() -> int` returns `secrets.randbits(63)` without
filtering; `draw_new_epoch()` in the core owns the zero/current/collision retry
rule. The node passes `epoch_factory` unchanged into `XsensSourceCore`.

When it creates the context it owns and terminates it; when a context is injected it closes only its socket. `XsensSourceNode` owns one publisher and calls only `publisher.close()`—it never separately terminates the publisher's context.

- [ ] **Step 1: Write failing parameter, callback, publication-order, and cleanup tests**

```python
def make_arm_message(command_id, target_epoch, requested_epoch):
    message = Int64MultiArray()
    message.layout.dim = []
    message.layout.data_offset = 0
    message.data = [command_id, target_epoch, requested_epoch]
    return message


def test_arm_callback_rejects_nonempty_layout_and_bad_length(node_harness):
    message = Int64MultiArray()
    message.layout.dim = [MultiArrayDimension(label="invalid", size=3, stride=3)]
    message.data = [1, 2, 2]
    node_harness.node._arm_command_callback(message)
    assert node_harness.core.commands == []

    message = Int64MultiArray()
    message.data = [1, 2]
    node_harness.node._arm_command_callback(message)
    assert node_harness.core.commands == []


def test_epoch_status_precedes_first_new_epoch_pose(node_harness):
    node_harness.publisher.sent.clear()
    node_harness.core.accept_results = [
        AcceptResult(True, AcceptClassification.EPOCH_COMMITTED, True)
    ]
    node_harness.receiver.pending.append(ReceivedDatagram(
        build_mxtp02_packet(), 1, ("127.0.0.1", 4000)
    ))
    node_harness.node._tick()
    assert [topic for topic, fields in node_harness.publisher.sent[:2]] == [
        "xsens_status", "pose"
    ]


def test_status_send_failure_suppresses_pose_until_a_status_succeeds(node_harness):
    node_harness.publisher.sent.clear()
    node_harness.publisher.fail_next_topic = "xsens_status"
    node_harness.core.accept_results = [
        AcceptResult(True, AcceptClassification.EPOCH_COMMITTED, True)
    ]
    node_harness.receiver.pending.append(ReceivedDatagram(
        build_mxtp02_packet(), 1, ("127.0.0.1", 4000)
    ))
    node_harness.node._tick()
    assert "pose" not in [topic for topic, fields in node_harness.publisher.sent]

    node_harness.node._tick()
    assert [topic for topic, fields in node_harness.publisher.sent[-2:]] == [
        "xsens_status", "pose"
    ]


def test_exact_duplicate_command_waits_for_normal_heartbeat(node_harness):
    message = make_arm_message(31, node_harness.core.source_epoch, 0)
    node_harness.node._arm_command_callback(message)
    sends_after_first = len(node_harness.publisher.sent)
    node_harness.node._arm_command_callback(message)
    assert len(node_harness.publisher.sent) == sends_after_first
    node_harness.node._tick()
    assert node_harness.publisher.sent[-1][0] == "xsens_status"
```

Define the harness used by all node tests in the same test module. It injects
dependencies through the constructor contract above and therefore never binds
9763/5559:

```python
MOD_ROOT = Path(__file__).resolve().parents[1] / "mods" / "com.bxi.sonic"


@pytest.fixture
def rclpy_runtime():
    started_here = not rclpy.ok()
    if started_here:
        rclpy.init(args=[])
    yield
    if started_here and rclpy.ok():
        rclpy.shutdown()


def source_context(udp_port: int, pose_port: int, node_name: str) -> NodeBuildContext:
    params = dict(SOURCE_DEFAULTS)
    params.update({
        "udp_bind_host": "127.0.0.1",
        "udp_port": udp_port,
        "allowed_sender": "127.0.0.1",
        "pose_port": pose_port,
    })
    return NodeBuildContext(
        mod_id="com.bxi.sonic",
        node_id=f"com.bxi.sonic/{node_name}",
        node_name=node_name,
        mod_root=MOD_ROOT,
        params=params,
    )


@pytest.fixture
def xsens_source_context() -> NodeBuildContext:
    return source_context(reserve_udp_port(), reserve_tcp_port(), "xsens_unit")


class FakeReceiver:
    def __init__(self):
        self.pending = []
        self.closed = False
        self.stats = ReceiverStats()

    def drain(self, limit=256):
        batch, self.pending = self.pending[:limit], self.pending[limit:]
        return batch

    def close(self):
        self.closed = True


class FakePublisher:
    def __init__(self):
        self.sent = []
        self.fail_next_topic = None
        self.closed = False
        self.drop_count = 0

    def send(self, topic, fields):
        if self.fail_next_topic == topic:
            self.fail_next_topic = None
            self.drop_count += 1
            return False
        self.sent.append((topic, {key: value.copy() for key, value in fields.items()}))
        return True

    def close(self):
        self.closed = True


class FakeCore:
    def __init__(self):
        self.source_epoch = 71
        self.commands = []
        self.accept_results = []
        self.status_sequence = 0
        self.pose_fields = make_xsens_pose_fields(source_epoch=71)
        self.stats = SourceCoreStats()
        self.locked_sender = ("127.0.0.1", 4000, 0)
        self.newest_frame_index = 109
        self.newest_producer_monotonic_ns = 1_000_000_000

    def expire(self, now_ns):
        return False

    def check_stale(self, now_ns):
        return False

    def accept(self, packet):
        if self.accept_results:
            return self.accept_results.pop(0)
        return AcceptResult(True, AcceptClassification.FORWARD)

    def note_invalid_input(self):
        pass

    def handle_arm_command(self, command):
        if command in self.commands:
            return ArmCommandResult(
                ArmCommandClassification.DUPLICATE, False, False, False
            )
        self.commands.append(command)
        return ArmCommandResult(
            ArmCommandClassification.DISARMED, True, True, True
        )

    def build_status_fields(self, now_ns):
        self.status_sequence += 1
        return make_status(
            status_sequence=self.status_sequence,
            status_monotonic_ns=now_ns,
            source_epoch=self.source_epoch,
        )

    def build_pose_fields(self):
        return self.pose_fields


class NodeHarness:
    def __init__(self, context, *, integrated=False):
        self.clock = FakeClock(1_000_000_000)
        self.receiver = FakeReceiver()
        self.publisher = FakePublisher()
        self.converter = XsensMotionConverter()
        if integrated:
            draws = iter((71, 72, 73))
            self.core = XsensSourceCore(
                self.converter,
                clock_ns=self.clock.monotonic_ns,
                epoch_factory=lambda: next(draws),
            )
        else:
            self.core = FakeCore()
        self.next_counter = 1
        self.node = XsensSourceNode(
            context,
            clock_ns=self.clock.monotonic_ns,
            receiver_factory=lambda **kwargs: self.receiver,
            converter_factory=lambda: self.converter,
            core_factory=lambda *args, **kwargs: self.core,
            publisher_factory=lambda *args, **kwargs: self.publisher,
        )

    def feed(self, *, counter, timestamp_ns, time_code=0):
        self.clock.now_ns = timestamp_ns
        self.receiver.pending.append(ReceivedDatagram(
            payload=build_mxtp02_packet(
                sample_counter=counter, time_code=time_code
            ),
            receive_timestamp_ns=timestamp_ns,
            sender_address=("127.0.0.1", 4000),
        ))
        self.node._tick()


@pytest.fixture
def node_harness(rclpy_runtime, xsens_source_context):
    harness = NodeHarness(xsens_source_context)
    yield harness
    harness.node.destroy_node()


@pytest.fixture
def integrated_node_harness(rclpy_runtime, xsens_source_context):
    harness = NodeHarness(xsens_source_context, integrated=True)
    yield harness
    harness.node.destroy_node()
```

Add this explicit validation and lifecycle matrix:

```python
@pytest.mark.parametrize(
    "params,match",
    [
        ({"unknown": 1}, "unknown"),
        ({"udp_port": 0}, "udp_port"),
        ({"udp_port": True}, "udp_port"),
        ({"pose_port": 65536}, "pose_port"),
        ({"pose_host": "0.0.0.0"}, "pose_host"),
        ({"pose_topic": ""}, "pose_topic"),
        ({"status_topic": ""}, "status_topic"),
        ({"arm_command_topic": ""}, "arm_command_topic"),
        ({"status_rate_hz": 49.0}, "status_rate_hz"),
        ({"input_rate_hz": 50.0}, "input_rate_hz"),
        ({"publish_rate_hz": 60.0}, "publish_rate_hz"),
        ({"window_frames": 9}, "window_frames"),
        ({"same_epoch_resume_frames": 9}, "same_epoch_resume_frames"),
        ({"ready_frames": 29}, "ready_frames"),
        ({"stale_seconds": 0.0}, "stale_seconds"),
        ({"epoch_candidate_frames": 1}, "epoch_candidate_frames"),
        ({"epoch_candidate_timeout_s": 0.0}, "epoch_candidate_timeout_s"),
        ({"max_pelvis_span_m": -0.1}, "max_pelvis_span_m"),
        ({"max_segment_deviation_deg": 181.0}, "max_segment_deviation_deg"),
    ],
)
def test_source_rejects_unknown_or_unsafe_params(params, match):
    with pytest.raises(ValueError, match=match):
        validate_source_params(params)


def test_arm_subscription_is_reliable_and_volatile(node_harness):
    topic = node_harness.node.resolve_topic_name("sonic/xsens_arm_command")
    endpoints = node_harness.node.get_subscriptions_info_by_topic(topic)
    assert len(endpoints) == 1
    qos = endpoints[0].qos_profile
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.VOLATILE
    assert qos.history is HistoryPolicy.KEEP_LAST
    assert qos.depth == 10


def test_startup_and_empty_ticks_publish_status_heartbeat(node_harness):
    assert [topic for topic, _ in node_harness.publisher.sent] == ["xsens_status"]
    first = int(node_harness.publisher.sent[-1][1]["status_sequence"][0])
    node_harness.node._tick()
    second = int(node_harness.publisher.sent[-1][1]["status_sequence"][0])
    assert node_harness.publisher.sent[-1][0] == "xsens_status"
    assert second == first + 1


def test_tick_drains_in_order_and_publishes_only_newest_progressing_window(
    integrated_node_harness,
):
    harness = integrated_node_harness
    start_ns = harness.clock.now_ns
    source_times = [
        start_ns + ((index + 1) * 1_000_000_000) // 60
        for index in range(60)
    ]
    queued = 0
    for tick in range(1, 51):
        cutoff_ns = start_ns + tick * 20_000_000
        while queued < len(source_times) and source_times[queued] <= cutoff_ns:
            counter = queued + 1
            harness.receiver.pending.append(ReceivedDatagram(
                payload=build_mxtp02_packet(sample_counter=counter),
                receive_timestamp_ns=source_times[queued],
                sender_address=("127.0.0.1", 4000),
            ))
            queued += 1
        harness.clock.now_ns = cutoff_ns
        harness.node._tick()
    newest = [
        int(fields["frame_index"][-1])
        for topic, fields in harness.publisher.sent if topic == "pose"
    ]
    assert newest
    assert all(right > left for left, right in zip(newest, newest[1:]))
    assert newest[-1] == 60
    assert len(newest) <= 50


def test_malformed_packet_between_candidates_does_not_advance_candidate(
    integrated_node_harness,
):
    harness = integrated_node_harness
    harness.feed(counter=1000, timestamp_ns=1_000_000_000)
    harness.feed(counter=900, timestamp_ns=1_600_000_000)
    harness.receiver.pending.append(ReceivedDatagram(
        payload=b"bad", receive_timestamp_ns=1_610_000_000,
        sender_address=("127.0.0.1", 4000),
    ))
    harness.node._tick()
    assert harness.core.candidate_frame_count == 1
    harness.feed(counter=901, timestamp_ns=1_620_000_000)
    assert harness.core.source_epoch == 72


def test_conversion_failure_is_transactional_and_publishes_no_partial_pose(node_harness):
    node_harness.core.accept_results = [
        AcceptResult(False, AcceptClassification.CONVERSION_REJECTED)
    ]
    node_harness.core.pose_fields = None
    node_harness.receiver.pending.append(ReceivedDatagram(
        build_mxtp02_packet(), 1, ("127.0.0.1", 4000)
    ))
    before_pose_count = sum(t == "pose" for t, _ in node_harness.publisher.sent)
    node_harness.node._tick()
    after_pose_count = sum(t == "pose" for t, _ in node_harness.publisher.sent)
    assert after_pose_count == before_pose_count
    assert node_harness.publisher.sent[-1][0] == "xsens_status"


def test_bind_failure_rolls_back_receiver_and_publisher(
    rclpy_runtime, xsens_source_context,
):
    receiver = FakeReceiver()
    with pytest.raises(RuntimeError, match="bind failed"):
        XsensSourceNode(
            xsens_source_context,
            receiver_factory=lambda **kwargs: receiver,
            publisher_factory=lambda *args, **kwargs: (_ for _ in ()).throw(
                RuntimeError("bind failed")
            ),
        )
    assert receiver.closed


def test_destroy_node_is_idempotent(node_harness):
    first = node_harness.node.destroy_node()
    second = node_harness.node.destroy_node()
    assert second == first
    assert node_harness.receiver.closed and node_harness.publisher.closed


def test_repeated_source_lifecycle_uses_reserved_nonproduction_ports(rclpy_runtime):
    udp_port, pose_port = reserve_udp_port(), reserve_tcp_port()
    for suffix in ("a", "b"):
        node = XsensSourceNode(source_context(udp_port, pose_port, f"xsens_{suffix}"))
        node.destroy_node()


class CaptureLogger:
    def __init__(self):
        self.events = []

    def info(self, message):
        self.events.append(("info", str(message)))

    def warning(self, message):
        self.events.append(("warning", str(message)))

    def error(self, message):
        self.events.append(("error", str(message)))


def test_diagnostics_are_transition_or_summary_scoped_without_pose_arrays(node_harness):
    logger = CaptureLogger()
    node_harness.node.get_logger = lambda: logger
    node_harness.node._last_diagnostic_summary_ns = node_harness.clock.now_ns
    for _ in range(100):
        node_harness.clock.advance_ns(20_000_000)
        node_harness.node._tick()
    transition_count = len(logger.events)
    assert transition_count <= 1
    node_harness.clock.advance_ns(5_000_000_000)
    node_harness.node._tick()
    assert len(logger.events) == transition_count + 1
    text = "\n".join(message for _, message in logger.events)
    assert "term1_local" not in text
    assert "smpl_joints" not in text
    assert "array(" not in text
```

Use `DIAGNOSTIC_SUMMARY_SECONDS = 5.0` in `source_node.py`; the fake-clock test
above fixes its observable behavior.

- [ ] **Step 2: Run node tests and verify the module is missing**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_source_node.py -v
```

Expected: collection FAIL importing `xsens.source_node`.

- [ ] **Step 3: Implement validated parameters, ordered publishing, and rollback**

```python
SOURCE_DEFAULTS = {
    "udp_bind_host": "0.0.0.0", "udp_port": 9763,
    "allowed_sender": "127.0.0.1",
    "pose_host": "127.0.0.1", "pose_port": 5559,
    "pose_topic": "pose", "status_topic": "xsens_status",
    "arm_command_topic": "sonic/xsens_arm_command",
    "status_rate_hz": 50.0, "input_rate_hz": 60.0,
    "publish_rate_hz": 50.0, "window_frames": 10,
    "same_epoch_resume_frames": 10, "ready_frames": 30,
    "stale_seconds": 0.5, "epoch_candidate_frames": 2,
    "epoch_candidate_timeout_s": 0.25,
    "max_pelvis_span_m": 0.15,
    "max_segment_deviation_deg": 20.0,
}
```

Bind one PUB socket to `tcp://127.0.0.1:5559`, use topics `pose` and
`xsens_status`, `LINGER=0`, and nonblocking sends. `send()` returns false on
`zmq.Again` and increments a drop counter. Subscribe to the arm topic with
reliable, volatile `KEEP_LAST(depth=10)` QoS; accept only `layout.dim == []`,
`data_offset == 0`, and three signed-int64-compatible values. A malformed
layout/length/value calls `core.note_invalid_input()` and never reaches
`handle_arm_command`. Publish one startup status after every resource is ready;
the callback publishes immediately only when
`ArmCommandResult.publish_immediately` is true. Serialize callback/tick
core+publisher access with one node lock.

Each 50 Hz tick captures one `now_ns`, calls `core.expire(now_ns)`, drains,
parses, and accepts all queued datagrams in order, and finally calls
`core.check_stale(now_ns)`. `accept()` also evaluates the gap from the prior
active producer before each accepted packet, so a stale edge inside a drained
backlog cannot be skipped. Malformed bytes call `note_invalid_input()`;
conversion rejection is transactional. Build and send status before pose, and
suppress the corresponding pose whenever status send fails. Track the last
successfully published newest frame index, retry an unsent current window,
publish at most one newest complete progressing window per tick, and never mark
it sent before `send()` returns true.

Track received/accepted/malformed/duplicate/ambiguous/backward/inferred-drop/
ZMQ-drop counts using receiver stats, `AcceptResult.counter_kind`, and
`SourceCoreStats`. Log sender, measured rate, producer/status age,
readiness/recovery, and reason only on changes or five-second summaries, never
frame arrays. Validate `same_epoch_resume_frames == window_frames == 10` and
pass both into the core. Resources are constructed receiver -> publisher ->
subscription -> timer and cleaned in the exact reverse order timer ->
subscription -> publisher -> receiver before `super().destroy_node()`;
publisher alone owns and terminates its internally created context.

- [ ] **Step 4: Run source-node and packed-contract regressions**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_source_node.py test/test_xsens_source_contract.py test/test_zerolab_lifecycle.py -v
```

Expected: PASS with no port/context leak and no pose after an unsent status barrier. Unit lifecycle tests override the manifest ports with `reserve_udp_port()`/`reserve_tcp_port()`; exact 9763/5559 ownership is covered by Task 15 manifest plus stopped-listener manual validation.

- [ ] **Step 5: Commit the Xsens source process**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/xsens/source_node.py src/bxi_example_py_elf3/test/xsens_test_helpers.py src/bxi_example_py_elf3/test/test_xsens_source_node.py
git commit -m "feat(xsens): add state scoped source node"
```

### Task 11: Add the Authoritative Xsens Bridge and Immediate Status Passthrough

**Files:**

- Modify: `src/bxi_example_py_elf3/mods/com.bxi.sonic/pico/pose_to_smpl_ref_bridge.py:76-88,200-301,470-715`
- Create: `src/bxi_example_py_elf3/test/test_xsens_bridge_contract.py`
- Reuse: `src/bxi_example_py_elf3/test/xsens_test_helpers.py` (`FakeClock`, `make_status`, `make_xsens_pose_fields`, `reserve_tcp_port` from Tasks 4 and 9)
- Regression test without modification: `src/bxi_example_py_elf3/test/test_zerolab_pose_contract.py`
- Regression test without modification: `src/bxi_example_py_elf3/test/test_zerolab_lifecycle.py`

**Interfaces:**

- Consumes: Task 10 packed `pose` and `xsens_status` on one ordered ZMQ connection.
- Produces: `_build_authoritative_xsens_smpl_ref(fields)`, `_validate_xsens_status_fields(fields)`, and explicit `source_kind="xsens"` bridge behavior.
- Parameter API: `_validated_bridge_params(raw: Mapping[str, object]) -> dict[str, object]` canonicalizes aliases before defaults and validation.
- Compatibility: metadata alone never selects the fast path; `source_kind="legacy"` retains merger, gap handling, and default three-message debounce.
- Freshness: `MAX_PRODUCER_AGE_NS = 500_000_000`; age equal to the limit is fresh and age greater than the limit is stale.

```text
PackedMessageInput.drain() -> tuple[bytes, ...]
PackedMessageInput.close() -> None
PackedTopicOutput.send(topic: str, fields: Mapping[str, np.ndarray]) -> bool
PackedTopicOutput.close() -> None

SmplRefBridgeNode(
  context: NodeBuildContext, *,
  monotonic: Callable[[], float] = time.monotonic,
  monotonic_ns: Callable[[], int] = time.monotonic_ns,
  input_transport: PackedMessageInput | None = None,
  output_transport: PackedTopicOutput | None = None
)
```

When either transport argument is `None`, the node creates the corresponding
ZMQ transport; it owns/closes only transports it creates. Production output wraps
`pack_pose_message` and converts `zmq.Again` into `False`.

- [ ] **Step 1: Write failing authoritative-window and status-forwarding tests**

Create `test_xsens_bridge_contract.py` with these imports, exact schemas, and
tests. Everything in this step is in that one module unless a shared helper is
explicitly imported from `xsens_test_helpers.py`:

```python
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
```

Define the local runtime fixture and in-memory transport harness in the same
module. Every harness context uses two distinct, temporarily reserved
nonproduction TCP ports even though injected transports do not bind them:

```python
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
```

Append these executable ordering, freshness, compatibility, parameter, and
lifecycle tests to the same module:

```python
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
```

Append the complete schema-rejection matrices. They mutate a valid Task 9
message one dimension at a time, so every rejection has a single cause:

```python
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
        pytest.param(
            {
                "producer_monotonic_ns": 0,
                "newest_frame_index": -1,
                "ready": False,
                "reference_window_ready": False,
                "ready_frames": 0,
            },
            id="positive-epoch-with-no-data-sentinels",
        ),
    ],
)
def test_xsens_status_rejects_producer_sentinel_mismatch_or_bad_index(changes):
    with pytest.raises(ValueError):
        _validate_xsens_status_fields(_status_with(**changes))


def test_xsens_status_accepts_exact_no_data_sentinels():
    fields = _status_with(
        source_epoch=0,
        producer_monotonic_ns=0,
        newest_frame_index=-1,
        ready=False,
        reference_window_ready=False,
        ready_frames=0,
        reason_code=XsensReason.NO_DATA,
    )
    validated = _validate_xsens_status_fields(fields)
    assert set(validated) == STATUS_KEYS
    for name, value in fields.items():
        np.testing.assert_array_equal(validated[name], value)


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
```

- [ ] **Step 2: Run bridge tests and verify the Xsens branch is missing**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_bridge_contract.py -v
```

Expected: collection FAIL importing `_build_authoritative_xsens_smpl_ref`.

- [ ] **Step 3: Implement explicit bridge parameters and one ordered transport**

Use these canonical defaults and aliases in
`pose_to_smpl_ref_bridge.py`; aliases are accepted only in raw configuration
and never appear in the returned mapping:

```python
MAX_PRODUCER_AGE_NS = 500_000_000

BRIDGE_DEFAULTS = {
    "pico_host": PICO_HOST,
    "pico_port": PICO_PORT,
    "out_host": SMPL_REF_HOST,
    "out_port": SMPL_REF_PORT,
    "source_kind": "legacy",
    "input_pose_topic": PICO_TOPIC,
    "input_status_topic": "xsens_status",
    "output_reference_topic": SMPL_REF_TOPIC,
    "output_status_topic": "xsens_status",
    "authoritative_input_window": False,
    "readiness_debounce_messages": 3,
    "rate_hz": DEFAULT_RATE_HZ,
    "history_frames": HISTORY_FRAMES,
    "max_gap_frames": MAX_GAP_FRAMES,
    "catch_up_enabled": True,
    "stale_warning_seconds": PICO_STALE_SECONDS,
}
BRIDGE_ALIASES = {
    "pico_topic": "input_pose_topic",
    "out_topic": "output_reference_topic",
}
```

Implement `_validated_bridge_params(raw)` in this exact order:

1. Copy `raw`; allowed raw keys are `set(BRIDGE_DEFAULTS) |
   set(BRIDGE_ALIASES)`. For `unknown = set(normalized) - allowed`, raise
   `ValueError(f"unknown bridge params: {sorted(unknown)}")` whenever `unknown`
   is nonempty, before applying defaults.
2. For each alias/canonical pair, copy an alias-only value to the canonical
   key; accept both keys only when their values compare equal; raise
   `ValueError(f"conflicting bridge params: {alias} and {canonical}")` when
   they differ; remove the alias in either accepted case.
3. Produce every canonical key with `normalized.get(name, default)` only after
   steps 1 and 2.
4. Require `source_kind` to be exactly `"legacy"` or `"xsens"`. Require host
   and topic values to be nonempty strings. Require ports to be non-boolean
   integers in `1..65535`; `history_frames` and `max_gap_frames` to be
   non-boolean integers at least zero; `readiness_debounce_messages` to be a
   non-boolean integer at least one; `rate_hz` and `stale_warning_seconds` to be
   non-boolean finite numbers greater than zero; and both boolean parameters
   to have exact `bool` type.
5. For Xsens, additionally require `authoritative_input_window is True` and
   `readiness_debounce_messages == 1`. Selecting the authoritative path depends
   only on `source_kind == "xsens"`; a legacy message's optional metadata never
   changes its branch. The legacy defaults remain false and three.
6. Return only the canonical mapping. Keep `_validated_params(raw)` as a thin
   compatibility call to `_validated_bridge_params(raw)` for callers outside
   this task that imported the old private name.

Add two concrete production transport adapters behind the interfaces declared
at the start of this task:

- `ZmqPackedMessageInput(host, port, topics)` owns one ZMQ context and one SUB
  socket, sets `LINGER=0` and `RCVHWM=64`, subscribes once to each supplied
  topic, connects to `tcp://host:port`, and has `drain()` repeatedly call
  nonblocking `recv()` until `zmq.Again`, returning the received bytes as one
  ordered tuple. Its idempotent `close()` closes the socket before terminating
  its context.
- `ZmqPackedTopicOutput(host, port)` owns one ZMQ context and one PUB socket,
  sets `LINGER=0` and `SNDHWM=64`, and binds `tcp://host:port`. `send(topic,
  fields)` calls `pack_pose_message(fields, topic=topic, version=4)` and a
  nonblocking socket send; it returns false only for `zmq.Again` and true only
  after the bytes were accepted. Its idempotent `close()` closes the socket
  before terminating its context.
- Xsens constructs one input adapter with `(input_pose_topic,
  input_status_topic)` and one output adapter shared by status and reference.
  Legacy constructs its input with only `input_pose_topic`. When a constructor
  argument supplies an adapter, the node borrows it and never closes it. On
  partial construction failure and on `destroy_node()`, close only adapters the
  node created.

Classify each raw input message before decoding. For a configured topic
`topic`, an exact packed message must start with
`topic.encode("utf-8") + b"{"`, because the first byte after the topic is the
first byte of the 1280-byte JSON header. Check the configured status topic and
pose topic independently, then pass the exact matched topic to
`_decode_packed_message`. Ignore bytes matching neither marker. This rejects
`pose_suffix` even though the SUB socket's transport filter accepts it. Catch
decode/schema exceptions per message, count/log the rejection, and continue
draining later messages.

Use these exact runtime schemas in the two validators; require every value to
be an `np.ndarray` with the listed dtype and shape and reject both missing and
extra keys. Every `ValueError` for key-set, dtype, shape, or numeric-array
failure names the offending field (including an unexpected key):

```text
pose:
  frame_index             int64   (10,)
  smpl_joints             float32 (10,24,3)
  body_quat_w             float32 (10,4)
  joint_pos               float32 (10,29)
  stream_mode             int32   (1,)
  calibration_ready       bool    (1,)
  producer_monotonic_ns   int64   (1,)
  source_epoch            int64   (1,)

status (all shapes are (1,)):
  int64: status_sequence, status_monotonic_ns, source_epoch,
         last_arm_command_id, last_arm_target_epoch,
         last_requested_arm_epoch, accepted_arm_epoch,
         producer_monotonic_ns, newest_frame_index
  bool:  ready, reference_window_ready, source_stale
  int32: ready_frames, recovery_frames, reason_code
```

`_build_authoritative_xsens_smpl_ref(fields)` performs all validation before
allocating output: require finite `smpl_joints`, `body_quat_w`, and `joint_pos`;
all ten root-quaternion norms greater than zero; `stream_mode[0] == 1`;
`producer_monotonic_ns[0] > 0`; `source_epoch[0] > 0`; and ten strictly
increasing frame indices. It then returns exactly this mapping, with no merger,
fill, interpolation, clamping, or readiness streak:

```python
{
    "term1_local": np.ascontiguousarray(
        fields["smpl_joints"].reshape(10, 72), dtype=np.float32
    ),
    "root_quat": np.ascontiguousarray(
        fields["body_quat_w"], dtype=np.float32
    ),
    "wrist": np.ascontiguousarray(
        fields["joint_pos"][:, ELF3_NATIVE_WRIST_IDX], dtype=np.float32
    ),
    "frame_index": np.array([fields["frame_index"][0]], dtype=np.int64),
    "source_ready": np.array(
        [fields["calibration_ready"][0]], dtype=np.bool_
    ),
    "producer_monotonic_ns": np.array(
        fields["producer_monotonic_ns"], copy=True
    ),
    "source_epoch": np.array(fields["source_epoch"], copy=True),
    "source_newest_frame_index": np.array(
        [fields["frame_index"][9]], dtype=np.int64
    ),
}
```

`_validate_xsens_status_fields(fields)` validates all fifteen arrays and then
enforces these scalar relationships before returning a copied mapping for
passthrough:

1. `status_sequence >= 1`; `status_monotonic_ns >= 0`; and all epoch, receipt,
   authorization, producer-time, and frame scalars are nonnegative except the
   one permitted `newest_frame_index == -1` sentinel.
2. With `last_arm_command_id == 0`, target and requested epochs must both be
   zero. With a positive command ID, target must be positive and requested must
   be zero or equal to target.
3. `accepted_arm_epoch` is zero or equals the positive `source_epoch`; a
   positive authorization also requires a positive receipt command ID.
4. No-data is represented only by the pair
   `(producer_monotonic_ns, newest_frame_index) == (0, -1)` and requires
   `source_epoch == 0`, `ready == false`, and
   `reference_window_ready == false`. Data-present status requires a positive
   producer timestamp, nonnegative newest index, and positive source epoch.
5. `source_stale == true` requires both readiness booleans false; `ready ==
   true` requires `reference_window_ready == true`; either readiness boolean
   requires data-present status.
6. Require `0 <= ready_frames <= 30`, `0 <= recovery_frames <= 10`, and
   `reason_code` to be one of the integer values in `XsensReason` (0 through
   11). Do not reinterpret or drop any valid status field.

Store both injected clocks and make all runtime time reads use them:

```text
self._monotonic = monotonic
self._monotonic_ns = monotonic_ns
```

The names `time.monotonic` and `time.monotonic_ns` appear only as constructor
default arguments. Neither `_drain_input`, status/pose handlers, the legacy
readiness path, diagnostics, nor `_tick` calls the `time` module directly.
Each tick calls each injected clock once and passes the captured `now_mono` and
`now_ns` down to every helper.

Initialize this Xsens state explicitly:

```text
_pending_xsens_pose: dict[str, np.ndarray] | None = None
_pending_status: dict[str, np.ndarray] | None = None
_last_forwarded_status_epoch: int | None = None
_last_forwarded_status_frame: int = -1
_last_forwarded_status_sequence: int = 0
_last_xsens_epoch_seen: int | None = None
_last_xsens_status_sequence_seen: int = 0
_last_xsens_pose_epoch: int | None = None
_last_xsens_pose_newest_seen: int = -1
_status_send_failed_this_tick: bool = False
```

Process a validated Xsens status in arrival order with this algorithm:

1. Read its epoch, sequence, stale flag, and newest frame. If the epoch differs
   from `_last_xsens_epoch_seen`, set the seen epoch, reset the per-epoch status
   sequence, discard an old-epoch `_pending_status`, invalidate the forwarded
   barrier, and clear `_pending_xsens_pose` only when that pose's epoch is not
   the new epoch. Reset the pose progress watermark when it belongs to another
   epoch.
2. Regardless of send outcome, a valid `source_stale == true` status clears
   `_pending_xsens_pose` and invalidates the forwarded barrier immediately.
3. Ignore a same-epoch status whose sequence is not greater than
   `_last_xsens_status_sequence_seen`. Otherwise advance that watermark and
   replace `_pending_status` with a copy of this newest status; an older pending
   status is never retried after replacement.
4. Attempt the new status immediately through the shared output transport. On
   false, retain it and set `_status_send_failed_this_tick = true`. On true,
   clear that pending status and record its sequence. A successful non-stale
   status sets the forwarded barrier epoch/frame/sequence; a successful stale
   status leaves the barrier invalid while still recording its forwarded
   sequence.

Process a validated Xsens pose in arrival order with this algorithm:

1. Build the exact authoritative reference and read its copied epoch, newest
   frame, and producer timestamp. If a status epoch is already known and does
   not match the pose epoch, discard the pose.
2. Compute age only as `now_ns - producer_monotonic_ns`. Discard a pose when
   age is greater than `MAX_PRODUCER_AGE_NS`; equality is accepted. Never store
   receive time in the pose and never replace its producer timestamp.
3. Reset the cross-message newest-frame watermark on a new pose epoch. Within
   one epoch, discard a newest frame not strictly greater than the watermark.
   Otherwise advance the watermark and replace `_pending_xsens_pose`, so a
   newer progressing pose supersedes an older unsent one.

Run one tick in this exact order:

1. If closed, return. Capture `now_mono = self._monotonic()` and `now_ns =
   self._monotonic_ns()` exactly once and clear the per-tick status-failure flag.
2. Call the input transport's `drain()` once and process every returned message
   sequentially. Xsens status uses the immediate-send algorithm; Xsens pose
   uses the authoritative algorithm; legacy pose uses only
   `PicoSourceReadinessGate(required_consecutive=readiness_debounce_messages)`,
   `_parse_incoming_chunk`, and `StreamedSmplRefMerger`.
3. If any newly received status send returned false, return without retrying it
   in the same tick and without attempting a reference. This fixes observable
   order under a one-shot failure.
4. If `_pending_status` was carried from an earlier tick, retry it once through
   the shared output. On false, return. On true, update/invalidate the barrier
   exactly as for an immediate success and clear the pending status.
5. For legacy, if the readiness gate accepted a pending pose, call
   `_parse_incoming_chunk`, merge the resulting chunk once, clear the pending
   fields, and reset the gate/merger on a parse failure. Then call
   `_build_live_smpl_ref_if_ready(source_gate, merger, now_mono,
   stale_warning_seconds)` exactly once. When it returns fields, send them on
   `output_reference_topic`; retain the existing send-drop counter behavior.
   Compute diagnostic input age from the last legacy receive timestamp and the
   same `now_mono`, update transition-only waiting/streaming/stale logs, and
   return. Xsens metadata never invokes this path, and legacy metadata never
   invokes the Xsens path.
6. For Xsens, return if there is no pending pose. Recompute producer age with
   the same captured `now_ns`; clear and return only when age is greater than
   500,000,000 ns. Return while a status remains pending.
7. Send the reference only when
   `_last_forwarded_status_epoch == pose.source_epoch` and
   `_last_forwarded_status_frame >= pose.source_newest_frame_index`. Use
   `output_reference_topic` on the same output object that forwarded status.
   Clear the pending pose only after `send()` returns true; retain it on false
   for a later freshness/barrier recheck.

Preserve legacy button publication, merger/gap behavior, stale diagnostics,
and three-message default debounce. Store `_source_kind` so tests can prove the
branch is explicit. Construction creates resources in node -> owned input ->
owned output -> ROS publishers -> timer order; if a later step fails, unwind
only completed owned resources in reverse order before destroying the ROS
node. `destroy_node()` is idempotent: destroy the timer and publishers, close
owned output then owned input, and call `super().destroy_node()` once. Injected
transports remain open for their caller.

- [ ] **Step 4: Run Xsens and legacy bridge regressions**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_bridge_contract.py test/test_zerolab_pose_contract.py test/test_zerolab_lifecycle.py -v
```

Expected: PASS; optional metadata on a legacy message does not alter its path.

- [ ] **Step 5: Commit bridge support**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/pico/pose_to_smpl_ref_bridge.py src/bxi_example_py_elf3/test/test_xsens_bridge_contract.py
git commit -m "feat(sonic): bridge authoritative xsens windows"
```

### Task 12: Join Xsens Status and Canonical References Behind a Live Gate

**Files:**

- Modify: `src/bxi_example_py_elf3/mods/com.bxi.sonic/policy.py:97-104,258-359,372-635`
- Modify: `src/bxi_example_py_elf3/test/xsens_test_helpers.py`
- Create: `src/bxi_example_py_elf3/test/test_xsens_policy.py`

**Interfaces:**

- Consumes: Task 11 `smpl_ref` and `xsens_status` on ZMQ 5557.
- Produces: extended `SmplReferenceFrame`, canonical `XsensStatusSnapshot`, `JoinedSourceSnapshot`, `poll_source_snapshot()`, and a closed-by-default Xsens live gate.
- Compatibility: legacy PICO/ZeroLab references without metadata keep local receive-time freshness and automatic live/idle behavior.
- Constructor addition, after existing `backend` to preserve positional callers:

```python
def __init__(
    self,
    model_onnx_path: str,
    stream_reference_npz: str,
    use_smpl_ref_zmq: bool = True,
    smpl_ref_zmq_host: str = SMPL_REF_HOST,
    smpl_ref_zmq_port: int = SMPL_REF_PORT,
    smpl_ref_zmq_topic: str = SMPL_REF_TOPIC,
    yaw_bias_rad: float = math.pi / 2.0,
    live_ref_timeout_s: float = 0.5,
    idle_frame_start: int = DEFAULT_IDLE_FRAME_START,
    source_blend_duration_s: float = 0.4,
    runtime: InferenceRuntime | None = None,
    backend: str = "auto",
    *,
    xsens_status_zmq_topic: str = "xsens_status",
    monotonic: MonotonicSeconds = time.monotonic,
    monotonic_ns: MonotonicNanoseconds = time.monotonic_ns,
) -> None:
    ...  # Signature only; retain the existing constructor body and append fields.
```

Store these as `_monotonic`/`_monotonic_ns` and use them for every policy
freshness/blend decision. The startup resource always subscribes to exact
topics `smpl_ref_zmq_topic` and `xsens_status_zmq_topic`; runtime `source_kind`
controls selection, not socket construction.

- [ ] **Step 1: Write failing separate-queue, snapshot, freshness, and gate tests**

```python
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
    PROPRIOCEPTION_DIM,
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
        received = self._monotonic() if received_mono is None else received_mono
        message = pack_pose_message(fields, topic=self.xsens_status_zmq_topic)
        with self._message_lock:
            self._status_messages.append((message, float(received)))

    def inject_reference(
        self,
        reference: SmplReferenceFrame,
        received_mono: float | None = None,
    ) -> None:
        received = self._monotonic() if received_mono is None else received_mono
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
            fields["source_epoch"] = np.array([reference.source_epoch], np.int64)
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
        policy.inject_status(make_status(status_sequence=sequence, source_epoch=71))
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
    inject_exact_authorized_pair(policy, epoch=71, command_id=7, sequence=11, newest=110)
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
```

Add the exact boundary, malformed-input, reset, and legacy tests:

```python
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


def test_republication_does_not_replace_forwarded_producer_time(policy, fake_clock):
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
        source_blend_duration_s=0.25, source_kind="legacy", status_timeout_s=0.2,
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
```

- [ ] **Step 2: Run policy tests and verify joined snapshots are missing**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_policy.py -v
```

Expected: FAIL importing `XsensStatusSnapshot` or calling `poll_source_snapshot`.

- [ ] **Step 3: Implement typed decode, independent latest values, and gate validation**

Replace the mutable reference container and add the joined gate types exactly:

```python
from threading import Event, Lock, Thread
from typing import Callable, Mapping


MonotonicSeconds = Callable[[], float]
MonotonicNanoseconds = Callable[[], int]


@dataclass(frozen=True)
class SmplReferenceFrame:
    term1_local: np.ndarray
    root_quat: np.ndarray
    wrist: np.ndarray
    anchor_quat: np.ndarray | None = None
    frame_index: int = -1
    sequence: int = 0
    source_ready: bool = True
    producer_monotonic_ns: int | None = None
    source_epoch: int | None = None
    source_newest_frame_index: int | None = None
    received_monotonic: float = 0.0


@dataclass(frozen=True)
class XsensStatusSnapshot:
    status_sequence: int
    status_monotonic_ns: int
    source_epoch: int
    last_arm_command_id: int
    last_arm_target_epoch: int
    last_requested_arm_epoch: int
    accepted_arm_epoch: int
    producer_monotonic_ns: int
    newest_frame_index: int
    ready: bool
    reference_window_ready: bool
    source_stale: bool
    ready_frames: int
    recovery_frames: int
    reason_code: int
    received_monotonic: float


@dataclass(frozen=True)
class JoinedSourceSnapshot:
    status: XsensStatusSnapshot | None
    reference: SmplReferenceFrame | None


@dataclass(frozen=True)
class ArmGateProof:
    command_id: int
    target_source_epoch: int
    requested_arm_epoch: int
    pre_status_sequence: int
```

Use the complete constructor signature from the Interfaces block. Immediately
after the existing topic assignments, install the injected clocks and
closed-by-default runtime state:

```python
self.xsens_status_zmq_topic = str(xsens_status_zmq_topic)
self._monotonic = monotonic
self._monotonic_ns = monotonic_ns
self.source_kind = "legacy"
self.status_timeout_s = 0.2
self.live_reference_gate_open = False
self.armed_source_epoch: int | None = None
self.minimum_source_frame_index: int | None = None
self._gate_rearm_required = False
self._gate_reset_yaw_requested = False
self._gate_hold_requested = False
self._latest_status: XsensStatusSnapshot | None = None
self._latest_reference: SmplReferenceFrame | None = None
self._latest_authorized_reference: SmplReferenceFrame | None = None
```

Extend runtime configuration without breaking existing callers:

```python
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
    self.yaw_bias_rad = float(yaw_bias_rad)
    self.live_ref_timeout_s = float(live_ref_timeout_s)
    self.idle_frame_start = int(idle_frame_start)
    self.source_blend_duration_s = float(source_blend_duration_s)
    if source_kind not in {"legacy", "xsens"}:
        raise ValueError(f"unsupported SONIC source_kind: {source_kind}")
    if not math.isfinite(status_timeout_s) or status_timeout_s <= 0.0:
        raise ValueError("status_timeout_s must be positive and finite")
    self.source_kind = source_kind
    self.status_timeout_s = float(status_timeout_s)
    self._validate_runtime_config()
    if hasattr(self, "ref_term1"):
        self.idle_frame_start = int(np.clip(
            self.idle_frame_start, 0, self.ref_term1.shape[0] - WINDOW
        ))
```

Replace the single queue with two timestamped queues and retain the existing
thread startup, error propagation, shutdown, and `LINGER=0` lifecycle. The
receiver subscribes to both exact topics before connecting. Immediately after
each successful `recv` it captures the injected clock, classifies and appends
under the shared lock:

```python
def _init_zmq(self) -> None:
    self._message_lock = Lock()
    self._reference_messages: deque[tuple[bytes, float]] = deque(maxlen=64)
    self._status_messages: deque[tuple[bytes, float]] = deque(maxlen=64)
    self._zmq_stop = Event()
    self._zmq_ready = Event()
    self._zmq_error: BaseException | None = None
    self._zmq_thread: Thread | None = None
    if not self.use_smpl_ref_zmq:
        return
    self._zmq_thread = Thread(
        target=self._run_reference_receiver,
        name="sonic-reference",
        daemon=False,
    )
    self._zmq_thread.start()
    self._zmq_ready.wait()
    if self._zmq_error is not None:
        raise RuntimeError(
            f"cannot initialize SONIC reference receiver: {self._zmq_error}"
        ) from self._zmq_error


def _append_source_message(self, message: bytes, received: float) -> None:
    status_prefix = self.xsens_status_zmq_topic.encode("utf-8")
    reference_prefix = self.smpl_ref_zmq_topic.encode("utf-8")
    with self._message_lock:
        if message.startswith(status_prefix):
            self._status_messages.append((message, received))
        elif message.startswith(reference_prefix):
            self._reference_messages.append((message, received))


# In _run_reference_receiver(), retain the existing try/finally lifecycle and
# replace its one subscription and one-queue append with these exact lines.
socket.setsockopt_string(zmq.SUBSCRIBE, self.smpl_ref_zmq_topic)
socket.setsockopt_string(zmq.SUBSCRIBE, self.xsens_status_zmq_topic)
socket.connect(f"tcp://{self.smpl_ref_zmq_host}:{self.smpl_ref_zmq_port}")
while not self._zmq_stop.is_set():
    events = dict(poller.poll(timeout=50))
    if socket not in events:
        continue
    while True:
        try:
            message = socket.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            break
        received = self._monotonic()
        self._append_source_message(message, received)
```

Decode scalar fields with exact dtype and shape. Malformed messages are ignored
individually, so the last valid value on the other queue cannot be overwritten:

```python
def _scalar(
    fields: Mapping[str, np.ndarray],
    name: str,
    dtype: np.dtype | type,
):
    if name not in fields:
        raise KeyError(name)
    value = np.asarray(fields[name])
    expected = np.dtype(dtype)
    if value.shape != (1,) or value.dtype != expected:
        raise ValueError(
            f"{name} must have dtype {expected} and shape (1,), "
            f"got {value.dtype} {value.shape}"
        )
    return value[0].item()


def _status_from_fields(
    self,
    fields: Mapping[str, np.ndarray],
    received_monotonic: float,
) -> XsensStatusSnapshot:
    status = XsensStatusSnapshot(
        status_sequence=int(_scalar(fields, "status_sequence", np.int64)),
        status_monotonic_ns=int(
            _scalar(fields, "status_monotonic_ns", np.int64)
        ),
        source_epoch=int(_scalar(fields, "source_epoch", np.int64)),
        last_arm_command_id=int(
            _scalar(fields, "last_arm_command_id", np.int64)
        ),
        last_arm_target_epoch=int(
            _scalar(fields, "last_arm_target_epoch", np.int64)
        ),
        last_requested_arm_epoch=int(
            _scalar(fields, "last_requested_arm_epoch", np.int64)
        ),
        accepted_arm_epoch=int(
            _scalar(fields, "accepted_arm_epoch", np.int64)
        ),
        producer_monotonic_ns=int(
            _scalar(fields, "producer_monotonic_ns", np.int64)
        ),
        newest_frame_index=int(
            _scalar(fields, "newest_frame_index", np.int64)
        ),
        ready=bool(_scalar(fields, "ready", np.bool_)),
        reference_window_ready=bool(
            _scalar(fields, "reference_window_ready", np.bool_)
        ),
        source_stale=bool(_scalar(fields, "source_stale", np.bool_)),
        ready_frames=int(_scalar(fields, "ready_frames", np.int32)),
        recovery_frames=int(_scalar(fields, "recovery_frames", np.int32)),
        reason_code=int(_scalar(fields, "reason_code", np.int32)),
        received_monotonic=float(received_monotonic),
    )
    nonnegative = (
        status.status_sequence,
        status.status_monotonic_ns,
        status.source_epoch,
        status.last_arm_command_id,
        status.last_arm_target_epoch,
        status.last_requested_arm_epoch,
        status.accepted_arm_epoch,
        status.producer_monotonic_ns,
        status.ready_frames,
        status.recovery_frames,
    )
    if any(value < 0 for value in nonnegative):
        raise ValueError("status counters, epochs, IDs, and times are nonnegative")
    if status.newest_frame_index < -1:
        raise ValueError("newest_frame_index must be at least -1")
    if status.reason_code not in range(12):
        raise ValueError("reason_code is outside the XsensReason contract")
    if not math.isfinite(status.received_monotonic):
        raise ValueError("received_monotonic must be finite")
    return status


def _optional_scalar(
    fields: Mapping[str, np.ndarray],
    name: str,
    dtype: np.dtype | type,
) -> int | None:
    if name not in fields:
        return None
    return int(_scalar(fields, name, dtype))


def _frame_from_fields(
    self,
    fields: Mapping[str, np.ndarray],
    received_monotonic: float,
) -> SmplReferenceFrame:
    source_ready = (
        True
        if "source_ready" not in fields
        else bool(_scalar(fields, "source_ready", np.bool_))
    )
    producer_ns = _optional_scalar(
        fields, "producer_monotonic_ns", np.int64
    )
    source_epoch = _optional_scalar(fields, "source_epoch", np.int64)
    newest = _optional_scalar(
        fields, "source_newest_frame_index", np.int64
    )
    if producer_ns is not None and producer_ns < 0:
        raise ValueError("producer_monotonic_ns must be nonnegative")
    if source_epoch is not None and source_epoch < 0:
        raise ValueError("source_epoch must be nonnegative")
    if newest is not None and newest < -1:
        raise ValueError("source_newest_frame_index must be at least -1")
    if not math.isfinite(received_monotonic):
        raise ValueError("received_monotonic must be finite")

    stream_mode = fields.get("source_stream_mode")
    if stream_mode is not None and int(np.asarray(stream_mode).reshape(-1)[-1]) != 1:
        raise ValueError("smpl_ref source is not in POSE mode")
    calibration_ready = fields.get("source_calibration_ready")
    if calibration_ready is not None and not bool(
        np.asarray(calibration_ready).reshape(-1)[-1]
    ):
        raise ValueError("smpl_ref source is not calibrated")

    frame_indices = fields.get("frame_index")
    frame_index = (
        -1
        if frame_indices is None or np.asarray(frame_indices).size == 0
        else int(np.asarray(frame_indices).reshape(-1)[-1])
    )
    anchor = fields.get("anchor_quat")
    frame = SmplReferenceFrame(
        term1_local=_as_window(fields["term1_local"], 72, "term1_local"),
        root_quat=_as_window(fields["root_quat"], 4, "root_quat"),
        wrist=_as_window(fields["wrist"], 6, "wrist"),
        anchor_quat=(
            _as_window(anchor, 4, "anchor_quat")
            if anchor is not None
            else None
        ),
        frame_index=frame_index,
        sequence=self.live_sequence + 1,
        source_ready=source_ready,
        producer_monotonic_ns=producer_ns,
        source_epoch=source_epoch,
        source_newest_frame_index=newest,
        received_monotonic=float(received_monotonic),
    )
    arrays = [frame.term1_local, frame.root_quat, frame.wrist]
    if frame.anchor_quat is not None:
        arrays.append(frame.anchor_quat)
    if not all(np.isfinite(array).all() for array in arrays):
        raise ValueError("smpl_ref contains non-finite values")
    if np.any(np.linalg.norm(frame.root_quat, axis=1) <= 1.0e-6):
        raise ValueError("smpl_ref contains an invalid root quaternion")
    if frame.anchor_quat is not None and np.any(
        np.linalg.norm(frame.anchor_quat, axis=1) <= 1.0e-6
    ):
        raise ValueError("smpl_ref contains an invalid anchor quaternion")
    self.live_sequence += 1
    return frame


_DECODE_ERRORS = (
    IndexError,
    KeyError,
    TypeError,
    UnicodeDecodeError,
    ValueError,
    json.JSONDecodeError,
)


def poll_source_snapshot(self) -> JoinedSourceSnapshot:
    if not self.use_smpl_ref_zmq:
        return JoinedSourceSnapshot(self._latest_status, self._latest_reference)
    with self._message_lock:
        status_messages = tuple(self._status_messages)
        reference_messages = tuple(self._reference_messages)
        self._status_messages.clear()
        self._reference_messages.clear()

        for message, received in status_messages:
            try:
                fields = _decode_packed_message(
                    message, self.xsens_status_zmq_topic
                )
                if fields:
                    self._latest_status = self._status_from_fields(
                        fields, received
                    )
            except _DECODE_ERRORS:
                continue

        for message, received in reference_messages:
            try:
                fields = _decode_packed_message(
                    message, self.smpl_ref_zmq_topic
                )
                if fields:
                    self._latest_reference = self._frame_from_fields(
                        fields, received
                    )
            except _DECODE_ERRORS:
                continue

        self.latest_live_ref = self._latest_reference
        self.latest_live_ref_time = (
            0.0
            if self._latest_reference is None
            else self._latest_reference.received_monotonic
        )
        return JoinedSourceSnapshot(
            status=self._latest_status,
            reference=self._latest_reference,
        )


def poll_reference(self) -> SmplReferenceFrame | None:
    return self.poll_source_snapshot().reference


def _drain_source_queues(self) -> None:
    with self._message_lock:
        self._status_messages.clear()
        self._reference_messages.clear()
        self._latest_status = None
        self._latest_reference = None
```

Replace `has_fresh_live_reference` with the injected-clock legacy check. Xsens
callers use the explicit gate:

```python
def has_fresh_live_reference(self, timeout_s: float | None = None) -> bool:
    reference = self.poll_reference()
    timeout = self.live_ref_timeout_s if timeout_s is None else float(timeout_s)
    age = (
        math.inf
        if reference is None
        else self._monotonic() - reference.received_monotonic
    )
    return bool(reference is not None and 0.0 <= age <= timeout)
```

Make reset additive: retain every existing history/cursor/status operation,
capture measured joints only once, clear both queues and caches, and preserve
all configured runtime values:

```python
def reset(
    self,
    frame: InferenceFrame | None = None,
    *,
    seed_target_from_robot: bool = False,
) -> None:
    measured = (
        self.bind_joints(frame).position.copy()
        if frame is not None
        else None
    )
    if seed_target_from_robot and measured is None:
        raise ValueError("seed_target_from_robot requires an inference frame")

    self.last_action.fill(0.0)
    self.base_ang_vel_history.fill(0.0)
    self.joint_pos_history.fill(0.0)
    self.joint_vel_history.fill(0.0)
    self.action_history.fill(0.0)
    self.gravity_history.fill(0.0)
    self.motion_cursor = self.idle_frame_start
    self.yaw_aligned = False
    self.yaw_offset = 0.0
    self.latest_live_ref = None
    self.latest_live_ref_time = 0.0
    self.live_sequence = 0
    self.reference_source = None
    self.source_blend_from = self.default_dof_pos.copy()
    self.source_blend_started_at = 0.0
    self.source_blend_active = False
    self.source_transition_from = None
    self.policy_active = False
    self.last_status = "reset"
    self._reported_status = None
    self.live_reference_gate_open = False
    self.armed_source_epoch = None
    self.minimum_source_frame_index = None
    self._gate_rearm_required = False
    self._gate_reset_yaw_requested = False
    self._gate_hold_requested = False
    self.pending_yaw_offset = None
    self._latest_authorized_reference = None
    self._drain_source_queues()

    target = (
        measured
        if seed_target_from_robot
        else self.default_dof_pos
    )
    np.copyto(self.target_dof_pos, target)
    self.publish_output(self.target_dof_pos, self.kps, self.kds)
```

Implement the reusable transport predicate, gate, and Task-12 selection path:

```python
def _xsens_transport_valid(
    self,
    snapshot: JoinedSourceSnapshot,
    *,
    source_epoch: int,
    minimum_source_frame_index: int,
    now_mono: float,
    now_ns: int,
) -> bool:
    status, reference = snapshot.status, snapshot.reference
    if (
        status is None
        or reference is None
        or reference.producer_monotonic_ns is None
        or reference.source_epoch is None
        or reference.source_newest_frame_index is None
    ):
        return False
    status_local_age = now_mono - status.received_monotonic
    reference_local_age = now_mono - reference.received_monotonic
    status_producer_age = now_ns - status.producer_monotonic_ns
    reference_producer_age = now_ns - reference.producer_monotonic_ns
    maximum_producer_age = int(
        self.live_ref_timeout_s * 1_000_000_000
    )
    return bool(
        source_epoch > 0
        and 0.0 <= status_local_age <= self.status_timeout_s
        and reference_local_age >= 0.0
        and 0 <= status_producer_age <= maximum_producer_age
        and 0 <= reference_producer_age <= maximum_producer_age
        and not status.source_stale
        and status.source_epoch == source_epoch
        and status.accepted_arm_epoch == source_epoch
        and status.ready
        and status.reference_window_ready
        and status.newest_frame_index >= minimum_source_frame_index
        and reference.source_ready
        and reference.source_epoch == source_epoch
        and reference.source_newest_frame_index >= minimum_source_frame_index
        and status.newest_frame_index
        >= reference.source_newest_frame_index
    )


def open_live_reference_gate(
    self,
    *,
    source_epoch: int,
    minimum_source_frame_index: int,
    reset_yaw: bool,
    arm_proof: ArmGateProof | None,
) -> bool:
    snapshot = self.poll_source_snapshot()
    now_mono = self._monotonic()
    now_ns = self._monotonic_ns()
    status, reference = snapshot.status, snapshot.reference
    if status is None or reference is None:
        return False
    if not self._xsens_transport_valid(
        snapshot,
        source_epoch=source_epoch,
        minimum_source_frame_index=minimum_source_frame_index,
        now_mono=now_mono,
        now_ns=now_ns,
    ):
        return False
    proof_required = (
        self.armed_source_epoch != source_epoch or self._gate_rearm_required
    )
    if proof_required:
        if arm_proof is None:
            return False
        valid_receipt = (
            status.status_sequence > arm_proof.pre_status_sequence
            and status.last_arm_command_id == arm_proof.command_id
            and status.last_arm_target_epoch == arm_proof.target_source_epoch
            and status.last_requested_arm_epoch == arm_proof.requested_arm_epoch
            and status.accepted_arm_epoch == arm_proof.requested_arm_epoch
            and arm_proof.target_source_epoch == source_epoch
            and arm_proof.requested_arm_epoch == source_epoch
        )
        if not valid_receipt:
            return False
    self.live_reference_gate_open = True
    self.armed_source_epoch = source_epoch
    self.minimum_source_frame_index = minimum_source_frame_index
    self._gate_rearm_required = False
    self._gate_reset_yaw_requested = bool(reset_yaw)
    self._latest_authorized_reference = reference
    return True

def close_live_reference_gate(
    self,
    *,
    hold_last_reference: bool,
    rearm_required: bool,
    preserve_pending_yaw: bool = False,
) -> None:
    self.live_reference_gate_open = False
    self._gate_hold_requested = bool(hold_last_reference)
    self._gate_rearm_required = bool(rearm_required)
    if not preserve_pending_yaw:
        self.pending_yaw_offset = None


def _active_reference(self) -> tuple[SmplReferenceFrame, str, float]:
    if self.source_kind == "legacy":
        live = self.poll_reference()
        now_mono = self._monotonic()
        live_age = (
            math.inf
            if live is None
            else now_mono - live.received_monotonic
        )
        if live is not None and 0.0 <= live_age <= self.live_ref_timeout_s:
            return live, "live", now_mono
        if live is not None:
            self.latest_live_ref = None
            self.latest_live_ref_time = 0.0
        return self._offline_frame(), "idle", now_mono

    snapshot = self.poll_source_snapshot()
    now_mono = self._monotonic()
    now_ns = self._monotonic_ns()
    valid = bool(
        self.live_reference_gate_open
        and self.armed_source_epoch is not None
        and self.minimum_source_frame_index is not None
        and self._xsens_transport_valid(
            snapshot,
            source_epoch=self.armed_source_epoch,
            minimum_source_frame_index=self.minimum_source_frame_index,
            now_mono=now_mono,
            now_ns=now_ns,
        )
    )
    if valid:
        assert snapshot.reference is not None
        self._latest_authorized_reference = snapshot.reference
        return snapshot.reference, "live", now_mono
    if self.live_reference_gate_open:
        changed_epoch = bool(
            snapshot.status is not None
            and self.armed_source_epoch is not None
            and snapshot.status.source_epoch != self.armed_source_epoch
        )
        self.close_live_reference_gate(
            hold_last_reference=True,
            rearm_required=changed_epoch,
            preserve_pending_yaw=not changed_epoch,
        )
    return self._offline_frame(), "idle", now_mono
```

Before returning true, `open_live_reference_gate` must see a local status age `<=status_timeout_s`, producer age `<=live_ref_timeout_s`, non-stale status, matching nonzero source and accepted-arm epochs, ready/complete status, a matching canonical reference, and both status/reference frame indices at or above the floor. Initial arm and changed-epoch re-arm additionally require `ArmGateProof` and compare exact command ID, target, request, and `status_sequence > pre_status_sequence`; a same-epoch recovery may pass `arm_proof=None` only when the policy already retains that armed epoch and no re-arm barrier. Store the epoch/index gate floor and repeat transport/epoch/authorization checks before every inference. Legacy source mode bypasses this manual gate and keeps its previous fallback semantics.

Task 13 replaces only the Xsens fallback at the end of `_active_reference`
with its held-reference selection. The predicate itself and the legacy branch
remain unchanged. No method after construction calls `time.monotonic()` or
`time.monotonic_ns()` directly.

- [ ] **Step 4: Run focused policy and legacy pose regressions**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_policy.py test/test_zerolab_pose_contract.py -v
```

Expected: PASS; cached or mismatched authorization never selects live.

- [ ] **Step 5: Commit joined snapshots and gating**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/policy.py src/bxi_example_py_elf3/test/xsens_test_helpers.py src/bxi_example_py_elf3/test/test_xsens_policy.py
git commit -m "feat(sonic): gate joined xsens references"
```

### Task 13: Add Static Human Hold, Phase-Owned Yaw, and Preemptible 0.4-Second Blends

**Files:**

- Modify: `src/bxi_example_py_elf3/mods/com.bxi.sonic/policy.py:491-804`
- Modify: `src/bxi_example_py_elf3/test/xsens_test_helpers.py`
- Modify: `src/bxi_example_py_elf3/test/test_xsens_policy.py`

**Interfaces:**

- Consumes: Task 12 live gate and joined snapshot.
- Produces: static held-reference selection; `active_yaw_offset`, `hold_yaw_offset`, `pending_yaw_offset`; `reset_xsens_alignment(context)`; and read-only properties `held_reference`, `selected_reference`, `source_blend_active`, `source_blend_started_at`, and `source_blend_from` (the array property returns a copy).
- Hold invariant: the old hold remains available until a complete recovery/re-arm blend succeeds or the full policy/state exits.

- [ ] **Step 1: Add failing hold, proprioception, yaw, blend, and manual-reset tests**

```python
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
    np.testing.assert_array_equal(policy.selected_reference.term1_local, policy.held_reference.term1_local)


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
```

Add the following helpers and tests. Recovery-count authority is intentionally
left for Task 14; policy tests only exercise an explicitly opened/closed gate.

```python
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
    inject_healthy_pair(policy, reference, sequence=12)
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


@pytest.mark.parametrize("with_anchor", [False, True])
def test_hold_tiles_wrist_and_optional_anchor(policy, with_anchor):
    reference = make_reference(row_marker=np.arange(10), with_anchor=with_anchor)
    authorize_reference(policy, reference)
    policy.close_live_reference_gate(hold_last_reference=True, rearm_required=False)
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
            held.anchor_quat, np.repeat(reference.anchor_quat[-1:], 10, axis=0)
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
```

- [ ] **Step 2: Run hold/yaw tests and verify stale still falls back to idle**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_policy.py -k 'hold or yaw or blend or alignment' -v
```

Expected: FAIL because `_active_reference()` currently discards stale live data and selects packaged idle.

- [ ] **Step 3: Implement static-reference ownership and transition preemption**

```python
def _static_last_row(frame: SmplReferenceFrame) -> SmplReferenceFrame:
    def tile(values: np.ndarray) -> np.ndarray:
        return np.ascontiguousarray(np.repeat(values[-1:], 10, axis=0), dtype=np.float32)
    return SmplReferenceFrame(
        term1_local=tile(frame.term1_local),
        root_quat=tile(frame.root_quat),
        wrist=tile(frame.wrist),
        anchor_quat=tile(frame.anchor_quat) if frame.anchor_quat is not None else None,
        frame_index=frame.frame_index,
        sequence=frame.sequence,
        source_ready=frame.source_ready,
        producer_monotonic_ns=frame.producer_monotonic_ns,
        source_epoch=frame.source_epoch,
        source_newest_frame_index=frame.source_newest_frame_index,
        received_monotonic=frame.received_monotonic,
    )
```

Add exact blend kinds and initialize the new fields in the constructor after
the existing legacy yaw/source fields:

```python
from typing import Literal


SourceBlendKind = Literal[
    "idle_to_live",
    "live_to_hold",
    "hold_to_live",
    "hold_to_rearm",
    "source_change",
    "alignment",
]
AlignmentContext = Literal["fresh", "same_epoch_hold", "rearm_hold"]


self.active_yaw_offset = 0.0
self.hold_yaw_offset = 0.0
self._pending_yaw_offset: float | None = None
self._pending_yaw_epoch: int | None = None
self._held_reference: SmplReferenceFrame | None = None
self._selected_reference: SmplReferenceFrame | None = None
self._selected_yaw_offset = 0.0
self._last_robot_anchor = np.array(
    [1.0, 0.0, 0.0, 0.0], dtype=np.float64
)
self._source_blend_from = self.default_dof_pos.copy()
self._source_blend_started_at = 0.0
self._source_blend_active = False
self._source_blend_kind: SourceBlendKind | None = None
```

Expose read-only state. The array property returns a defensive copy:

```python
@property
def held_reference(self) -> SmplReferenceFrame | None:
    return self._held_reference


@property
def selected_reference(self) -> SmplReferenceFrame | None:
    return self._selected_reference


@property
def pending_yaw_offset(self) -> float | None:
    return self._pending_yaw_offset


@property
def source_blend_active(self) -> bool:
    return self._source_blend_active


@property
def source_blend_started_at(self) -> float:
    return self._source_blend_started_at


@property
def source_blend_from(self) -> np.ndarray:
    return self._source_blend_from.copy()
```

Update the legacy transition helper only for the private blend-field rename;
its existing live/idle edge and yaw-reset behavior stays intact:

```python
def _begin_source_transition(self, source: str, now_mono: float) -> None:
    if source == self.reference_source:
        return
    self.source_transition_from = self.reference_source
    self.reference_source = source
    self.reset_yaw_alignment()
    self._source_blend_from = self.target_dof_pos.copy()
    self._source_blend_started_at = now_mono
    self._source_blend_kind = "source_change"
    self._source_blend_active = self.source_blend_duration_s > 0.0
```

Capture a static hold once. An interrupted initial-arm blend has no older
active live yaw, so its pending yaw becomes the hold yaw before pending
retention/clearing is decided:

```python
def _capture_hold_if_needed(self) -> None:
    if self._held_reference is not None:
        return
    reference = self._latest_authorized_reference
    if reference is None:
        return
    self._held_reference = _static_last_row(reference)
    if (
        self._pending_yaw_offset is not None
        and self._pending_yaw_epoch == self.armed_source_epoch
    ):
        self.hold_yaw_offset = self._pending_yaw_offset
    else:
        self.hold_yaw_offset = self.active_yaw_offset


def close_live_reference_gate(
    self,
    *,
    hold_last_reference: bool,
    rearm_required: bool,
    preserve_pending_yaw: bool = False,
) -> None:
    if hold_last_reference:
        self._capture_hold_if_needed()
    self.live_reference_gate_open = False
    self._gate_hold_requested = bool(hold_last_reference)
    self._gate_rearm_required = bool(rearm_required)
    pending_matches_armed = bool(
        self._pending_yaw_offset is not None
        and self._pending_yaw_epoch == self.armed_source_epoch
    )
    keep_pending = pending_matches_armed and (
        preserve_pending_yaw or not rearm_required
    )
    if not keep_pending:
        self._pending_yaw_offset = None
        self._pending_yaw_epoch = None
```

Replace only the Xsens selection fallback from Task 12. The legacy branch is
repeated here so the implementer does not have to infer what remains:

```python
def _active_reference(self) -> tuple[SmplReferenceFrame, str, float]:
    if self.source_kind == "legacy":
        live = self.poll_reference()
        now_mono = self._monotonic()
        live_age = (
            math.inf
            if live is None
            else now_mono - live.received_monotonic
        )
        if live is not None and 0.0 <= live_age <= self.live_ref_timeout_s:
            return live, "live", now_mono
        if live is not None:
            self.latest_live_ref = None
            self.latest_live_ref_time = 0.0
        return self._offline_frame(), "idle", now_mono

    snapshot = self.poll_source_snapshot()
    now_mono = self._monotonic()
    now_ns = self._monotonic_ns()
    valid_live = bool(
        self.live_reference_gate_open
        and self.armed_source_epoch is not None
        and self.minimum_source_frame_index is not None
        and self._xsens_transport_valid(
            snapshot,
            source_epoch=self.armed_source_epoch,
            minimum_source_frame_index=self.minimum_source_frame_index,
            now_mono=now_mono,
            now_ns=now_ns,
        )
    )
    if valid_live:
        assert snapshot.reference is not None
        self._latest_authorized_reference = snapshot.reference
        return snapshot.reference, "live", now_mono

    if self.live_reference_gate_open:
        changed_epoch = bool(
            snapshot.status is not None
            and self.armed_source_epoch is not None
            and snapshot.status.source_epoch != self.armed_source_epoch
        )
        self.close_live_reference_gate(
            hold_last_reference=True,
            rearm_required=changed_epoch,
            preserve_pending_yaw=not changed_epoch,
        )
    if self._gate_hold_requested and self._held_reference is not None:
        return self._held_reference, "hold", now_mono
    return self._offline_frame(), "idle", now_mono
```

Compute yaw from the newest selected human row and the current measured robot
anchor. Selection edges, not reference message sequence numbers, start blends:

```python
def _yaw_offset_for(
    self,
    reference: SmplReferenceFrame,
    robot_anchor: np.ndarray,
) -> float:
    if reference.anchor_quat is not None:
        reference_yaw = _yaw_from_quat_wxyz(reference.anchor_quat[-1])
        bias = 0.0
    else:
        reference_yaw = _yaw_from_quat_wxyz(reference.root_quat[-1])
        bias = self.yaw_bias_rad
    return (
        reference_yaw
        - _yaw_from_quat_wxyz(robot_anchor)
        + bias
    )


def _compute_selected_yaw_offset(self) -> float:
    if self._selected_reference is None:
        raise RuntimeError("no selected Xsens reference")
    return self._yaw_offset_for(
        self._selected_reference, self._last_robot_anchor
    )


def _begin_xsens_blend(
    self,
    kind: SourceBlendKind,
    now_mono: float,
) -> None:
    self._source_blend_from = self.target_dof_pos.copy()
    self._source_blend_started_at = float(now_mono)
    self._source_blend_kind = kind
    self._source_blend_active = self.source_blend_duration_s > 0.0
    if not self._source_blend_active:
        self._complete_xsens_blend()


def _complete_xsens_blend(self) -> None:
    completed_kind = self._source_blend_kind
    if completed_kind in {"idle_to_live", "hold_to_rearm"}:
        if (
            self._pending_yaw_offset is not None
            and self._pending_yaw_epoch == self.armed_source_epoch
        ):
            self.active_yaw_offset = self._pending_yaw_offset
            self._pending_yaw_offset = None
            self._pending_yaw_epoch = None
    if completed_kind in {"hold_to_live", "hold_to_rearm"}:
        self._held_reference = None
        self._gate_hold_requested = False
    self._source_blend_active = False
    self._source_blend_kind = None
    self.source_transition_from = None


def _prepare_xsens_selection(
    self,
    reference: SmplReferenceFrame,
    source: str,
    robot_anchor: np.ndarray,
    now_mono: float,
) -> float:
    previous = self.reference_source
    if (
        source == "live"
        and self._gate_reset_yaw_requested
    ):
        self._pending_yaw_offset = self._yaw_offset_for(
            reference, robot_anchor
        )
        self._pending_yaw_epoch = self.armed_source_epoch
        self._gate_reset_yaw_requested = False

    if source == "live":
        selected_yaw = (
            self._pending_yaw_offset
            if self._pending_yaw_offset is not None
            and self._pending_yaw_epoch == self.armed_source_epoch
            else self.active_yaw_offset
        )
    elif source == "hold":
        selected_yaw = self.hold_yaw_offset
    else:
        self._capture_yaw_if_needed(reference, robot_anchor)
        selected_yaw = self.yaw_offset

    if source != previous:
        if source == "live" and previous in {None, "idle"}:
            kind: SourceBlendKind = "idle_to_live"
        elif source == "hold" and previous == "live":
            kind = "live_to_hold"
        elif source == "live" and previous == "hold":
            kind = (
                "hold_to_rearm"
                if self._pending_yaw_offset is not None
                else "hold_to_live"
            )
        else:
            kind = "source_change"
        self.source_transition_from = previous
        self.reference_source = source
        self._begin_xsens_blend(kind, now_mono)

    self._selected_reference = reference
    self._selected_yaw_offset = float(selected_yaw)
    return self._selected_yaw_offset


def _blend_source_target(
    self,
    candidate: np.ndarray,
    now_mono: float,
) -> np.ndarray:
    if not self._source_blend_active:
        return np.asarray(candidate, dtype=np.float32)
    progress = float(np.clip(
        (now_mono - self._source_blend_started_at)
        / self.source_blend_duration_s,
        0.0,
        1.0,
    ))
    alpha = progress * progress * (3.0 - 2.0 * progress)
    blended = (
        (1.0 - alpha) * self._source_blend_from
        + alpha * candidate
    )
    if progress >= 1.0:
        self._complete_xsens_blend()
    return np.asarray(blended, dtype=np.float32)
```

Refactor model-input construction so history is still updated from current
robot proprioception on every hold tick, while Xsens and legacy yaw ownership
remain separate:

```python
def _build_model_input(
    self,
    frame: SmplReferenceFrame,
    source: str,
    now_mono: float,
    q: np.ndarray,
    dq: np.ndarray,
    quat_wxyz: np.ndarray,
    omega: np.ndarray,
) -> np.ndarray:
    robot_anchor = self._update_history(q, dq, quat_wxyz, omega)
    self._last_robot_anchor = robot_anchor.copy()
    if self.source_kind == "xsens":
        selected_yaw = self._prepare_xsens_selection(
            frame, source, robot_anchor, now_mono
        )
    else:
        self._capture_yaw_if_needed(frame, robot_anchor)
        selected_yaw = self.yaw_offset
        self._selected_reference = frame
        self._selected_yaw_offset = selected_yaw

    anchor_aligned = _quat_mul_wxyz(
        _axis_angle_quat_wxyz("z", selected_yaw),
        robot_anchor,
    )
    model_input = np.zeros(MODEL_INPUT_DIM, dtype=np.float32)
    self._write_smpl_tokenizer(frame, anchor_aligned, model_input)
    proprio = np.concatenate(
        [
            self.base_ang_vel_history.reshape(-1),
            self.joint_pos_history.reshape(-1),
            self.joint_vel_history.reshape(-1),
            self.action_history.reshape(-1),
            self.gravity_history.reshape(-1),
        ]
    ).astype(np.float32)
    model_input[SMPL_TOKENIZER_DIM:] = proprio
    return model_input.reshape(1, -1)
```

In `inference_step` retain the existing dtype/shape validation, backend call,
action clipping, target publication, status logging, and return value. Replace
only reference/transition/input construction with these exact calls:

```python
frame, source, now_mono = self._active_reference()
if self.source_kind == "legacy":
    self._begin_source_transition(source, now_mono)
model_input = self._build_model_input(
    frame, source, now_mono, q, dq, quat_wxyz, omega
)
np.copyto(self.input_buffer, model_input)
raw_action = np.asarray(
    self._backend.run(self._inputs)["action"]
).reshape(-1)
if raw_action.size != NUM_JOINTS:
    raise ValueError(
        f"SONIC output has {raw_action.size} values; expected {NUM_JOINTS}"
    )
action = np.clip(raw_action, -ACTION_CLIP, ACTION_CLIP).astype(np.float32)
candidate = self.default_dof_pos + action * self.action_scale
np.copyto(
    self.target_dof_pos,
    self._blend_source_target(candidate, now_mono),
)
```

Manual alignment validates both the requested ownership context and stable
selection, mutates only the owned yaw values, and begins exactly one blend:

```python
def reset_xsens_alignment(self, context: AlignmentContext) -> bool:
    if (
        self.source_kind != "xsens"
        or self._source_blend_active
        or self._pending_yaw_offset is not None
        or self._selected_reference is None
    ):
        return False
    valid_context = {
        "fresh": (
            self.reference_source == "live"
            and self.live_reference_gate_open
        ),
        "same_epoch_hold": (
            self.reference_source == "hold"
            and not self._gate_rearm_required
        ),
        "rearm_hold": (
            self.reference_source == "hold"
            and self._gate_rearm_required
        ),
    }
    if context not in valid_context or not valid_context[context]:
        return False

    replacement = self._compute_selected_yaw_offset()
    if context == "fresh":
        self.active_yaw_offset = replacement
    elif context == "same_epoch_hold":
        self.active_yaw_offset = replacement
        self.hold_yaw_offset = replacement
    else:
        self.hold_yaw_offset = replacement
    self._selected_yaw_offset = replacement
    self._begin_xsens_blend("alignment", self._monotonic())
    return True
```

No raw Xsens pose, quaternion, or action sample is filtered; only the 29-joint
command target is smoothstep-blended.

Change reset to support Xsens entry seeding without changing legacy callers:

```python
def reset(
    self,
    frame: InferenceFrame | None = None,
    *,
    seed_target_from_robot: bool = False,
) -> None:
    measured = (
        self.bind_joints(frame).position.copy()
        if frame is not None else None
    )
    if seed_target_from_robot and measured is None:
        raise ValueError("seed_target_from_robot requires an inference frame")

    self.last_action.fill(0.0)
    self.base_ang_vel_history.fill(0.0)
    self.joint_pos_history.fill(0.0)
    self.joint_vel_history.fill(0.0)
    self.action_history.fill(0.0)
    self.gravity_history.fill(0.0)
    self.motion_cursor = self.idle_frame_start
    self.yaw_aligned = False
    self.yaw_offset = 0.0
    self.active_yaw_offset = 0.0
    self.hold_yaw_offset = 0.0
    self._pending_yaw_offset = None
    self._pending_yaw_epoch = None
    self.latest_live_ref = None
    self.latest_live_ref_time = 0.0
    self.live_sequence = 0
    self.reference_source = None
    self._held_reference = None
    self._selected_reference = None
    self._selected_yaw_offset = 0.0
    self._last_robot_anchor = np.array(
        [1.0, 0.0, 0.0, 0.0], dtype=np.float64
    )
    self._latest_authorized_reference = None
    self.live_reference_gate_open = False
    self.armed_source_epoch = None
    self.minimum_source_frame_index = None
    self._gate_rearm_required = False
    self._gate_reset_yaw_requested = False
    self._gate_hold_requested = False
    target = measured if seed_target_from_robot else self.default_dof_pos
    self._source_blend_from = np.asarray(target, np.float32).copy()
    self._source_blend_started_at = 0.0
    self._source_blend_active = False
    self._source_blend_kind = None
    self.source_transition_from = None
    self.policy_active = False
    self.last_status = "reset"
    self._reported_status = None
    self._drain_source_queues()
    np.copyto(self.target_dof_pos, target)
    self.publish_output(self.target_dof_pos, self.kps, self.kds)
```

When true, initialize the current 29-joint target from the bound measured robot frame before the first idle inference; existing PICO/ZeroLab calls use the false default.

- [ ] **Step 4: Run the complete policy suite**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_policy.py test/test_zerolab_pose_contract.py -v
```

Expected: PASS for preemption, yaw ownership, manual alignment, and legacy behavior.

- [ ] **Step 5: Commit hold and smoothing semantics**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/policy.py src/bxi_example_py_elf3/test/xsens_test_helpers.py src/bxi_example_py_elf3/test/test_xsens_policy.py
git commit -m "feat(sonic): hold xsens pose across stream gaps"
```

### Task 14: Implement the Single-State Xsens Phase Machine and Correlated Commands

**Files:**

- Modify: `src/bxi_example_py_elf3/mods/com.bxi.sonic/state.py`
- Modify: `src/bxi_example_py_elf3/test/xsens_test_helpers.py`
- Create: `src/bxi_example_py_elf3/test/test_xsens_state.py`

**Interfaces:**

- Consumes: Tasks 1, 9, 12, and 13 remote snapshot, arm/status contract, joined snapshot, and policy gate/hold APIs.
- Produces: `XsensPhase`, `XsensLinkStatus`, `XsensHoldCause`, `PendingArmRequest`, and `XsensSonicTeleopState`.
- Manual event: `activate_xsens`; alignment event remains `reset_alignment` with Xsens phase restrictions.
- Update the `SonicPolicy` protocol in this file with the exact Task 12/13 signatures: extended `configure_runtime`, `reset(..., seed_target_from_robot=False)`, `poll_source_snapshot`, `open_live_reference_gate(..., arm_proof) -> bool`, `close_live_reference_gate`, `reset_xsens_alignment(...) -> bool`, and read-only `source_blend_active`, `pending_yaw_offset`, and `armed_source_epoch`.

The protocol declarations are literal, not comments:

```python
def reset(
    self, frame: InferenceFrame | None = None, *,
    seed_target_from_robot: bool = False,
) -> None: ...
def poll_source_snapshot(self) -> JoinedSourceSnapshot: ...
def open_live_reference_gate(
    self, *, source_epoch: int, minimum_source_frame_index: int,
    reset_yaw: bool, arm_proof: ArmGateProof | None,
) -> bool: ...
def close_live_reference_gate(
    self, *, hold_last_reference: bool, rearm_required: bool,
    preserve_pending_yaw: bool = False,
) -> None: ...
def reset_xsens_alignment(
    self, context: Literal["fresh", "same_epoch_hold", "rearm_hold"]
) -> bool: ...
@property
def source_blend_active(self) -> bool: ...
@property
def pending_yaw_offset(self) -> float | None: ...
@property
def armed_source_epoch(self) -> int | None: ...
```

```text
XsensSonicTeleopState(
  name: str, state_id: int, policy: ResourceHandle[SonicPolicy], *,
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
  monotonic: Callable[[], float] = time.monotonic
)
```

The compatibility-shaped flags are validated to the approved values rather
than creating alternate modes: `require_live_reference=False`,
`manual_live_enable=True`, `manual_enable_neutral_value=0`,
`seed_entry_from_robot=True`, `hold_last_live_reference=True`,
`auto_resume_same_epoch=True`, `rearm_on_source_epoch_change=True`, and
`hardware_gripper=False`. Any other value raises `ValueError` at construction.

- [ ] **Step 1: Write failing phase, latch, command-join, recovery, and exit tests**

```python
# test/test_xsens_state.py
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
    FakeClock,
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
        self.messages.append(clone)


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
```

```python
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
    state_harness.publish_status(sequence=23, epoch=71, ready=False)
    assert state_harness.state.on_action(state_harness.ctx, "activate_xsens") is True
    assert state_harness.command_count() == baseline_count
    assert not state_harness.state.neutral_latch_open

    state_harness.make_ready(epoch=71, status_sequence=24, newest_frame=111)
    state_harness.release_and_press()
    assert state_harness.state.arm_pending
    state_harness.publish_status(sequence=25, epoch=71, ready=False)
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
```

```python
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
```

- [ ] **Step 2: Run state tests and verify the Xsens state class is missing**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_state.py -v
```

Expected: fixture setup FAIL resolving `XsensSonicTeleopState` from the
dynamically loaded state module.

- [ ] **Step 3: Implement explicit enums, pending request correlation, and QoS**

```python
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
```

Subclass `SonicTeleopState` so PICO/ZeroLab code paths do not gain Xsens
branches. Define these public observations as getter-only properties; tests and
diagnostics must not mutate them:

```python
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
        self._pending_request is not None
        and self._pending_request.requested_arm_epoch == 0
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
```

Refactor the existing state preparation once, preserving all existing
logger/gripper/reset behavior. The Xsens subclass calls this shared helper
directly and never calls the legacy `SonicTeleopState.on_prepare()` first:

```python
# SonicTeleopState
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

def on_prepare(self, ctx, from_state) -> None:
    self._prepare_policy(ctx)


# XsensSonicTeleopState
def on_prepare(self, ctx, from_state) -> None:
    self._prepare_policy(
        ctx,
        source_kind="xsens",
        status_timeout_s=self.status_timeout_s,
        seed_target_from_robot=True,
    )
    self._reset_phase_session()
```

Create/destroy the command publisher with literal QoS and keep issued IDs for
the state-object lifetime:

```python
def on_bind(self, ctx) -> None:
    super().on_bind(ctx)
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    self._arm_command_publisher = ctx.ros_node.create_publisher(
        Int64MultiArray, self.arm_command_topic, qos
    )

def on_unbind(self, ctx) -> None:
    if self._arm_command_publisher is not None:
        ctx.ros_node.destroy_publisher(self._arm_command_publisher)
        self._arm_command_publisher = None
    super().on_unbind(ctx)

def _draw_command_id(self) -> int:
    while True:
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

def _publish_command(
    self, *, target_source_epoch: int, requested_arm_epoch: int,
    pre_status_sequence: int, now: float,
) -> PendingArmRequest:
    command_id = self._draw_command_id()
    message = Int64MultiArray()
    message.layout.dim = []
    message.layout.data_offset = 0
    message.data = [
        command_id,
        int(target_source_epoch),
        int(requested_arm_epoch),
    ]
    self._arm_command_publisher.publish(message)
    return PendingArmRequest(
        command_id=command_id,
        target_source_epoch=int(target_source_epoch),
        requested_arm_epoch=int(requested_arm_epoch),
        pre_status_sequence=int(pre_status_sequence),
        deadline_monotonic=now + self.arm_ack_timeout_s,
    )

@staticmethod
def _receipt_matches(
    status: XsensStatusSnapshot, request: PendingArmRequest
) -> bool:
    return (
        status.status_sequence > request.pre_status_sequence
        and status.last_arm_command_id == request.command_id
        and status.last_arm_target_epoch == request.target_source_epoch
        and status.last_requested_arm_epoch == request.requested_arm_epoch
        and status.accepted_arm_epoch == request.requested_arm_epoch
    )
```

The internal methods are fixed to these signatures so phase evaluation is
separate from inference and logging:

| Helper | Exact signature |
|---|---|
| session reset | `_reset_phase_session(self) -> None` |
| latch observation | `_observe_manual_slot(self, value: int) -> None` |
| transport classification | `_status_health(self, snapshot: JoinedSourceSnapshot, now: float) -> tuple[bool, XsensHoldCause \| None]` |
| readiness recheck | `_ready_for_manual_arm(self, snapshot: JoinedSourceSnapshot, epoch: int, now: float) -> bool` |
| epoch barrier | `_accept_epoch_edge(self, snapshot: JoinedSourceSnapshot, now: float) -> None` |
| pre-live phases | `_advance_waiting_or_ready(self, snapshot: JoinedSourceSnapshot, now: float) -> None` |
| correlated join | `_advance_pending_join(self, snapshot: JoinedSourceSnapshot, now: float) -> None` |
| live hold/recovery | `_advance_live(self, snapshot: JoinedSourceSnapshot, now: float) -> None` |
| ordered dispatcher | `_advance_phase(self, snapshot: JoinedSourceSnapshot, now: float) -> None` |
| correlated cancellation | `_cancel_same_epoch_request(self, snapshot: JoinedSourceSnapshot, now: float) -> None` |
| edge-only diagnostics | `_emit_phase_diagnostic(self, snapshot: JoinedSourceSnapshot, now: float) -> None` |

Write each body from this exhaustive decision table:

| Ordered condition | Required mutation |
|---|---|
| no status | initial phases stay waiting; live closes the gate to `HOLD/STATUS_HEARTBEAT` |
| new nonzero status epoch | cancel an old join, accept `accepted_arm_epoch=0` as the atomic disarm barrier, retain an existing hold, reset readiness to that status, and send no command |
| same-epoch disarm receipt matches | clear only the pending disarm; readiness remains false until the status itself reaches 30 current frames |
| local status age `> 0.2` | waiting/ready regress to waiting; live selects `HOLD/STATUS_HEARTBEAT` |
| current producer/reference age `> 0.5` or `source_stale=true` | waiting/ready regress; live selects `HOLD/PRODUCER_STALE` |
| pending arm/re-arm and readiness regresses or `now > deadline` | publish exactly one new targeted disarm, replace pending request with it, close the latch, and keep idle/old hold |
| matching post-command ACK without canonical join | replace the frozen request with `acknowledged_frame_index=status.newest_frame_index` and remain pending |
| matching canonical epoch/frame after ACK | call `open_live_reference_gate(..., ArmGateProof(...))`; only a true result enters `LIVE/FRESH` and clears pending |
| same-epoch live recovery after producer stale | require `recovery_frames == 10` and joined frame at the latched status frame before opening with `arm_proof=None` |
| live recovery after heartbeat-only hold | require current accepted same-epoch status/reference, but do not require a fabricated recovery count |
| `READY` or re-arm-ready eligibility | all current epoch, ready, complete-window, non-stale, local-heartbeat, producer-age, and non-disarm-pending checks are true |

Create exact `XsensHoldCause.STATUS_HEARTBEAT` only for the local 0.2-second
status channel. Create `XsensHoldCause.PRODUCER_STALE` for status-declared UDP
staleness or either joined producer timestamp exceeding 0.5 seconds. Equality
at both thresholds is healthy.

Every Xsens update uses this literal order—one captured clock value, the latest
slot level, snapshot/phase/gate work, policy inference, then diagnostics:

```python
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
```

Use these literal lifecycle/action bodies. `_emit_phase_diagnostic()` owns the
two READY prompts and deduplicates by
`(phase, link_status, reason_code, pending-kind, source_blend_active)`:

```python
XSENS_READY_PROMPT = (
    "Xsens READY — release controls, then press LT+RT+Y to request LIVE"
)
XSENS_REARM_READY_PROMPT = (
    "Xsens new session READY — release controls, then press LT+RT+Y to re-arm"
)

def _reset_phase_session(self) -> None:
    self._phase = XsensPhase.WAITING_FOR_DATA
    self._link_status = None
    self._hold_cause = None
    self._current_source_epoch = None
    self._pending_source_epoch = None
    self._ready_frames = 0
    self._rearm_ready = False
    self._pending_request = None
    self._neutral_latch_open = False
    self._latest_status = None
    self._latest_status_sequence = 0
    self._entered = False
    self._diagnostic_key = None

def on_enter(self, ctx: RobotControlContext) -> None:
    self._entered = True
    self._neutral_latch_open = (
        ctx.remote_slot_value(self.manual_enable_slot)
        == self.manual_enable_neutral_value
    )
    self.logger.info(f"SONIC Xsens遥操已启动；{self.operator_prompt}")

def on_exit(self, ctx: RobotControlContext) -> None:
    now = self._monotonic()
    if self._current_source_epoch is not None:
        self._publish_command(
            target_source_epoch=self._current_source_epoch,
            requested_arm_epoch=0,
            pre_status_sequence=self._latest_status_sequence,
            now=now,
        )
    self.policy.close_live_reference_gate(
        hold_last_reference=False,
        rearm_required=False,
    )
    self._pending_request = None
    self._neutral_latch_open = False
    self._entered = False
    super().on_exit(ctx)

def _reason_name(self, snapshot: JoinedSourceSnapshot) -> str:
    if snapshot.status is None:
        return XsensReason.NO_DATA.name
    return XsensReason(snapshot.status.reason_code).name

def on_action(self, ctx: RobotControlContext, action_name: str) -> bool:
    if action_name not in {"activate_xsens", "reset_alignment"}:
        return False
    if action_name == "reset_alignment":
        context = None
        if (
            self._phase is XsensPhase.LIVE
            and not self.arm_pending
            and not self.rearm_pending
            and not self.policy.source_blend_active
            and self.policy.pending_yaw_offset is None
        ):
            context = {
                XsensLinkStatus.FRESH: "fresh",
                XsensLinkStatus.HOLD: "same_epoch_hold",
                XsensLinkStatus.HOLD_REARM_REQUIRED: "rearm_hold",
            }.get(self._link_status)
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

    self._pending_request = self._publish_command(
        target_source_epoch=epoch,
        requested_arm_epoch=epoch,
        pre_status_sequence=snapshot.status.status_sequence,
        now=now,
    )
    return True
```

`on_action()` therefore returns `True` for every recognized
`activate_xsens` or `reset_alignment` event even when phase/latch/blend rules
reject or ignore its effect; it returns `False` only for unknown action names.

On entry, initialize the neutral latch from `ctx.remote_slot_value("btn_10")`, configure policy source kind `xsens`, reset with `seed_target_from_robot=True`, and use idle. After first status, send one targeted disarm baseline. Do not honor readiness while `disarm_pending`; clear it only on a strictly newer status echoing exact ID/target/request zero and accepted zero, or on a later epoch's atomic unarmed status. Emit the exact prompts `Xsens READY — release controls, then press LT+RT+Y to request LIVE` and `Xsens new session READY — release controls, then press LT+RT+Y to re-arm` only on the corresponding phase/reason edge; an early press is handled and logs the current wait reason.

An arm/re-arm request records the pre-command status sequence. ACK requires all five exact comparisons:

```text
status_sequence > pre_status_sequence
last_arm_command_id == command_id
last_arm_target_epoch == target_source_epoch
last_requested_arm_epoch == requested_arm_epoch
accepted_arm_epoch == requested_arm_epoch
```

Latch the ACK's `newest_frame_index`; later heartbeats cannot move the join
floor. Open policy only after a same-epoch canonical reference reaches it,
passing `ArmGateProof(command_id, target_source_epoch,
requested_arm_epoch, pre_status_sequence)` so the policy independently
rechecks the exact receipt. Same-epoch regression or elapsed time `>0.5` sends
exactly one fresh-ID targeted disarm and waits for its correlated barrier.
Epoch change cancels the old join without a redundant disarm.

During LIVE: status/producer stale creates hold; a same-epoch recovery status with `recovery_frames=10`, accepted epoch, fresh/window flags latches a frame floor and auto-opens on canonical join with `arm_proof=None` and no yaw reset; a local heartbeat-only outage can rejoin current healthy status/reference without manufacturing a UDP recovery count. Changed epoch selects `HOLD_REARM_REQUIRED`, retains old hold, and requires 30-frame readiness plus a new neutral-latched manual arm and a new exact proof. Route stable manual alignment to the policy's exact ownership context. Emit logs only on phase/reason/pending/blend changes or bounded summaries, including phase/link, readiness/recovery, status age/sequence/reason, source/requested/accepted/armed epochs, exact command receipt, joined frame, neutral-latch rejection, hold/recovery/re-arm, blend completion, and yaw preserve/reset; never print a motion frame.

- [ ] **Step 4: Run state, policy, and framework-latch tests**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_state.py test/test_xsens_policy.py test/test_framework_remote_slots.py -v
```

Expected: PASS; no cached heartbeat or canonical frame can create false LIVE.

- [ ] **Step 5: Commit the Xsens phase machine**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/state.py src/bxi_example_py_elf3/test/xsens_test_helpers.py src/bxi_example_py_elf3/test/test_xsens_state.py
git commit -m "feat(sonic): add xsens live phase machine"
```

### Task 15: Register the State, Scope Nodes and Routes, Document Operation, and Run Full Regression

**Files:**

- Modify: `src/bxi_example_py_elf3/mods/com.bxi.sonic/plugin.py`
- Modify: `src/bxi_example_py_elf3/mods/com.bxi.sonic/mod.yaml`
- Create: `src/bxi_example_py_elf3/mods/com.bxi.sonic/XSENS_MVN.md`
- Modify: `src/bxi_example_py_elf3/mods/com.bxi.sonic/README.md`
- Create: `src/bxi_example_py_elf3/test/test_xsens_manifest.py`
- Modify: `src/bxi_example_py_elf3/test/test_zerolab_manifest.py`
- Modify: `src/bxi_example_py_elf3/test/test_zerolab_lifecycle.py`

**Interfaces:**

- Consumes: all previous tasks.
- Produces: plugin factory `sonic_xsens`, state-scoped `xsens_source`/`xsens_bridge`, exact event/routes/actions/params, lifecycle ownership, and an operator/vendor-confirmation runbook.
- Route invariant: PICO, ZeroLab, and Xsens live states can be entered only from basic `normal`; there are no direct routes among them.

- [ ] **Step 1: Write failing plugin, manifest, route, config, and lifecycle tests**

```python
# test/test_xsens_manifest.py
from __future__ import annotations

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
```

```python
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
        "signal": "SIGINT",
        "terminate_after": 3.0,
        "kill_after": 5.0,
    }
    assert {item["import"] for item in source["runtime_requirements"]["python"]} == {
        "numpy",
        "scipy",
        "zmq",
    }
    assert {item["package"] for item in source["runtime_requirements"]["ros"]} == {
        "rclpy",
        "std_msgs",
    }
    assert bridge["entrypoint"] == "pico.pose_to_smpl_ref_bridge:create_node"
    assert bridge["runtime"] == "python"
    assert bridge["execution"] == "in_process"
    assert bridge["runtime_profile"] == "host_ros"
    assert bridge["lifecycle"] == "state"
    assert bridge["depends_on"] == ["xsens_source"]
    assert {item["import"] for item in bridge["runtime_requirements"]["python"]} == {
        "zmq"
    }
    assert {item["package"] for item in bridge["runtime_requirements"]["ros"]} == {
        "rclpy",
        "std_msgs",
    }

    for module_name in ("numpy", "scipy", "zmq", "rclpy", "std_msgs"):
        assert importlib.import_module(module_name) is not None
    source_module = importlib.import_module("xsens.source_node")
    bridge_module = importlib.import_module("pico.pose_to_smpl_ref_bridge")
    assert callable(source_module.create_node)
    assert callable(bridge_module.create_node)

    spec, package_name = load_process_node_spec(
        MOD_ROOT / "mod.yaml", "xsens_source"
    )
    try:
        assert callable(spec.factory)
        assert spec.execution == "process"
        assert spec.states == ("com.bxi.sonic/sonic_xsens",)
        assert spec.params["udp_port"] == 9763
        assert spec.params["pose_port"] == 5559
    finally:
        _remove_module_prefixes((package_name,))

    bridge_spec, package_name = load_process_node_spec(
        MOD_ROOT / "mod.yaml", "xsens_bridge"
    )
    try:
        assert callable(bridge_spec.factory)
        assert bridge_spec.execution == "in_process"
        assert bridge_spec.states == ("com.bxi.sonic/sonic_xsens",)
        assert bridge_spec.params["pico_port"] == 5559
        assert bridge_spec.params["out_port"] == 5557
    finally:
        _remove_module_prefixes((package_name,))

    captured = {}
    monkeypatch.setattr(
        setuptools, "setup", lambda **kwargs: captured.update(kwargs)
    )
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
    assert (
        "com.bxi.basic_actions/normal",
        "com.bxi.basic_actions/normal",
        "soft_switch",
    ) in exits
    assert (
        "com.bxi.basic_actions/zero_torque",
        "com.bxi.basic_actions/zero_torque",
        None,
    ) in exits
    assert (
        "com.bxi.basic_actions/pd_brake",
        "com.bxi.basic_actions/pd_brake",
        None,
    ) in exits
    assert (
        "com.bxi.basic_actions/recover",
        "com.bxi.basic_actions/recover",
        "soft_switch",
    ) in exits


def test_xsens_has_reset_alignment_action(manifest):
    matching = [
        action for action in manifest["actions"]
        if action["from"] == "sonic_xsens"
        and action["event"] == "reset_alignment"
    ]
    assert matching == [{
        "from": "sonic_xsens",
        "event": "reset_alignment",
        "action": "reset_alignment",
        "manifest": {"label": "重置朝向对齐", "ui": "refresh"},
    }]


def test_live_source_states_have_no_direct_cross_routes(manifest):
    live_states = {"sonic_teleop", "sonic_zerolab", "sonic_xsens"}
    assert not [
        route for route in manifest["routes"]
        if route["from"] in live_states
        and route["to"] in live_states
        and route["from"] != route["to"]
    ]
    entries = {
        (route["event"], route["to"])
        for route in manifest["routes"]
        if route["from"] == "com.bxi.basic_actions/normal"
        and route["to"] in live_states
    }
    assert entries == {
        ("activate", "sonic_teleop"),
        ("activate_zerolab", "sonic_zerolab"),
        ("activate_xsens", "sonic_xsens"),
    }


def test_xsens_runbook_contains_exact_stream_settings_and_protocol_limit():
    text = (MOD_ROOT / "XSENS_MVN.md").read_text(encoding="utf-8")
    required = (
        "UDP 127.0.0.1:9763",
        "60 Hz",
        "Position + Orientation (Quaternion)",
        "one FullBody actor",
        "23 segments",
        "no props/fingers",
        "pd_brake -> normal",
        "first LT+RT+Y -> release -> READY",
        "exact ACK/reference join",
        "0.4 s blend",
        "Same epoch outage: HOLD, no button, ten-frame joined auto-recovery.",
        "New epoch: old HOLD, new READY, exact zero, one LT+RT+Y re-arm.",
        "official MVN 2025 datagram/sample/time-code/wrap/playback-seek/"
        "source-port semantics",
        "stable session/take ID or restart signal outside MXTP02",
        "standardized segment-frame meaning evidenced by the identity MVNX frame",
        "whether quaternion representatives may flip sign",
        "heading/origin setting effects on the global frame",
        "metres plus wxyz for this streamer selection",
        "BattleDragon LT axis 5, RT axis 4, Y button 4",
        "CRSF CH7/CH3/CH8-Y threshold delivery",
        "MXTP02 cannot detect a restart whose sender tuple, sample counter, "
        "and usable time code all continue forward.",
        "status_timeout_s=0.2 covers the local source/bridge heartbeat, "
        "not MVN UDP spacing.",
        "Keyboard hold/auto-repeat is unsupported.",
        "r is absent from the default hardware launch.",
    )
    for literal in required:
        assert literal in text
    assert "/home/fazepurple/" not in text
    assert "zhengbu_boy ceshi.mvnx" not in text
    readme = (MOD_ROOT / "README.md").read_text(encoding="utf-8")
    assert "XSENS_MVN.md" in readme
```

Append this structural pre-change snapshot to
`test/test_zerolab_manifest.py` before editing `mod.yaml`:

```python
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
  - {from: com.bxi.basic_actions/normal, event: activate, to: sonic_teleop,
     transition: soft_switch}
  - {from: sonic_teleop, event: com.bxi.basic_actions/normal,
     to: com.bxi.basic_actions/normal, transition: soft_switch}
  - {from: sonic_teleop, event: com.bxi.basic_actions/zero_torque,
     to: com.bxi.basic_actions/zero_torque}
  - {from: sonic_teleop, event: com.bxi.basic_actions/pd_brake,
     to: com.bxi.basic_actions/pd_brake}
  - {from: sonic_teleop, event: com.bxi.basic_actions/recover,
     to: com.bxi.basic_actions/recover, transition: soft_switch}
  - {from: com.bxi.basic_actions/normal, event: activate_zerolab,
     to: sonic_zerolab, transition: soft_switch}
  - {from: sonic_zerolab, event: com.bxi.basic_actions/normal,
     to: com.bxi.basic_actions/normal, transition: soft_switch}
  - {from: sonic_zerolab, event: com.bxi.basic_actions/zero_torque,
     to: com.bxi.basic_actions/zero_torque}
  - {from: sonic_zerolab, event: com.bxi.basic_actions/pd_brake,
     to: com.bxi.basic_actions/pd_brake}
  - {from: sonic_zerolab, event: com.bxi.basic_actions/recover,
     to: com.bxi.basic_actions/recover, transition: soft_switch}
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
    actual = {
        "nodes": {
            name: manifest["nodes"][name]
            for name in (
                "pico_manager", "smpl_bridge",
                "zerolab_source", "zerolab_bridge",
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
        "routes": [
            route for route in manifest["routes"]
            if route["from"] != "sonic_xsens"
            and route["to"] != "sonic_xsens"
            and route["event"] != "activate_xsens"
        ],
        "actions": [
            action for action in manifest["actions"]
            if action["from"] != "sonic_xsens"
        ],
    }
    assert actual == LEGACY_MANIFEST_SNAPSHOT
```

Append the lifecycle test to `test/test_zerolab_lifecycle.py`; it uses that
file's local `rclpy_runtime` fixture and therefore has no cross-test-module
fixture dependency:

```python
from xsens.source_node import SOURCE_DEFAULTS, XsensSourceNode
from xsens_test_helpers import reserve_tcp_port, reserve_udp_port


def xsens_lifecycle_source_context(udp_port, pose_port, node_name):
    params = dict(SOURCE_DEFAULTS)
    params.update({
        "udp_bind_host": "127.0.0.1",
        "udp_port": udp_port,
        "allowed_sender": "127.0.0.1",
        "pose_host": "127.0.0.1",
        "pose_port": pose_port,
    })
    return NodeBuildContext(
        "com.bxi.sonic",
        f"com.bxi.sonic/{node_name}",
        node_name,
        MOD_ROOT,
        params,
    )


def xsens_lifecycle_bridge_context(input_port, output_port, node_name):
    return NodeBuildContext(
        "com.bxi.sonic",
        f"com.bxi.sonic/{node_name}",
        node_name,
        MOD_ROOT,
        {
            "pico_host": "127.0.0.1",
            "pico_port": input_port,
            "input_pose_topic": "pose",
            "input_status_topic": "xsens_status",
            "out_host": "127.0.0.1",
            "out_port": output_port,
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
        },
    )


def test_repeated_xsens_lifecycle_releases_reserved_udp_and_zmq_ports(
    rclpy_runtime,
):
    udp_port = reserve_udp_port()
    pose_port = reserve_tcp_port()
    output_port = reserve_tcp_port()
    while output_port == pose_port:
        output_port = reserve_tcp_port()

    for suffix in ("a", "b"):
        source = XsensSourceNode(
            xsens_lifecycle_source_context(
                udp_port, pose_port, f"xsens_source_{suffix}"
            )
        )
        bridge = None
        try:
            bridge = SmplRefBridgeNode(
                xsens_lifecycle_bridge_context(
                    pose_port, output_port, f"xsens_bridge_{suffix}"
                )
            )
        finally:
            if bridge is not None:
                bridge.destroy_node()
            source.destroy_node()

    udp_probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    zmq_context = zmq.Context()
    pose_probe = zmq_context.socket(zmq.PUB)
    output_probe = zmq_context.socket(zmq.PUB)
    for probe in (pose_probe, output_probe):
        probe.setsockopt(zmq.LINGER, 0)
    try:
        udp_probe.bind(("127.0.0.1", udp_port))
        pose_probe.bind(f"tcp://127.0.0.1:{pose_port}")
        output_probe.bind(f"tcp://127.0.0.1:{output_port}")
    finally:
        udp_probe.close()
        pose_probe.close(linger=0)
        output_probe.close(linger=0)
        zmq_context.term()
```

- [ ] **Step 2: Run manifest tests and verify registration is absent**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_manifest.py -v
```

Expected: FAIL because `sonic_xsens`, its nodes, and `activate_xsens` are absent.

- [ ] **Step 3: Register the dedicated state and exact state-scoped graph**

Add `_build_xsens_state(state, policy) -> XsensSonicTeleopState` and register it without altering `_build_state()` used by PICO/ZeroLab. It must call every typed `StateBuildContext` accessor explicitly:

```python
def _build_xsens_state(state, policy):
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
        manual_enable_neutral_value=state.int_param("manual_enable_neutral_value", 0),
        seed_entry_from_robot=state.bool_param("seed_entry_from_robot", True),
        hold_last_live_reference=state.bool_param("hold_last_live_reference", True),
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
```

Add this exact source parameter block:

```yaml
udp_bind_host: 0.0.0.0
udp_port: 9763
allowed_sender: 127.0.0.1
pose_host: 127.0.0.1
pose_port: 5559
pose_topic: pose
status_topic: xsens_status
arm_command_topic: sonic/xsens_arm_command
status_rate_hz: 50.0
input_rate_hz: 60.0
publish_rate_hz: 50.0
window_frames: 10
same_epoch_resume_frames: 10
ready_frames: 30
stale_seconds: 0.5
epoch_candidate_frames: 2
epoch_candidate_timeout_s: 0.25
max_pelvis_span_m: 0.15
max_segment_deviation_deg: 20.0
```

Add this state configuration:

```yaml
require_live_reference: false
manual_live_enable: true
manual_enable_slot: btn_10
manual_enable_neutral_value: 0
seed_entry_from_robot: true
hold_last_live_reference: true
auto_resume_same_epoch: true
rearm_on_source_epoch_change: true
hardware_gripper: false
yaw_bias_rad: 1.57079632679
live_reference_timeout_s: 0.5
idle_frame_start: 3509
source_blend_seconds: 0.4
status_timeout_s: 0.2
arm_ack_timeout_s: 0.5
arm_command_topic: sonic/xsens_arm_command
```

Add these literal manifest sections; do not alter the existing PICO/ZeroLab
sections captured by the regression snapshot:

```yaml
nodes:
  xsens_source:
    runtime: python
    entrypoint: xsens.source_node:create_node
    execution: process
    runtime_profile: host_ros
    lifecycle: state
    states:
      - sonic_xsens
    params:
      udp_bind_host: 0.0.0.0
      udp_port: 9763
      allowed_sender: 127.0.0.1
      pose_host: 127.0.0.1
      pose_port: 5559
      pose_topic: pose
      status_topic: xsens_status
      arm_command_topic: sonic/xsens_arm_command
      status_rate_hz: 50.0
      input_rate_hz: 60.0
      publish_rate_hz: 50.0
      window_frames: 10
      same_epoch_resume_frames: 10
      ready_frames: 30
      stale_seconds: 0.5
      epoch_candidate_frames: 2
      epoch_candidate_timeout_s: 0.25
      max_pelvis_span_m: 0.15
      max_segment_deviation_deg: 20.0
    manifest:
      label: Xsens MVN姿态源
    runtime_requirements:
      python:
        - import: numpy
        - import: scipy
        - import: zmq
      ros:
        - package: rclpy
        - package: std_msgs
      system: []
    shutdown:
      signal: SIGINT
      terminate_after: 3.0
      kill_after: 5.0

  xsens_bridge:
    runtime: python
    entrypoint: pico.pose_to_smpl_ref_bridge:create_node
    execution: in_process
    runtime_profile: host_ros
    lifecycle: state
    states:
      - sonic_xsens
    depends_on:
      - xsens_source
    params:
      pico_host: 127.0.0.1
      pico_port: 5559
      input_pose_topic: pose
      input_status_topic: xsens_status
      out_host: 127.0.0.1
      out_port: 5557
      output_reference_topic: smpl_ref
      output_status_topic: xsens_status
      source_kind: xsens
      authoritative_input_window: true
      readiness_debounce_messages: 1
      rate_hz: 50.0
      history_frames: 5
      max_gap_frames: 200
      catch_up_enabled: true
      stale_warning_seconds: 0.5
    manifest:
      label: Xsens SMPL参考桥
    runtime_requirements:
      python:
        - import: zmq
      ros:
        - package: rclpy
        - package: std_msgs
      system: []

events:
  activate_xsens:
    slot: btn_10
    value: 11

states:
  sonic_xsens:
    manifest:
      label: SONIC Xsens遥操
      priority: 838
      group: Advanced
      icon: sports_esports
      confirm: true
      confirm_message: 请先保持近似中立姿势并等待READY；实时控制前确认机器人和人员周围安全
    params:
      operator_prompt: 保持近似中立姿势，等待 Xsens READY
      require_live_reference: false
      manual_live_enable: true
      manual_enable_slot: btn_10
      manual_enable_neutral_value: 0
      seed_entry_from_robot: true
      hold_last_live_reference: true
      auto_resume_same_epoch: true
      rearm_on_source_epoch_change: true
      hardware_gripper: false
      yaw_bias_rad: 1.57079632679
      live_reference_timeout_s: 0.5
      idle_frame_start: 3509
      source_blend_seconds: 0.4
      status_timeout_s: 0.2
      arm_ack_timeout_s: 0.5
      arm_command_topic: sonic/xsens_arm_command

routes:
  - from: com.bxi.basic_actions/normal
    event: activate_xsens
    to: sonic_xsens
    transition: soft_switch
  - from: sonic_xsens
    event: com.bxi.basic_actions/normal
    to: com.bxi.basic_actions/normal
    transition: soft_switch
  - from: sonic_xsens
    event: com.bxi.basic_actions/zero_torque
    to: com.bxi.basic_actions/zero_torque
  - from: sonic_xsens
    event: com.bxi.basic_actions/pd_brake
    to: com.bxi.basic_actions/pd_brake
  - from: sonic_xsens
    event: com.bxi.basic_actions/recover
    to: com.bxi.basic_actions/recover
    transition: soft_switch

actions:
  - from: sonic_xsens
    event: activate_xsens
    action: activate_xsens
    manifest:
      label: 请求Xsens实时控制
      ui: play_arrow
  - from: sonic_xsens
    event: reset_alignment
    action: reset_alignment
    manifest:
      label: 重置朝向对齐
      ui: refresh
```

The dynamic-import test imports `xsens.source_node`, resolves `create_node`,
and proves recursive mod-file packaging includes all six `xsens/*.py` files.

- [ ] **Step 4: Write the operator runbook and vendor hardening boundary**

Write `XSENS_MVN.md` with these literal headings and contract lines (additional
screenshots may be added later, but none are required):

```markdown
# Xsens MVN to SONIC operation

## Stream settings

MVN: UDP 127.0.0.1:9763, 60 Hz, Position + Orientation (Quaternion),
one FullBody actor, 23 segments, no props/fingers.

## Robot sequence

Robot: pd_brake -> normal -> first LT+RT+Y -> release -> READY ->
second LT+RT+Y -> exact ACK/reference join -> 0.4 s blend -> small motion.

Same epoch outage: HOLD, no button, ten-frame joined auto-recovery.

New epoch: old HOLD, new READY, exact zero, one LT+RT+Y re-arm.

Exit: existing normal/recovery/PD-brake/zero-torque controls.

status_timeout_s=0.2 covers the local source/bridge heartbeat, not MVN UDP spacing.
Keyboard hold/auto-repeat is unsupported.
r is absent from the default hardware launch.

## Vendor and deployed-hardware confirmations

- official MVN 2025 datagram/sample/time-code/wrap/playback-seek/source-port semantics
- stable session/take ID or restart signal outside MXTP02
- standardized segment-frame meaning evidenced by the identity MVNX frame
- whether quaternion representatives may flip sign
- heading/origin setting effects on the global frame
- metres plus wxyz for this streamer selection
- BattleDragon LT axis 5, RT axis 4, Y button 4
- CRSF CH7/CH3/CH8-Y threshold delivery

MXTP02 cannot detect a restart whose sender tuple, sample counter, and usable time code all continue forward.

If a vendor answer contradicts the verified 760-byte layout, wxyz order, or
standardized segment-frame assumption, stop before physical-robot motion and
revise the approved design instead of introducing an undocumented axis guess.
```

Add this exact README link:

```markdown
- [Xsens MVN streaming and operation](XSENS_MVN.md)
```

Do not include an operator desktop path as a runtime/test default and do not
copy any MVNX recording into the repository.

- [ ] **Step 5: Run focused Python and C++ suites**

Run:

```bash
cd src/bxi_example_py_elf3
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test/test_xsens_*.py test/test_framework_remote_slots.py test/test_zerolab_*.py -v
```

Expected: PASS for all Xsens and relevant legacy tests.

Run from repository root:

```bash
colcon build --packages-select remote_controller bxi_example_py_elf3 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select remote_controller bxi_example_py_elf3 --event-handlers console_direct+
colcon test-result --verbose
```

Expected: both packages build; CTest/pytest results report zero failures.

- [ ] **Step 6: Run the increasing-risk manual validation sequence**

```text
1. Pure parser/converter/readiness and full automated suites are green.
2. Inspect the operator-supplied MVNX locally only; do not add it to Git.
3. Run source + bridge with SONIC inference disabled; verify 60/50 Hz,
   finite arrays, exact epochs/status, and deterministic socket shutdown.
4. Verify BattleDragon axes/buttons with jstest --event /dev/input/jsBattleDragon.
5. Verify btn_10=11 and full release btn_10=0 using
   ros2 topic echo /motion_commands for gamepad and CRSF.
6. In MuJoCo validate WAITING, READY, exact arm join, FRESH, same-epoch
   HOLD/automatic recovery, and restarted-source HOLD_REARM_REQUIRED/manual re-arm.
7. On hardware use a safety operator at PD brake, an Xsens operator, limited
   motion amplitude, and clear surroundings; verify yaw and every limb first.
```

Stop before physical-robot motion if any verified byte-layout/quaternion/segment-frame assumption conflicts with a vendor answer. Record observed rate, direction, hold/recovery, and cleanup evidence in the implementation session's handoff rather than committing operator motion data.

- [ ] **Step 7: Commit integration and documentation**

```bash
git add src/bxi_example_py_elf3/mods/com.bxi.sonic/plugin.py src/bxi_example_py_elf3/mods/com.bxi.sonic/mod.yaml src/bxi_example_py_elf3/mods/com.bxi.sonic/XSENS_MVN.md src/bxi_example_py_elf3/mods/com.bxi.sonic/README.md src/bxi_example_py_elf3/test/test_xsens_manifest.py src/bxi_example_py_elf3/test/test_zerolab_manifest.py src/bxi_example_py_elf3/test/test_zerolab_lifecycle.py
git commit -m "feat(sonic): integrate xsens teleoperation state"
```

## Completion Evidence

Before declaring implementation complete, capture and report:

```text
git status --short
git diff --check d214de7..HEAD
all focused pytest command summaries
remote_controller CTest result
colcon test-result --verbose
ports 9763/5559/5557 released after repeated state exit
MuJoCo same-epoch auto-recovery and new-epoch manual re-arm observations
any vendor answer that changes a verified assumption
```

Do not claim physical-robot acceptance from automated or MuJoCo evidence alone. Physical validation remains a separately observed acceptance step with the safety setup in the approved specification.
