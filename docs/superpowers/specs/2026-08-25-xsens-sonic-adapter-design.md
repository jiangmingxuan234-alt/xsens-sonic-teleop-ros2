# Xsens MVN to SONIC/ELF3 Adapter Design

Date: 2026-08-25

Status: Proposed written specification based on the approved interaction
design; pending user review

## 1. Purpose

Add Xsens MVN full-body motion capture as an independent live input for the
existing ELF3 SONIC controller. MVN Analyze streams solved segment position and
orientation data over UDP. A new state-scoped adapter validates and converts
that stream into the existing SONIC pose/reference path.

The operator enters one framework state, `sonic_xsens`. That state has three
internal phases:

```text
WAITING_FOR_DATA -> READY -> LIVE
```

The adapter never enters `LIVE` automatically. A short, deliberately loose
data/stability check may make the state `READY`, but the operator must press a
dedicated control combination to enable live motion.

## 2. Approved decisions

The following decisions were made during design review:

- MVN Analyze uses UDP, not TCP, for the live motion stream.
- The configured destination is `127.0.0.1:9763`.
- MVN sends `Position + Orientation (Quaternion)` at 60 Hz.
- The observed live packet is `MXTP02`, 760 bytes, containing 23 body segments.
- SONIC consumes the newest complete reference at its existing 50 Hz rate.
- Xsens gets an independent `sonic_xsens` state; PICO and ZeroLab behavior is
  unchanged.
- `sonic_xsens` is one framework state with the three internal phases above,
  not three top-level states.
- The robot uses its packaged SONIC idle reference while waiting and ready.
- The operator does not perform an additional T-pose heading calibration.
- The supplied robot-zero image is an approximate recommended entry posture,
  not an exact human-pose template.
- Readiness checks are intentionally loose. They do not check exact elbow or
  wrist angles, bilateral symmetry, body height, or image-pose similarity.
- `READY -> LIVE` is manually armed.
- The new control event is `btn_10=11`.
- On an Xbox/BattleDragon-style controller, `btn_10=11` is `LT + RT + Y` with
  `LB` and `RB` released. The keyboard fallback is `r`.
- The same context-sensitive event is used twice: from `normal`, its first
  press enters `sonic_xsens`; after release and a `READY` indication, its
  second press enables `LIVE`.
- Loss of Xsens data does not request zero torque or PD brake by itself. It
  returns SONIC to its idle reference and requires manual re-arming.
- The existing 0.4-second SONIC source blend is used for `idle <-> live`.
- The existing ONNX/RKNN models and 1770-dimensional SONIC observation contract
  are unchanged.

## 3. Goals

The first release shall:

1. Receive and parse the verified Xsens `MXTP02` UDP datagram.
2. Validate header fields, segment completeness, finite values, quaternion
   norms, sample progression, and source freshness.
3. Normalize quaternions and make their signs continuous over time.
4. Convert the Xsens right-handed Z-up segment transforms into the existing
   SONIC/XRT-to-SMPL conversion convention.
5. Map 23 Xsens segments to the 24-joint virtual SMPL input used by the existing
   fixed-skeleton forward kinematics.
6. Derive the six ELF3 wrist values using the existing shared wrist helper.
7. Publish rolling 10-frame pose chunks on an Xsens-specific ZMQ port.
8. Reuse the existing pose-to-`smpl_ref` bridge and SONIC policy/model path.
9. Add the internal `WAITING_FOR_DATA`, `READY`, and `LIVE` phases and an
   explicit live-reference gate.
10. Preserve source timestamps and source epochs through the bridge so stale
    or restarted Xsens producers cannot be mistaken for fresh live data.
11. Provide clear transition and rejection diagnostics.
12. Preserve all existing PICO, ZeroLab, normal, PD-brake, and recovery paths.

## 4. Non-goals

The first release shall not:

- support TCP streaming;
- support Xsens messages other than `MXTP02`;
- support multiple actors, props, fingers, or non-FullBody segment layouts;
- import arbitrary MVNX files as a production control source;
- commit the user's MVNX recording or raw motion data to Git;
- use measured Xsens body dimensions to rescale the packaged SMPL skeleton;
- retarget fingers or control robot grippers;
- require a T-pose, N-pose, heading reset, or precise imitation of a picture;
- infer safety from human height, exact joint angles, or left/right symmetry;
- add automatic extrapolation to hide missing UDP frames;
- add an automatic transition from `READY` to `LIVE`;
- switch directly among PICO, ZeroLab, and Xsens SONIC states;
- change SONIC model weights, model inputs, actuator ownership, or the hardware
  watchdog.

## 5. Considered architectures

### 5.1 Selected: Xsens pose source plus existing bridge

```text
MVN UDP :9763
  -> xsens_source
  -> ZMQ pose :5559
  -> existing pose_to_smpl_ref_bridge
  -> ZMQ smpl_ref :5557
  -> gated SonicTeleopPolicy
```

This follows the already deployed ZeroLab integration pattern, preserves the
existing packed-pose contract, and keeps Xsens parsing/conversion separate from
SONIC inference.

The generic bridge gains only optional transport metadata propagation:
`producer_monotonic_ns` and `source_epoch`. Existing sources that omit these
fields retain their current behavior.

### 5.2 Rejected: publish `smpl_ref` directly from the Xsens process

This removes one local ZMQ hop, but makes the Xsens process own shared port
5557 and duplicate bridge window/metadata behavior. It also makes later source
comparison harder. The latency saving on localhost is not material compared
with the 20 ms SONIC control period.

### 5.3 Rejected: three framework states or an availability gate

Splitting waiting, ready, and live into separate framework states would add
routes and node lifetime changes for what is one operator session. Requiring a
live reference in `is_available()` would deadlock: state-lifecycle Xsens nodes
start only after the framework accepts/prepares the target state. Therefore
all three phases remain inside one state and `require_live_reference` remains
false.

## 6. End-to-end data flow

```text
Xsens suit
  -> MVN Analyze body solving
  -> Network Streamer
       UDP
       127.0.0.1:9763
       60 Hz
       Position + Orientation (Quaternion)
  -> xsens UDP receiver
       monotonic receive timestamp
       latest-complete-datagram selection
  -> MXTP02 parser
       header and 23-segment validation
       big-endian decoding
       sample-counter unwrapping
  -> Xsens converter
       quaternion normalization/sign continuity
       Z-up to XRT Y-up basis conversion
       Xsens23 -> virtual SMPL24 mapping
       fixed SMPL FK
       ELF3 wrist conversion
  -> loose readiness gate
       valid/fresh/progressing frames
       short approximate stillness window
  -> rolling 10-frame pose chunk at 50 Hz
  -> ZMQ tcp://127.0.0.1:5559 topic pose
  -> existing pose_to_smpl_ref_bridge
  -> ZMQ tcp://127.0.0.1:5557 topic smpl_ref
  -> SonicTeleopPolicy
       WAITING/READY: packaged idle reference
       LIVE: Xsens reference
       idle/live transition: 0.4 s smoothstep
  -> sonic.onnx or sonic.rknn
  -> existing 29-DOF target and ELF3 controller
```

The Xsens adapter never publishes motor commands. All actuator commands remain
inside the existing SONIC state and controller.

## 7. MVN Analyze configuration

The supported first-release configuration is exact:

```text
Protocol: UDP
Destination host: 127.0.0.1
Destination port: 9763
Rate: 60 Hz
Data: Position + Orientation (Quaternion)
Character count: 1
Body configuration: FullBody, 23 segments
Props: 0
Finger-tracking segments: 0
```

MVN may stream continuously before the controller enters `sonic_xsens`. UDP
datagrams sent while no process owns port 9763 are simply discarded. Test
listeners and recorders must be stopped before entering the state because only
one process may bind the UDP port.

## 8. Verified UDP wire contract

The real stream produced the following repeated shape:

```text
identifier: MXTP02
datagram bytes: 760
header bytes: 24
item count: 23
payload bytes: 736 = 23 * 32
byte order: big-endian
```

### 8.1 Header

| Offset | Size | Type | Meaning |
|---:|---:|---|---|
| 0 | 6 | ASCII | identifier, exact `MXTP02` |
| 6 | 4 | uint32 | sample counter |
| 10 | 1 | uint8 | datagram counter/last-datagram marker |
| 11 | 1 | uint8 | number of items, exact 23 |
| 12 | 4 | uint32 | Xsens time code |
| 16 | 1 | uint8 | character ID, supported value 0 |
| 17 | 1 | uint8 | body segment count, exact 23 |
| 18 | 1 | uint8 | prop count, exact 0 |
| 19 | 1 | uint8 | finger segment count, exact 0 |
| 20 | 2 | uint16 | reserved |
| 22 | 2 | uint16 | payload size, exact 736 |

The first release accepts only a single complete datagram with this verified
layout. It does not guess how to reassemble other message types or multi-
datagram actor/prop layouts.

### 8.2 Segment item

Each of the 23 payload items uses `>I7f`:

| Field | Type | Count |
|---|---|---:|
| segment ID | big-endian uint32 | 1 |
| global position x, y, z | big-endian float32 | 3 |
| global quaternion q0, q1, q2, q3 | big-endian float32 | 4 |

The quaternion is scalar-first `wxyz`; `q0` is the real component. It describes
the segment/body coordinate frame rotated into the Xsens global frame. Position
is the segment origin in the global frame in metres.

### 8.3 Packet validation

A datagram is accepted only when:

- its length is exactly 760;
- its identifier is exactly `MXTP02`;
- the item/body counts are 23 and payload size is 736;
- character ID is 0 and prop/finger counts are zero;
- segment IDs are unique and are exactly 1 through 23;
- every position and quaternion value is finite;
- every quaternion norm is within `[0.95, 1.05]` before normalization.

Malformed packets are rejected and counted. A rejected packet never partially
updates the latest frame.

### 8.4 Counter handling

The uint32 sample counter is unwrapped to a monotonic int64 frame index.

- A forward counter is accepted.
- A forward gap is accepted and counted as dropped source frames; it does not
  by itself reset readiness or live control.
- A duplicate counter is ignored.
- A counter older than the latest accepted counter is treated as out of order
  and ignored.
- Natural uint32 wrap-around is accepted.
- A source-process restart creates a new `source_epoch` even if the Xsens
  counter continues.

## 9. Xsens segment order

| ID | Segment | ID | Segment |
|---:|---|---:|---|
| 1 | Pelvis | 13 | LeftUpperArm |
| 2 | L5 | 14 | LeftForeArm |
| 3 | L3 | 15 | LeftHand |
| 4 | T12 | 16 | RightUpperLeg |
| 5 | T8 | 17 | RightLowerLeg |
| 6 | Neck | 18 | RightFoot |
| 7 | Head | 19 | RightToe |
| 8 | RightShoulder | 20 | LeftUpperLeg |
| 9 | RightUpperArm | 21 | LeftLowerLeg |
| 10 | RightForeArm | 22 | LeftFoot |
| 11 | RightHand | 23 | LeftToe |
| 12 | LeftShoulder |  |  |

The parser indexes by segment ID, not arrival order. A packet with 23 items but
missing, repeated, or unknown body IDs is invalid.

## 10. Quaternion and coordinate processing

### 10.1 Per-frame processing

For each accepted segment quaternion:

1. Decode Xsens `wxyz` and convert to the library's internal `xyzw` order.
2. Normalize to unit length.
3. Compare with the previous accepted quaternion for the same segment.
4. If the dot product is negative, negate the current quaternion.
5. Apply the fixed global-coordinate basis conversion.

The sign-continuity step is mandatory. The supplied MVNX sample contains 5,079
segment-level `q <-> -q` changes across adjacent normal frames, even though the
physical orientation remains continuous.

### 10.2 Global basis conversion

Xsens uses a right-handed global frame with `+Z` up. The first-release XRT input
frame preserves Xsens `+X` and maps vertical `+Z` to XRT `+Y`:

```text
[x, y, z]_xrt = [x, z, -y]_xsens
```

Let `C` be the proper rotation matrix for this mapping. Positions use
`p_xrt = C p_xsens`; rotation matrices use:

```text
R_xrt = C R_xsens C^-1
```

The existing SONIC converter then applies its established XRT-to-robot and
Y-up-to-Z-up conventions. Initial absolute horizontal heading is not a
calibration requirement: on the first enabled live reference, the existing
SONIC yaw alignment captures the current operator/robot relation.

There is no session-specific per-segment rest-pose subtraction. The first
release uses the standardized Xsens segment frames evidenced by the MVNX
identity frame, in which all 23 orientations are identity.

## 11. Xsens23 to virtual SMPL24 mapping

The converter constructs 24 virtual SMPL world rotations, then calls the
existing `compute_from_body_poses()` fixed-skeleton path.

| SMPL index | Meaning | Xsens source |
|---:|---|---|
| 0 | pelvis | Pelvis, ID 1 |
| 1 | left hip | LeftUpperLeg, ID 20 |
| 2 | right hip | RightUpperLeg, ID 16 |
| 3 | spine1 | L5, ID 2 |
| 4 | left knee | LeftLowerLeg, ID 21 |
| 5 | right knee | RightLowerLeg, ID 17 |
| 6 | spine2 | shortest-path 0.5 Slerp of L3 ID 3 and T12 ID 4 |
| 7 | left ankle | LeftFoot, ID 22 |
| 8 | right ankle | RightFoot, ID 18 |
| 9 | spine3 | T8, ID 5 |
| 10 | left foot/toe | LeftToe, ID 23 |
| 11 | right foot/toe | RightToe, ID 19 |
| 12 | neck | Neck, ID 6 |
| 13 | left collar | LeftShoulder, ID 12 |
| 14 | right collar | RightShoulder, ID 8 |
| 15 | head | Head, ID 7 |
| 16 | left shoulder | LeftUpperArm, ID 13 |
| 17 | right shoulder | RightUpperArm, ID 9 |
| 18 | left elbow | LeftForeArm, ID 14 |
| 19 | right elbow | RightForeArm, ID 10 |
| 20 | left wrist | LeftHand, ID 15 |
| 21 | right wrist | RightHand, ID 11 |
| 22 | left hand endpoint | copy virtual left wrist world rotation |
| 23 | right hand endpoint | copy virtual right wrist world rotation |

Copying the wrist rotation to the final hand endpoint makes that endpoint's
parent-relative rotation identity. Xsens finger data is not present in this
stream.

The parent list remains:

```text
[-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8,
  9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21]
```

Xsens positions are retained for validation, diagnostics, and the readiness
motion check. They do not replace packaged SMPL rest geometry, so operator
height and limb length do not alter the SONIC input distribution.

## 12. SMPL FK and ELF3 wrist output

The converter reuses:

```text
pico/gear_sonic/trl/utils/numpy_smpl.py
pico/gear_sonic/trl/utils/elf3_wrist.py
```

The FK path emits:

- `smpl_joints`: 24 fixed-skeleton joints in the root-local SONIC convention;
- `body_quat_w`: root quaternion in scalar-first `wxyz` order;
- SMPL local body pose used to derive the six ELF3 wrist joints.

The existing wrist helper produces:

```text
[left_wrist_x, left_wrist_y, left_wrist_z,
 right_wrist_x, right_wrist_y, right_wrist_z]
```

These values populate ELF3 `joint_pos` indices `19, 20, 21, 26, 27, 28`.
All other `joint_pos` entries remain zero, as in the existing pose-source
contract.

## 13. 60 Hz input and 50 Hz publication

The UDP receiver drains all available complete datagrams on each 50 Hz source
tick. The converter processes them in unwrapped-sample order and retains the
newest complete converted frame.

- Each accepted Xsens sample is appended at most once.
- An unchanged sample counter is never duplicated to fake progress.
- When more than one 60 Hz packet is available, all are validated and the
  rolling window advances through them; only the newest completed 10-frame
  chunk is published during that 50 Hz tick.
- Because publication is 50 Hz, not every intermediate rolling-window state is
  sent. Accepted 60 Hz source frames are still processed in counter order and
  may advance the next published window; this publication decimation is not an
  inferred UDP drop.
- Publication requires ten strictly increasing converted frame indices.

This matches the current SONIC behavior of using the newest complete reference
at each 20 ms control step.

## 14. Loose readiness gate

Readiness determines only whether the operator may manually enable live data.
It is not a pose classifier and does not claim the operator matches the robot.

### 14.1 Required checks

Before the source may report ready, it requires:

- valid `MXTP02` packets with all 23 segments;
- finite positions and unit-normalizable quaternions;
- advancing sample counters;
- no receive gap longer than 0.5 seconds;
- 30 accepted frames, approximately 0.5 seconds at 60 Hz;
- a complete 10-frame converted pose window;
- approximate short-term stillness over the 30-frame candidate window.

Approximate stillness uses relative motion, not absolute global position:

- pelvis-position diameter no greater than 0.15 m; and
- for every segment, the 95th percentile angular deviation from its
  hemisphere-aligned candidate mean no greater than 20 degrees.

These defaults deliberately tolerate natural sway and an approximate neutral
pose. They catch ongoing large motion, discontinuities, and clearly unstable
entry without enforcing a training pose.

### 14.2 Explicitly excluded checks

Readiness does not check:

- a T-pose or exact robot-zero pose;
- absolute yaw/heading;
- absolute pelvis coordinates;
- body height or segment lengths;
- exact elbow, shoulder, wrist, hip, knee, or ankle angles;
- bilateral symmetry;
- hands, fingers, or grippers;
- equality to the supplied image or MVNX recording.

### 14.3 Readiness while live

Before live enable, readiness reflects the current short motion window. The
state sends a reliable, volatile `std_msgs/Bool` control on relative topic
`sonic/xsens_live_enable`:

- `false`: apply the readiness/stillness gate;
- `true`: the operator has explicitly armed live control, so valid human motion
  is allowed and stillness is no longer enforced.

The message is volatile rather than latched. If the Xsens source process
restarts, it defaults to not-live and cannot silently resume from an old arm
command.

## 15. One state, three internal phases

### 15.1 WAITING_FOR_DATA

This is the entry phase. The state:

- runs SONIC on the packaged idle reference starting at frame 3509;
- starts the state-scoped Xsens source and bridge;
- sends `xsens_live_enable=false`;
- waits for a fresh, progressing, stable, complete reference;
- logs the current rejection reason without spamming each control tick.

Pressing `LT + RT + Y` in this phase does not enable live. The event is handled
and a message explains that the operator must wait for `READY`.

### 15.2 READY

The state enters `READY` when a fresh Xsens reference with a complete window is
available. The robot still uses the packaged idle reference. The state logs:

```text
Xsens READY — release controls, then press LT+RT+Y to enable LIVE
```

`READY` never advances automatically. On the next rising event for
`btn_10=11`, the state:

1. rechecks source freshness and source epoch;
2. sends `xsens_live_enable=true`;
3. enables the policy's live-reference gate;
4. resets yaw alignment so the newest Xsens root establishes heading;
5. changes the internal phase to `LIVE`.

### 15.3 LIVE

The policy consumes Xsens `smpl_ref` windows. The existing 0.4-second
smoothstep blends the current SONIC joint target into the live-source target.
After this transition, normal operator movement is allowed and the entry
stillness test is disabled.

Repeated `btn_10=11` events in `LIVE` are acknowledged but have no additional
effect. The operator leaves live control through the existing normal,
PD-brake, zero-torque, or recovery routes.

### 15.4 Stale or restarted source

If the newest producer timestamp becomes older than 0.5 seconds, the source
epoch changes, or no valid reference remains through the freshness timeout:

1. the live-reference gate is disabled;
2. `xsens_live_enable=false` is sent;
3. the policy selects its packaged idle reference;
4. the existing 0.4-second blend transitions toward the idle target;
5. the phase becomes `WAITING_FOR_DATA`;
6. returning data must pass readiness again and the operator must manually
   re-arm.

There is no automatic resume to `LIVE`.

## 16. Policy gating and entry smoothing

The shared `SonicTeleopPolicy` gains a live-reference enable flag. It continues
to receive and validate references while disabled so `sonic_xsens` can detect
`READY`, but `_active_reference()` selects live data only when the flag is
enabled.

On every `sonic_xsens` control update, phase logic polls and validates source
freshness and epoch before calling policy inference. A stale or new-epoch
reference therefore disables the live gate before that reference can be used
for a `LIVE` inference step.

Existing PICO and ZeroLab states configure the flag enabled and retain their
current automatic live-reference behavior. `sonic_xsens` configures manual
enable.

The Xsens state also seeds its initial SONIC target from the current measured
robot joint frame before the first idle inference. This prevents the existing
20 ms `soft_switch` hold from ending at an unrelated SONIC default target. The
entry seed is Xsens-state-specific; it does not change existing PICO or ZeroLab
entry behavior.

Two smoothing mechanisms remain distinct:

- `soft_switch`: holds the previous motor frame for 20 ms during framework
  state entry/exit;
- `source_blend_seconds=0.4`: smoothstep interpolation of SONIC's 29 target
  joints whenever the policy changes between idle and live references.

Raw Xsens samples and quaternions are not numerically low-pass filtered by this
0.4-second mechanism.

## 17. Pose and reference contracts

### 17.1 Xsens pose on port 5559

The Xsens source publishes the existing packed ZMQ topic `pose` with:

| Field | Dtype and shape | Meaning |
|---|---|---|
| `frame_index` | `int64 [10]` | unwrapped, strictly increasing Xsens samples |
| `smpl_joints` | `float32 [10,24,3]` | fixed-skeleton root-local positions |
| `body_quat_w` | `float32 [10,4]` | SONIC root quaternion, wxyz |
| `joint_pos` | `float32 [10,29]` | six wrist entries populated |
| `stream_mode` | `int32 [1]` | exact value 1 |
| `calibration_ready` | `bool [1]` | adapter eligibility, not an MVN T-pose calibration |
| `producer_monotonic_ns` | `int64 [1]` | receive time of newest UDP frame |
| `source_epoch` | `int64 [1]` | unique source-process session identifier |

The packed-message header remains 1,280-byte NUL-padded JSON followed by
little-endian, C-contiguous field payloads. Xsens UDP is big-endian; the local
ZMQ pose contract remains the existing little-endian format.

### 17.2 Bridge output on port 5557

The existing bridge continues to produce:

| Field | Dtype and shape |
|---|---|
| `term1_local` | `float32 [10,72]` |
| `root_quat` | `float32 [10,4]` |
| `wrist` | `float32 [10,6]` |
| `frame_index` | `int64 [1]` |
| `source_ready` | `bool [1]` |

For Xsens input it also forwards:

```text
producer_monotonic_ns int64 [1]
source_epoch          int64 [1]
```

The policy uses the producer timestamp for Xsens freshness instead of treating
each bridge re-publication as a new source frame. Sources without this optional
field continue to use local policy-receive time.

## 18. Ports and lifecycle ownership

| Purpose | Port |
|---|---:|
| MVN Xsens UDP input | 9763 |
| existing PICO pose ZMQ | 5556 |
| existing ZeroLab pose ZMQ | 5558 |
| new Xsens pose ZMQ | 5559 |
| shared SONIC `smpl_ref` ZMQ | 5557 |

`xsens_source` is the sole owner of UDP 9763 and ZMQ 5559 while
`sonic_xsens` is active. `xsens_bridge` is the sole owner of ZMQ 5557 in that
state. All sockets use zero linger and close deterministically on state exit.

PICO, ZeroLab, and Xsens states have routes only through basic `normal`.
There are no direct routes among those three states because the framework may
start target-state nodes before old-state nodes have released port 5557.

## 19. BXI Mod integration

The Mod adds:

```text
mods/com.bxi.sonic/
├── xsens/
│   ├── __init__.py
│   ├── protocol.py
│   ├── udp_receiver.py
│   ├── converter.py
│   ├── source_core.py
│   └── source_node.py
├── policy.py                  # optional source metadata and live gate
├── state.py                   # internal phase/action handling
├── plugin.py                  # sonic_xsens factory registration
└── mod.yaml                   # nodes, params, event, state, routes, action
```

Responsibilities:

- `protocol.py`: immutable wire structures and `MXTP02` parsing only;
- `udp_receiver.py`: nonblocking socket ownership, timestamps, sender filter,
  and receive/drop statistics;
- `converter.py`: quaternion/basis conversion, segment mapping, fixed FK, and
  wrist derivation as pure functions/state;
- `source_core.py`: counter unwrapping, readiness, 60-to-50 selection,
  source epoch, staleness, and the 10-frame window;
- `source_node.py`: ROS/ZMQ lifecycle and live-enable subscription.

The state configuration is:

```text
require_live_reference: false
manual_live_enable: true
seed_entry_from_robot: true
hardware_gripper: false
yaw_bias_rad: 1.57079632679
live_reference_timeout_s: 0.5
idle_frame_start: 3509
source_blend_seconds: 0.4
```

The source configuration is:

```text
udp_bind_host: 0.0.0.0
udp_port: 9763
allowed_sender: 127.0.0.1
pose_host: 127.0.0.1
pose_port: 5559
pose_topic: pose
input_rate_hz: 60.0
publish_rate_hz: 50.0
window_frames: 10
ready_frames: 30
stale_seconds: 0.5
max_pelvis_span_m: 0.15
max_segment_deviation_deg: 20.0
```

## 20. Remote and keyboard event mapping

`xbox_default.yaml` adds a keyboard source/control for `r` and one output:

```text
btn_10=11 when:
  - LT pressed
  - RT pressed
  - Y pressed
  - LB released
  - RB released
or:
  - keyboard r
```

Both triggers must cross their configured pressed thresholds. Existing
single-trigger combinations require the opposite trigger released, so they do
not collide with this dual-trigger rule.

This combination also works with the current CRSF channel model: LT and RT are
independent channels and Y is in button group A. `LB + RB` was not selected
because both shoulder buttons share one CRSF enum channel and cannot be
represented simultaneously there.

The event is rising-edge controlled. Holding the combination while the first
press enters `sonic_xsens` does not also enable `LIVE`; the operator must
release it and press it again after `READY`.

## 21. Error handling and network behavior

- Wrong UDP length/identifier/counts: reject the whole packet and count it.
- Missing/duplicate segment ID: reject the whole packet and count it.
- Non-finite value or invalid quaternion norm: reject the whole packet.
- Duplicate/out-of-order sample: ignore and count; do not regress the window.
- Forward sample gap: accept the new frame and count missing samples.
- Receive gap up to 0.5 seconds: retain the last valid live reference; do not
  trigger a phase change from one ordinary delayed packet.
- Receive age over 0.5 seconds: stop live publication and return to
  `WAITING_FOR_DATA` through the policy gate.
- Source epoch change: require manual re-arm even when UDP is already fresh.
- ZMQ bind failure: fail node preparation visibly; never choose another port.
- Conversion/FK failure: reject the frame and never publish partial arrays.
- Repeated invalid data eventually becomes stale through the same timeout.
- State exit: send live false and close UDP/ZMQ resources deterministically.

The stale transition does not request PD brake or zero torque. SONIC holds its
current blended trajectory toward the packaged idle reference. Existing
operator emergency controls and the hardware command watchdog remain
available and unchanged.

## 22. Diagnostics

Logs are emitted only on state/reason changes and periodic summaries, not once
per packet. They include:

- source bind address and expected protocol;
- sender address;
- received, accepted, malformed, duplicate, out-of-order, and inferred-drop
  counters;
- measured packet rate and newest source age;
- readiness progress and the active rejection reason;
- `WAITING_FOR_DATA`, `READY`, and `LIVE` transitions;
- manual enable accepted/rejected;
- stale and source-epoch events;
- ZMQ publication drops and lifecycle shutdown.

The phase prompt tells the operator exactly when the second control press is
allowed. Diagnostic logs never print full motion frames.

## 23. MVNX evidence and test-data policy

The local file `/home/fazepurple/桌面/zhengbu_boy ceshi.mvnx` was inspected as
design evidence:

```text
MVNX version: 4
MVN exporter: Xsens 2025.0.1 on Linux
configuration: FullBody
segment count: 23
recorded frame rate: 240 Hz
normal frames: 2384, indices 0..2383
duration: 9929 ms
calibration metadata frames: identity, tpose, tpose-isb
per-frame data: orientation 23x4 and position 23x3
```

All values are finite and quaternion norm error is below `8.14e-7`. The file
contains no serialized joint, sensor, velocity, acceleration, or contact
channels. Its 240 Hz recording rate does not change the live Network Streamer
configuration of 60 Hz.

The file is not copied into the repository. Automated tests use synthetic,
anonymous packets and synthetic quaternion sign flips. A local opt-in
validation command may accept an operator-supplied MVNX path, but no test or
runtime code hardcodes the desktop path.

## 24. Testing

Tests are added under `src/bxi_example_py_elf3/test/`.

### 24.1 Protocol tests

- Decode an exact synthetic 760-byte, big-endian `MXTP02` packet.
- Assert every header offset and all 23 `>I7f` items.
- Reject wrong identifiers, sizes, counts, character IDs, props, fingers,
  payload sizes, missing IDs, repeated IDs, NaN, infinity, and invalid norms.
- Prove item arrival order does not affect ID-indexed output.
- Prove the captured six-byte header and observed 760-byte shape are accepted.

### 24.2 Counter and receiver tests

- Accept normal increments and uint32 wrap-around.
- Ignore duplicate and out-of-order samples.
- Accept forward gaps and report the missing count.
- Use monotonic receive time rather than Xsens time code for freshness.
- Drain multiple queued datagrams and retain the newest complete progression.
- Release UDP 9763 after repeated receiver start/stop cycles.

### 24.3 Conversion tests

- Normalize quaternions and eliminate `q <-> -q` discontinuities.
- Apply the exact `[x,z,-y]` vector basis and matrix conjugation.
- Map each of the 23 segments to the declared virtual SMPL joints.
- Use shortest-path Slerp for the L3/T12 spine midpoint.
- Preserve explicit left/right toe motion.
- Make virtual hand-endpoint local rotations identity.
- Produce finite `smpl_joints [24,3]`, `body_quat_w [4]`, and
  `joint_pos [29]`.
- Populate only the six declared wrist indices.
- Preserve fixed skeleton scale when global Xsens positions are translated or
  subject proportions change.
- Validate isolated yaw, arm, elbow, leg, foot, toe, and mirrored motions.

### 24.4 Readiness and rate tests

- No ready output before 30 valid stable frames and a full 10-frame window.
- Approximate neutral poses pass without matching a T-pose template.
- Pelvis/global origin offsets do not affect stability.
- Motion beyond either loose threshold prevents `READY` before arm.
- A counter gap does not reset readiness; a 0.5-second receive gap does.
- `live_enable=true` disables the stillness requirement but not validity or
  freshness checks.
- 60 Hz input produces progressing newest-frame publication at 50 Hz without
  duplicate frame indices.

### 24.5 Contract and bridge tests

- Every pose field has the exact dtype and shape.
- Existing packed-message decoding accepts Xsens output.
- The bridge produces exact `term1_local [10,72]`, `root_quat [10,4]`, and
  `wrist [10,6]`.
- Producer timestamp and source epoch survive the bridge unchanged.
- A repeated bridge publication cannot extend producer freshness.
- Existing PICO and ZeroLab messages without optional metadata remain accepted
  with unchanged freshness semantics.

### 24.6 State and policy tests

- `sonic_xsens` can be entered without pre-existing UDP data.
- First `btn_10=11` from normal enters the state; holding the event cannot arm
  live.
- `WAITING_FOR_DATA` uses idle and rejects a live-enable action.
- Fresh complete input changes waiting to `READY` but still uses idle.
- A second rising `btn_10=11` changes ready to `LIVE`.
- The live transition resets yaw and starts one 0.4-second source blend.
- Entry target seeding starts from the current measured robot joint frame.
- Stale data and source-epoch changes return to waiting and require another
  manual arm.
- No stale condition requests PD brake or zero torque.
- Existing normal, recover, PD-brake, and zero-torque routes still work.
- Existing PICO and ZeroLab states retain automatic reference behavior.

### 24.7 Mapping and lifecycle tests

- `LT + RT + Y` and keyboard `r` emit `btn_10=11` and release to zero.
- The new rule does not emit existing single-trigger actions.
- Mod validation has no conflicting event/route/action for the same state.
- Repeated enter/exit releases UDP 9763, ZMQ 5559, and bridge ZMQ 5557.
- No direct PICO/ZeroLab/Xsens route can overlap ownership of port 5557.

## 25. Validation sequence

Validation proceeds in increasing-risk order:

1. Run all pure parser/converter/readiness tests.
2. Validate the user-supplied MVNX locally without adding it to Git.
3. Run the live source and bridge with SONIC inference disabled; inspect rates,
   counters, phase, and array validity.
4. Verify controller axes with:

   ```bash
   jstest --event /dev/input/jsBattleDragon
   ```

5. Verify `btn_10=11` with `ros2 topic echo /motion_commands`.
6. Run the complete chain in MuJoCo: waiting, ready, manual live, normal exit,
   UDP interruption, recovery, and re-arm.
7. Run the physical robot first with a safety operator at PD brake, a second
   operator wearing Xsens, limited motion amplitude, and clear surroundings.
8. Expand motions only after yaw, left/right limbs, wrists, feet, and stale
   fallback have been observed correctly.

## 26. Operator procedure

```text
1. Start MVN Analyze and confirm Network Streamer configuration.
2. Start playback/live capture and confirm MXTP02 packets reach UDP 9763.
3. Put the robot in pd_brake, then enter normal using the existing controls.
4. Stand approximately in the recommended neutral/image posture and remain
   briefly still; exact imitation is not required.
5. Press LT+RT+Y once, then release, to enter sonic_xsens.
6. Confirm the robot remains on SONIC idle and wait for the READY log.
7. Press LT+RT+Y a second time, then release, to enable LIVE.
8. Wait through the 0.4-second blend and begin with small, slow movements.
9. If data becomes stale, stop moving, wait for READY again, and manually
   re-arm. The system never resumes live automatically.
10. Use the existing normal/PD-brake/zero-torque controls to leave or stop.
```

Keyboard `r` follows the same first-press/second-press behavior.

## 27. Acceptance criteria

The first release is accepted when:

1. All new and existing relevant automated tests pass.
2. Real `MXTP02` packets parse continuously at approximately 60 Hz with the
   verified 760-byte/23-segment structure.
3. Synthetic and real quaternion sign flips do not create converted-pose jumps.
4. The adapter reaches `READY` from an approximate neutral posture without a
   T-pose or strict body-angle test.
5. The robot/MuJoCo remains on the packaged idle reference until the second
   `btn_10=11` rising event.
6. Manual enable establishes yaw alignment and a finite live 10-frame
   reference, then applies the configured 0.4-second blend.
7. Left/right arms, legs, feet, toes, torso, head, and wrist controls have the
   expected direction in MuJoCo.
8. A source gap or restart returns to idle without automatic live resumption.
9. Normal, PD-brake, zero-torque, and recovery exits remain available.
10. Repeated state entry/exit leaves no process or socket owning ports 9763,
    5559, or 5557.
11. PICO, ZeroLab, their ports, and their runtime behavior remain unchanged.
12. No user motion recording is committed to the repository.

## 28. Non-blocking vendor and hardware confirmations

Implementation can begin from the verified stream and official MVN manual.
The following confirmations improve hardening but do not block the first
parser/converter implementation:

- the official Real-Time Network Streaming Protocol document for MVN 2025,
  especially datagram-counter semantics and sample-counter wrap guarantees;
- confirmation that `MXTP02` segment quaternions use the standardized segment
  frame represented by the MVNX identity pose;
- confirmation that quaternion representatives are not guaranteed to be
  sign-continuous over time;
- confirmation of global-coordinate behavior when MVN heading/origin settings
  are changed;
- confirmation that Position remains metres and Orientation remains `wxyz` for
  this Network Streamer selection;
- BattleDragon hardware confirmation that LT is axis 5, RT is axis 4, and Y is
  button 4 on the deployed controller;
- CRSF confirmation that CH7, CH3, and CH8/Y reach the configured thresholds.

Any vendor answer that contradicts the verified byte layout, quaternion order,
or standardized segment-frame assumption requires a design-spec revision
before physical-robot validation; it must not be patched with an undocumented
axis guess.
