# ZeroLab F2 Pro to SONIC/BXI Adapter Design

Date: 2026-08-04

Status: Approved design baseline

## 1. Purpose

Add ZeroLab F2 Pro full-body motion capture as a second human-motion source for
the existing ELF3 SONIC controller. The adapter must translate the vendor UDP
stream into the existing internal pose contract. The existing PICO path,
pose-to-smpl_ref bridge algorithms, SONIC policy, ONNX model, and robot command
path remain unchanged. The shared state receives only a source-specific
operator prompt; its control behavior is unchanged.

The first release prioritizes a working and diagnosable ZeroLab path. It does
not implement the complete PICO-versus-ZeroLab evaluation suite.

## 2. Approved operating assumptions

- The subject wears the 15-sensor ZeroLab configuration, including independent
  left and right shoulder sensors.
- The subject may wear PICO and ZeroLab simultaneously for paired offline data
  collection.
- MotionCaptureMaster runs on Windows, performs vendor calibration and skeleton
  solving, and continuously sends UDP to Ubuntu after Stream Output is enabled.
- The default destination is the Ubuntu host on UDP port 18000.
- ZeroLab sends 50 frames per second.
- Every datagram is exactly 992 bytes.
- Numeric fields are Windows little-endian.
- Joint quaternions use x, y, z, w order.
- All 47 joint quaternions are world-space attitudes in Unity coordinates.
- The vendor rest pose is T-pose.
- The Left Hand and Right Hand transforms describe the palm rigid bodies after
  the wrists.
- On entering the ZeroLab SONIC state, the subject holds T-pose for two seconds.
  The adapter uses approximately 100 valid frames for a session-specific rest
  alignment before declaring the source ready.
- Finger tracking and gripper control are excluded from the first release.

## 3. Goals

The first release shall:

1. Receive, validate, timestamp, and optionally record the complete ZeroLab UDP
   datagram.
2. Parse every documented field, even when a field is not consumed by SONIC.
3. Compute a stable T-pose rest alignment from 100 valid frames.
4. Convert the first 17 body world rotations to a canonical SMPL body pose.
5. Synthesize missing spine, neck, and toe rotations deterministically.
6. Run the existing fixed SMPL skeleton forward kinematics.
7. Derive the six native ELF3 wrist angles with the same convention used by the
   PICO manager.
8. Publish the existing pose contract as 10-frame chunks on a ZeroLab-specific
   ZMQ port.
9. Reuse the existing pose-to-smpl_ref bridge and SONIC policy unchanged.
10. Add an independent BXI state for ZeroLab so PICO and ZeroLab do not compete
    for ports.
11. Fail safely on malformed, stale, uncalibrated, or non-finite input.

## 4. Non-goals

The first release shall not:

- send the 30 finger-joint quaternions to SONIC;
- map the 12 hand uint16 values to grippers;
- change sonic.onnx or the 1770-dimensional observation contract;
- replace fixed SMPL geometry with ZeroLab-measured joint positions;
- implement sub-frame PICO/ZeroLab latency calibration;
- implement long-session clock drift correction;
- implement the final metric report or composite score;
- implement deterministic full-sequence offline SONIC inference;
- implement seamless live switching between PICO and ZeroLab;
- modify actuator ownership, robot safety states, or motor command publication.

## 5. End-to-end architecture

### 5.1 ZeroLab live control

~~~
ZeroLab sensors
  -> Windows MotionCaptureMaster
  -> UDP 992 bytes at 50 Hz to Ubuntu:18000
  -> zerolab source
       protocol validation
       monotonic receive timestamp
       optional raw recording
       two-second T-pose calibration
       body conversion and SMPL FK
       ELF3 wrist conversion
       10-frame pose chunk
  -> ZMQ tcp://127.0.0.1:5558 topic pose
  -> existing pose_to_smpl_ref_bridge
  -> ZMQ tcp://127.0.0.1:5557 topic smpl_ref
  -> existing SonicTeleopPolicy
  -> obs_dict [1,1770]
  -> sonic.onnx
  -> action [1,29]
  -> existing clip and action scaling
  -> target_dof_pos [29]
  -> existing ELF3 controller
~~~

### 5.2 Paired offline capture

Paired collection does not send both sources into SONIC simultaneously.

~~~
PICO existing collection on Ubuntu -----------+
                                               +-> paired trial storage
ZeroLab record-only UDP receiver on Ubuntu ----+
~~~

Both receive paths use the Ubuntu monotonic clock. A later evaluation component
will align a visible synchronization movement, place both sources on a common
50 Hz timeline, and create comparable canonical files. That component is
outside this first implementation.

## 6. UDP wire contract

The parser shall use the following fixed layout:

| Byte range | Type | Shape | Meaning |
|---|---|---:|---|
| 0-11 | little-endian float32 | 3 | root_translation |
| 12-763 | little-endian float32 | 47 x 4 | world joint quaternion, xyzw |
| 764-775 | little-endian uint16 | 6 | left hand bends and thumb rotation |
| 776-787 | little-endian uint16 | 6 | right hand bends and thumb rotation |
| 788-991 | little-endian float32 | 17 x 3 | main joint positions |

The first block is 191 float32 values:

~~~
3 root translation values + 47 x 4 quaternion values = 191
191 x 4 bytes = 764 bytes
~~~

The parser shall retain:

~~~
receive_timestamp_ns       int64
local_frame_index          int64
root_translation           float32 [3]
joint_quat_world_xyzw      float32 [47,4]
left_hand_values           uint16  [6]
right_hand_values          uint16  [6]
joint_position             float32 [17,3]
raw_payload                uint8   [992]
sender_address             metadata
~~~

The first 17 quaternions are body transforms. The remaining 30 are three joints
for each finger on both hands and are recorded but not converted to robot
actions.

The UDP root_translation is the translation of the parent of Hips. It is not a
root quaternion. SONIC root orientation is derived from body quaternion index
10, Hips/Pelvis.

## 7. Body skeleton

The ZeroLab body indices and parent tree are:

~~~
10 Hips/Pelvis
├── 1 Chest
│   ├── 0 Head
│   ├── 2 Left Shoulder
│   │   └── 3 Left Upper Arm
│   │       └── 4 Left Forearm
│   │           └── 5 Left Hand/Palm
│   └── 6 Right Shoulder
│       └── 7 Right Upper Arm
│           └── 8 Right Forearm
│               └── 9 Right Hand/Palm
├── 11 Left Thigh
│   └── 12 Left Calf
│       └── 13 Left Foot
└── 14 Right Thigh
    └── 15 Right Calf
        └── 16 Right Foot
~~~

Independent shoulder sensors are present. Left Shoulder and Right Shoulder are
therefore treated as measured clavicle/collar transforms. Upper Arm transforms
are treated as humerus/shoulder transforms. Hand transforms are treated as palm
rigid bodies and supply wrist rotations relative to the forearms.

## 8. Quaternion processing and T-pose calibration

### 8.1 Per-frame validation

For every quaternion:

1. Reject non-finite values.
2. Reject norms below 1e-6.
3. Normalize to unit length.
4. Maintain sign continuity against the previous accepted quaternion. If the
   dot product is negative, negate the current quaternion.

### 8.2 Calibration state machine

On entry to the ZeroLab state:

1. Set calibration_ready to false.
2. Clear the 10-frame output buffer.
3. Collect 100 consecutive valid body frames while the subject holds T-pose.
4. Normalize each incoming frame and align its quaternion signs against the
   previous accepted calibration frame.
5. Append the aligned frame to a candidate window, then compute the normalized
   component mean for each of the 17 body joints over that candidate.
6. Compute quaternion angular distance from every candidate frame, for every
   joint, to that candidate mean. If any distance is strictly greater than five
   degrees, discard the old window and retain the current frame as frame one of
   a new window; equality does not restart calibration. Otherwise accept the
   entire candidate window.
7. At exactly 100 accepted frames, store the already-validated candidate mean
   as the rest rotations for the current state session.
8. Collect 10 new post-calibration frames before publishing a ready pose chunk.
9. Do not automatically recalibrate during the session. Leaving and re-entering
   the state starts a new calibration.

### 8.3 Rest alignment

For joint j, let G_rest_j be its averaged world rotation in T-pose and G_raw_j(t)
be its current world rotation. The aligned world rotation is:

~~~
G_aligned_j(t) = G_raw_j(t) * inverse(G_rest_j)
~~~

At T-pose, every aligned joint rotation is identity. A rigid whole-body yaw
therefore changes the aligned pelvis world rotation while leaving non-root
parent-relative rotations near identity.

This session-specific alignment removes sensor mounting and bind-orientation
offsets without replacing MotionCaptureMaster's sensor calibration.

## 9. ZeroLab to SMPL mapping

The adapter constructs virtual SMPL world rotations in the same Unity-frame
convention expected by the existing NumPy PICO conversion.

| SMPL body index | Meaning | ZeroLab source or synthesis |
|---:|---|---|
| 0 | pelvis | aligned Hips, index 10 |
| 1 | left hip | aligned Left Thigh, index 11 |
| 2 | right hip | aligned Right Thigh, index 14 |
| 3 | spine1 | SO(3) interpolation pelvis to chest at 1/3 |
| 4 | left knee | aligned Left Calf, index 12 |
| 5 | right knee | aligned Right Calf, index 15 |
| 6 | spine2 | SO(3) interpolation pelvis to chest at 2/3 |
| 7 | left ankle | aligned Left Foot, index 13 |
| 8 | right ankle | aligned Right Foot, index 16 |
| 9 | spine3/chest | aligned Chest, index 1 |
| 10 | left foot/toe | copy left ankle world rotation |
| 11 | right foot/toe | copy right ankle world rotation |
| 12 | neck | SO(3) interpolation chest to head at 1/2 |
| 13 | left collar | aligned Left Shoulder, index 2 |
| 14 | right collar | aligned Right Shoulder, index 6 |
| 15 | head | aligned Head, index 0 |
| 16 | left shoulder | aligned Left Upper Arm, index 3 |
| 17 | right shoulder | aligned Right Upper Arm, index 7 |
| 18 | left elbow | aligned Left Forearm, index 4 |
| 19 | right elbow | aligned Right Forearm, index 8 |
| 20 | left wrist | aligned Left Hand/Palm, index 5 |
| 21 | right wrist | aligned Right Hand/Palm, index 9 |

The final two SONIC FK output points are fixed hand-chain endpoints selected by
the existing SMPL asset. They do not consume ZeroLab finger rotations.

SO(3) interpolation uses shortest-path quaternion Slerp. Copying ankle world
rotation to the toe makes the toe parent-relative rotation identity. This is
intentional because ZeroLab supplies no independent toe transform.

The virtual 24-joint parent list is:

~~~
[-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8,
  9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21]
~~~

## 10. Coordinate conversion and fixed skeleton FK

The converter shall reuse:

~~~
com.bxi.sonic/pico/gear_sonic/trl/utils/numpy_smpl.py
~~~

After rest alignment and missing-joint synthesis, it constructs virtual
24-by-7 body poses with Unity world quaternions in xyzw order and calls the
existing NumPy path. The existing path:

- converts global rotations to parent-relative local rotations;
- applies the established PICO/SONIC base orientation convention;
- converts Unity Y-up orientation to the SONIC robot convention;
- runs fixed 55-joint SMPL-X skeleton FK;
- selects the 24 points consumed by SONIC;
- rotates FK positions into the published root frame;
- emits the root quaternion in wxyz order.

The adapter shall not use the PICO manager's three-point visualization
conversion or visualization offsets. Those are unrelated to the SONIC SMPL
tokenizer and would apply a second coordinate transform.

ZeroLab root_translation and joint_position are stored for diagnostics. They do
not replace fixed-skeleton FK positions. Their unit and exact position-space
semantics are not required for the first SONIC integration and are explicitly
excluded from first-release position metrics.

## 11. ELF3 wrist conversion

The adapter derives six ELF3 wrist angles from the SMPL local axis-angle pose.
It shall reproduce the current PICO formulas exactly.

The relevant body-pose indices are:

~~~
left elbow   17
right elbow  18
left wrist   19
right wrist  20
~~~

Elbow rotations are decomposed into swing and twist about the local Y axis.
Elbow twist is discarded. Intrinsic XYZ Euler components are combined as:

~~~
Lx = left_elbow_swing_x + left_wrist_x
Ly = left_wrist_y
Lz = left_elbow_swing_z + left_wrist_z

Rx = -(right_elbow_swing_x + right_wrist_x)
Ry = -right_wrist_y
Rz = right_elbow_swing_z + right_wrist_z
~~~

The output order is:

~~~
[Lx, Ly, Lz, Rx, Ry, Rz]
~~~

Values are radians. The first release does not add wrapping or clipping because
that would change the distribution seen by the existing SONIC model.

## 12. Existing pose contract

The ZeroLab source publishes topic pose on TCP port 5558 using the existing
packed ZMQ format and pack_pose_message helper.

Every ready message contains 10 strictly increasing frames:

| Field | Dtype and shape | Meaning |
|---|---|---|
| frame_index | int64 [10] | adapter-generated, strictly increasing |
| smpl_joints | float32 [10,24,3] | fixed-skeleton FK positions in root frame |
| body_quat_w | float32 [10,4] | SONIC root quaternion, wxyz |
| joint_pos | float32 [10,29] | only six ELF3 wrist indices are populated |
| stream_mode | int32 [1] | value 1 |
| calibration_ready | bool [1] | true only after stable calibration and full window |

The six joint_pos indices are:

~~~
19, 20, 21, 26, 27, 28
~~~

All other joint_pos values are zero, matching the current PICO pose contract.

The source publishes a rolling 10-frame chunk after every new valid post-
calibration frame. It does not publish a tiled single frame. This preserves the
existing bridge's temporal semantics.

## 13. BXI integration

### 13.1 Ports

| Purpose | Port |
|---|---:|
| ZeroLab vendor UDP input | 18000 |
| Existing PICO pose ZMQ | 5556 |
| New ZeroLab pose ZMQ | 5558 |
| Existing SONIC smpl_ref ZMQ | 5557 |

The ZeroLab ZMQ publisher binds only to 127.0.0.1 by default. UDP binds to
0.0.0.0 by default and may optionally filter the Windows sender address.

### 13.2 New package boundary

~~~
mods/com.bxi.sonic/
├── zerolab/
│   ├── __init__.py
│   ├── protocol.py
│   ├── udp_receiver.py
│   ├── recording.py
│   ├── converter.py
│   └── source_node.py
└── existing PICO, bridge, policy, state, and model files
~~~

Responsibilities:

- protocol.py: wire layout only;
- udp_receiver.py: socket ownership, receive timestamp, sender filter, statistics;
- recording.py: lossless raw records and deterministic reading;
- converter.py: calibration, skeleton mapping, FK, and wrist output as pure logic;
- source_node.py: BXI lifecycle, live/record mode, rolling window, and sole ZMQ
  publisher ownership.

### 13.3 New state

mod.yaml adds:

- zerolab_source, a state-scoped Python process;
- zerolab_bridge, an instance of the existing pose_to_smpl_ref bridge configured
  to read port 5558 and publish port 5557;
- sonic_zerolab, a state using the existing SonicTeleopPolicy and
  SonicTeleopState;
- routes between basic normal and sonic_zerolab.

SonicTeleopState and its plugin factory gain a source-specific operator-prompt
parameter. The existing PICO state keeps its current default prompt. The
ZeroLab state instructs the operator to hold T-pose until calibration completes.
This is a logging and user-guidance change only.

sonic_zerolab uses:

~~~
require_live_reference: false
hardware_gripper: false
rate_hz: 50
window_frames: 10
udp_bind_host: 0.0.0.0
udp_port: 18000
pose_host: 127.0.0.1
pose_port: 5558
pose_topic: pose
stale_seconds: 0.5
~~~

require_live_reference is false because state-scoped source nodes start only
after the framework accepts the target state. Requiring an already-live
reference during availability checking would prevent the ZeroLab source from
ever starting. During the two-second calibration and initial 10-frame fill, the
existing policy remains on its packaged idle reference. Once a ready smpl_ref
arrives, the existing source-blend behavior transitions to live ZeroLab motion.
The operator prompt and state confirmation explicitly require holding T-pose
through this interval.

PICO and ZeroLab bridges are state-scoped and mutually exclusive. There is no
direct route between the two SONIC states because target-state preparation can
temporarily overlap node lifetimes and cause both bridges to bind port 5557.
The operator returns to basic normal before entering the other source state.

## 14. Raw recording

Raw recording is optional in live mode and always enabled in record-only mode.
A recording contains:

~~~
metadata.json
records.bin
~~~

metadata includes protocol version, packet size, expected rate, sender, start
time, joint order, mapping version, sensor count 15, and shoulder-sensor
presence.

records.bin is a sequence of fixed-size little-endian records:

~~~
receive_timestamp_ns  uint64
local_frame_index     uint64
payload               uint8 [992]
~~~

The writer flushes on clean shutdown. The reader rejects truncated trailing
records. Recordings are stored outside the Mod source directory.

## 15. Error handling and safety

- Datagram length other than 992: reject and count.
- Non-finite root, quaternion, or position float: reject and count.
- Near-zero quaternion norm: reject and count.
- Unexpected sender when a sender filter is configured: ignore and count.
- UDP receive gap over 0.5 seconds: mark source stale, clear the output window,
  and stop ready publication.
- No completed rest calibration: calibration_ready remains false.
- Invalid synthesized or FK output: reject the frame and never publish it.
- ZMQ bind failure: fail node startup visibly; do not silently select another
  port.
- Disk recording failure: report the failure. Live publication may continue
  only when recording is optional; record-only mode exits non-zero.
- State exit: close UDP, ZMQ, and recording resources deterministically.

The source never publishes actuator commands. Existing BXI state and policy
safety behavior remains the only robot command path.

Before calibration, or after a stale source, the existing policy uses its idle
reference and existing blend behavior. The ZeroLab adapter does not invent,
repeat, or extrapolate motion to conceal missing UDP input.

## 16. Testing

Tests are added under:

~~~
src/bxi_example_py_elf3/test/
├── test_zerolab_protocol.py
├── test_zerolab_recording.py
├── test_zerolab_converter.py
├── test_zerolab_pose_contract.py
└── test_zerolab_lifecycle.py
~~~

### 16.1 Protocol tests

- Exact synthetic 992-byte packet parses at every documented offset.
- Root, all 47 quaternions, both hand blocks, and all 17 positions round-trip.
- Truncated and oversized packets are rejected.
- NaN, infinity, and invalid quaternion values are rejected.
- Little-endian interpretation matches the vendor C# example.

### 16.2 Calibration and conversion tests

- Quaternion sign flips do not create discontinuities.
- Stable T-pose completes after 100 valid frames.
- Moving calibration data restarts the calibration window.
- T-pose produces near-zero parent-relative body rotations.
- A rigid whole-body yaw changes root orientation while keeping root-local FK
  positions unchanged within tolerance.
- A synthetic left-elbow motion affects the expected left arm chain only.
- Left/right mapping is symmetric for mirrored synthetic input.
- Missing spine and neck rotations use the specified Slerp fractions.
- Toe local rotations remain identity.
- FK output is finite and has shape 24 by 3.
- Wrist output is finite, radians, and has the specified six-value order.

### 16.3 Contract and lifecycle tests

- Every pose field has the required dtype and shape.
- Frame indices are strictly increasing.
- No ready message is published before calibration and 10 post-calibration
  frames.
- State entry is possible before live readiness, and the policy remains on its
  idle reference until ZeroLab readiness is established.
- Existing pack_pose_message and bridge decoding accept ZeroLab output.
- Three progressing messages satisfy the existing bridge readiness gate.
- Stale input clears readiness and the window.
- Repeated enter and exit releases UDP 18000, ZMQ 5558, and bridge ZMQ 5557.
- PICO 5556 is never bound by the ZeroLab source.

### 16.4 Hardware validation recordings

After the minimal recorder is available, collect:

1. T-pose for five seconds;
2. rigid left and right whole-body yaw;
3. isolated left-elbow flexion;
4. isolated left-leg lift;
5. shoulder shrug and arm elevation.

The recordings validate actual rest quaternions, axis signs, palm semantics,
shoulder behavior, packet cadence, and the absence of undocumented headers.

## 17. Acceptance criteria

The first release is accepted when:

1. All automated protocol, conversion, contract, and lifecycle tests pass.
2. A real 15-sensor recording parses without undocumented bytes or non-finite
   output.
3. Two-second T-pose calibration reliably reaches ready while motion prevents
   false readiness.
4. Rigid yaw and isolated-joint recordings satisfy the expected invariants.
5. The existing bridge emits term1_local [10,72], root_quat [10,4], and wrist
   [10,6] from the ZeroLab pose source.
6. The existing policy accepts the resulting smpl_ref without changes.
7. MuJoCo can enter sonic_zerolab on the idle reference, transition to finite
   live 29-DOF targets after calibration, handle UDP interruption through
   existing idle fallback behavior, and leave the state cleanly.
8. Existing PICO SONIC behavior and ports remain unchanged.

## 18. Follow-on work

After the adapter is accepted, a separate evaluation design will add:

- simultaneous PICO and ZeroLab trial coordination;
- common Ubuntu timestamps for both sources;
- synchronization-event delay estimation;
- common 50 Hz resampling;
- pico_canonical.npz and zerolab_canonical.npz;
- SMPL consistency metrics;
- deterministic SONIC open-loop comparison with identical robot histories;
- separately reset MuJoCo closed-loop trials and robot metrics.

These features consume the adapter's raw recordings and canonical pose output
without changing the live ZeroLab-to-pose boundary defined here.
