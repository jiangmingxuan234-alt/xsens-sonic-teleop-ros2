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

The initial transition into `LIVE`, and adoption of any new source epoch, are
never automatic. A short, deliberately loose data/stability check may make the
state `READY`, but the operator must press a dedicated control combination to
enable live motion. Recovery from an ordinary same-epoch network gap is
different: the base phase remains `LIVE` and may return automatically from
`HOLD` to `FRESH`.

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
  `LB` and `RB` released. The development keyboard fallback is `r`.
- The same context-sensitive event is used twice: from `normal`, its first
  press enters `sonic_xsens`; after the mapped command has returned to exact
  zero and a `READY` indication appears, its second press requests `LIVE`.
  Live begins only after the source acknowledges that same epoch and its
  canonical reference is joined.
- Loss of Xsens data does not request zero torque or PD brake by itself. After
  the first manual arm, an ordinary same-session network outage holds the last
  human reference inside `LIVE` and resumes automatically when a fresh
  10-frame window returns.
- A new manual arm is required only when the Xsens/adapter source epoch changes,
  including a detected MVN session restart; an ordinary network gap does not
  require another button press.
- The existing 0.4-second SONIC source blend is used for initial `idle -> live`,
  `live -> hold`, `hold -> recovered live`, `hold -> newly armed epoch`, and
  accepted manual-alignment transitions.
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
10. Hold the newest human pose as a static 10-frame reference while an already
    armed same-epoch source is stale, while continuing closed-loop SONIC
    inference from robot proprioception.
11. Preserve source timestamps and source epochs through the bridge so an
    ordinary gap can auto-resume while an observed new Xsens/MVN epoch cannot
    resume without manual re-arm, subject to the protocol limit in Section 28.
12. Provide clear transition and rejection diagnostics.
13. Preserve all existing PICO, ZeroLab, normal, PD-brake, zero-torque, and
    recovery paths.

## 4. Non-goals

The first release shall not:

- support TCP streaming;
- support Xsens messages other than `MXTP02`;
- support multiple actors, props, fingers, or non-FullBody segment layouts;
- import arbitrary MVNX files as a production control source;
- commit the user's MVNX recording or raw motion data to Git;
- use measured Xsens body dimensions to rescale the packaged SMPL skeleton;
- retarget fingers or control robot grippers;
- require a T-pose, N-pose, manual heading-reset calibration, or precise
  imitation of a picture;
- infer safety from human height, exact joint angles, or left/right symmetry;
- add automatic extrapolation to hide missing UDP frames;
- freeze one raw 29-DOF motor target as the stale-link holding strategy;
- add an automatic transition from `READY` to `LIVE`;
- switch directly among PICO, ZeroLab, and Xsens SONIC states;
- change SONIC model weights, model inputs, actuator ownership, or the hardware
  watchdog.

## 5. Considered architectures

### 5.1 Selected: Xsens pose source plus existing bridge

```text
MVN UDP :9763
  -> xsens_source
  -> ZMQ pose + xsens_status :5559
  -> existing pose_to_smpl_ref_bridge
  -> ZMQ smpl_ref + xsens_status :5557
  -> gated SonicTeleopPolicy

sonic_xsens
  -> ROS sonic/xsens_arm_command
  -> xsens_source
```

This follows the already deployed ZeroLab integration pattern, preserves the
existing packed-pose contract, and keeps Xsens parsing/conversion separate from
SONIC inference.

The generic bridge gains an explicitly configured Xsens mode: optional
transport metadata propagation, authoritative 10-row conversion, and immediate
`xsens_status` passthrough. Existing PICO/ZeroLab instances keep the default
merger/readiness behavior and ignore these additions.

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
  -> in parallel, immediate/heartbeat xsens_status
       ZMQ :5559 -> Xsens bridge passthrough -> ZMQ :5557
       epoch, readiness, progress, stale, arm-command acknowledgement
  -> SonicTeleopPolicy
       WAITING/READY: packaged idle reference
       LIVE/FRESH: Xsens reference
       LIVE/HOLD: last human pose tiled to a static 10-frame reference
       LIVE/HOLD_REARM_REQUIRED: old held pose; new epoch gated
       idle/live/hold/recovery transitions: 0.4 s smoothstep
  -> sonic.onnx or sonic.rknn
  -> existing 29-DOF target and ELF3 controller
```

The Xsens adapter never publishes motor commands. All actuator commands remain
inside the existing SONIC state and controller.

The only reverse control path is the targeted ROS arm command from
`sonic_xsens` to `xsens_source`; pose, status, and canonical-reference traffic
remain on the ZMQ paths shown above.

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

### 8.4 Counter and source-epoch handling

The uint32 sample counter is unwrapped only within one `source_epoch`. For
`delta = (new - previous) mod 2^32`:

| Delta | Classification |
|---:|---|
| `0` | duplicate |
| `1 .. 2^31-1` | forward, including natural wrap |
| `2^31` | ambiguous anomaly |
| `2^31+1 .. 2^32-1` | backward anomaly |

A forward gap is accepted and counted as missing source frames; it does not by
itself reset readiness or live control. A duplicate is ignored. The exact
half-range value is never guessed to be forward.

`source_epoch` identifies the observable adapter/MVN source session, not a gap
duration. Each adapter-process start draws a random nonzero 63-bit value stored
as `int64`. A committed upstream reset draws again until the value is nonzero
and differs from the current epoch. The generator is injectable for
deterministic zero/collision/retry tests. Cross-process uniqueness is
probabilistic, with a negligible 63-bit random collision risk; `MXTP02` provides
no persistent identifier to make it absolute.

The active sender identity is `(source IP, source UDP port, character ID)`.
`allowed_sender` filters the IP; the full tuple identifies the locked active
sender. The first parser-valid allowed packet locks the initial sender and
initializes its counter/time-code baselines. The old sender remains locked until
a session-reset candidate commits. For every later parser-valid packet,
classification follows this priority:

1. If sender matches, the sample counter is forward, and there is no meaningful
   time-code regression, the packet is active-compatible. It cancels any reset
   candidate and is accepted into the current epoch even when the previous
   packet was stale. This is the ordinary same-epoch recovery path.
2. A matching-sender duplicate is ignored and cancels any session-reset
   candidate, but it does not refresh producer freshness.
3. A backward/ambiguous counter or meaningful time-code regression from the
   matching sender opens a session-reset candidate. A changed sender may open
   one only after the active producer age is greater than 0.5 seconds.
4. Candidate packets never update the active epoch, active rolling window, or
   active producer timestamp.

A session-reset candidate commits only after `epoch_candidate_frames=2`
parser-valid packets from the same candidate sender arrive within
`epoch_candidate_timeout_s=0.25` and have strictly forward sample counters.
The second packet must be compared with the first candidate packet, but
active-compatible classification against the active baseline always takes
priority. A duplicate candidate packet does not advance it. An incompatible
candidate packet replaces the candidate and starts the two-frame proof again;
a malformed packet is rejected without advancing it; timeout clears it. Thus a
late `900` followed by active `1001` cannot be mistaken for a reset from active
`1000`, even though `1001` is forward relative to `900`.

On commit, the source atomically generates the new epoch, sets
`accepted_arm_epoch=0`, clears the last-command receipt fields plus active
unwrap/quaternion/readiness/output state, then replays both candidate packets
in receive order as the first two accepted frames of the new epoch. The set of
processed command IDs remains for replay rejection. The first unwrapped frame
index starts at its raw uint32 counter and the second advances by the modular
delta; their original receive timestamps are retained, and the second becomes
the newest producer timestamp.

Time code is diagnostic session evidence, never a freshness clock. Its state is
`UNKNOWN_OR_CONSTANT` until two nonzero active packets establish a forward
modular delta; it then becomes `ADVANCING`. While advancing, its last trusted
baseline changes only on an active-compatible forward/wrap value; duplicate
time code is allowed without moving the baseline. A backward/ambiguous delta,
including an implausible reset to zero but excluding a valid wrap to zero,
feeds the session-reset candidate and never overwrites the trusted baseline.
Candidate confirmation depends on sample-counter progression, not candidate
time-code progression. A time code that remains zero/constant is ignored;
constant-to-progressing changes mode without changing epoch.

A receive gap by itself never changes epoch, regardless of duration. Once a
candidate commits, ambiguity among MVN restart, playback seek, and sender-socket
replacement is resolved conservatively as a new epoch and manual re-arm. If a
real MVN restart preserves a forward counter, forward/ignored time code, sender
tuple, and adapter process, `MXTP02` cannot distinguish it from an ordinary gap
and it remains the same epoch.

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
- On the stale edge, the source clears the rolling output/recovery window and
  stops `pose` publication until ten later same-epoch frames have been
  accepted. The old canonical hold is owned downstream and is not cleared.
- On an epoch edge, the source additionally clears counter unwrapping,
  quaternion sign history, and readiness evidence. No published 10-row chunk
  may mix pre-edge rows, epochs, or old and recovered data.

Once a complete window exists, the source publishes it at the 50 Hz selection
rate even when `calibration_ready=false`. That flag reflects the current
readiness gate on every message, so motion or instability after `READY` is
observable immediately instead of leaving an old true value until timeout.

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

### 14.3 Arming and readiness while live

Before live enable, readiness reflects the current short motion window. The
state publishes a reliable, volatile `std_msgs/msg/Int64MultiArray` request on
relative topic `sonic/xsens_arm_command`. Its layout has `dim=[]` and
`data_offset=0`; `data` has exactly three signed-int64 values:

```text
[command_id, target_source_epoch, requested_arm_epoch]
```

- `command_id` is a freshly drawn nonzero positive 63-bit value, unique within
  the state process. The generator is injectable and redraws on zero or an
  already issued value.
- `target_source_epoch` is the nonzero epoch against which the command may act.
- `requested_arm_epoch=target_source_epoch` requests arming of exactly that
  epoch. `requested_arm_epoch=0` requests disarming only that target epoch.
- A nonempty layout, any other data length, nonpositive ID/target, or requested
  epoch other than zero or the target is rejected and cannot mutate
  authorization or readiness.

The source retains processed command IDs for its process lifetime. A duplicate
ID with the identical three-value payload is idempotent and does not reapply a
readiness reset; a reused ID with different fields is invalid. Every first-seen
structurally valid command, including a target-epoch mismatch, is echoed in the
status receipt fields defined in Section 17.3. A target mismatch never changes
authorization, readiness, or an output/recovery window.

An ordinary same-epoch network outage retains the matching accepted epoch,
letting the source build ten fresh motion frames and resume without a second
button press. The command topic is volatile rather than latched. A
source-process restart starts with `accepted_arm_epoch=0` and a new random
epoch. An upstream reset in the same process changes epoch atomically, so any
old command's target immediately mismatches. A delayed old arm or disarm
command therefore cannot authorize or clear readiness in a new epoch.

Command delivery is a request, not proof of arming. The source publishes the
atomic `xsens_status` contract in Section 17.3 independently of pose-window
publication. It accepts an arm request only when its target/requested epochs,
readiness, freshness, and complete window still match. The state records the
pre-command status sequence and never opens the policy gate until a newer
status echoes the exact command ID and both epochs, acknowledges
`accepted_arm_epoch=target_source_epoch`, and joins the corresponding canonical
reference.

A distinct, matching-target disarm request is an acknowledged barrier. Whether
or not `accepted_arm_epoch` was already zero, the source atomically sets it to
zero, clears all stability/readiness evidence, records the command receipt,
advances `status_sequence`, and publishes status before it can become ready
again. Only frames accepted after that command may populate the new 30-frame
candidate. This prevents a failed arm whose request was never accepted from
reusing readiness evidence that predates its cancellation.

After receiving the first current-epoch status on state entry, the state sends
one targeted disarm baseline. Each same-epoch arm/re-arm cancellation likewise
sends exactly one fresh-ID disarm request after recording the current status
sequence. The state remains `disarm_pending` and ignores readiness until a
later same-epoch status has a newer sequence, echoes that exact command ID and
target with requested epoch zero, and reports `accepted_arm_epoch=0`. The
source may already be accumulating its post-command candidate while the ACK is
in flight, but the state cannot return to `READY` until that candidate reaches
all 30 frames. Heartbeats make the correlated ACK observable; the state does
not retry with a new ID because each distinct disarm would restart the window.

A committed epoch change is itself an atomic disarm barrier: it sets the new
`source_epoch`, `accepted_arm_epoch=0`, and cleared command/readiness state,
then replays its two candidate packets. The first new-epoch status exposes that
unarmed state and the resulting post-replay readiness progress before any pose
window. The state confirms it but sends no redundant disarm. A disarm already
in flight for the old target epoch is rejected on arrival without clearing the
new window, so the replayed packets remain frames one and two. Commands are
therefore sent only for the entry baseline, same-epoch arm/re-arm attempts or
cancellations, and state exit.

## 15. One state, three internal phases

### 15.1 WAITING_FOR_DATA

This is the entry phase. The state:

- runs SONIC on the packaged idle reference starting at frame 3509;
- starts the state-scoped Xsens source and bridge;
- waits for the first current-epoch status, sends one targeted
  `[command_id, source_epoch, 0]` disarm baseline, and accepts readiness only
  after its correlated ACK or a later epoch's atomic disarm status;
- waits for a fresh, progressing, stable, complete reference;
- logs the current rejection reason without spamming each control tick.

Pressing `LT + RT + Y` in this phase does not enable live. The event is handled
and a message explains that the operator must wait for `READY`.

### 15.2 READY

The state enters `READY` only when all checks in Section 14.1 pass. The robot
still uses the packaged idle reference. The state logs:

```text
Xsens READY — release controls, then press LT+RT+Y to request LIVE
```

`READY` automatically returns to `WAITING_FOR_DATA` if the current status loses
readiness, freshness, its complete window, or its epoch. On the next
`btn_10=11` action that passes the exact-zero neutral latch, the state:

1. rechecks all Section 14.1 readiness conditions, source freshness, and source
   epoch; if any check has regressed, it rejects the press and returns to
   `WAITING_FOR_DATA`;
2. records the current status sequence and epoch, generates a fresh command ID,
   sends `[command_id, source_epoch, source_epoch]`, and remains in `READY` with
   `arm_pending=true`;
3. waits for the first newer status that reports the same `source_epoch`,
   echoes that command ID with both target/requested epochs equal to
   `source_epoch`, reports `accepted_arm_epoch=source_epoch`, and keeps
   ready/fresh/complete true; it then latches that ACK sequence and
   `newest_frame_index`, and later heartbeats do not move this join target;
4. waits for a canonical reference from that epoch whose
   `source_newest_frame_index` is at least the latched ACK frame;
5. only then binds `armed_source_epoch`, enables the policy's live-reference
   gate, resets yaw alignment so the newest Xsens root establishes heading, and
   changes the internal phase to `LIVE/FRESH`;
6. if readiness, freshness, status health, or the reference regresses within
   the same epoch before step 5, or the full ACK/reference join exceeds 0.5
   seconds, closes `arm_pending`, enters
   `WAITING_FOR_DATA/disarm_pending`, generates a fresh command ID, and sends
   `[command_id, source_epoch, 0]` once; only after a newer status echoes the
   exact command and acknowledges `accepted_arm_epoch=0` may the state honor
   the source's new post-command 30-frame readiness window;
7. if the epoch changes before step 5, closes `arm_pending`, accepts the new
   epoch's `accepted_arm_epoch=0` status as the disarm barrier without sending
   another disarm command, ignores any in-flight old-target command, and waits
   for that epoch's readiness window.

### 15.3 LIVE

The policy consumes Xsens `smpl_ref` windows. The existing 0.4-second
smoothstep blends the current SONIC joint target into the live-source target.
After this transition, normal operator movement is allowed and the entry
stillness test is disabled.

`LIVE` has link-status details, not additional framework phases:

```text
LIVE/FRESH
LIVE/HOLD
LIVE/HOLD_REARM_REQUIRED
```

An eligible `btn_10=11` event in `LIVE/FRESH` or `LIVE/HOLD` is phase-ignored,
but still closes the neutral latch. Only `LIVE/HOLD_REARM_REQUIRED` can use a
later event to re-arm, and only after another exact zero. The operator leaves
live control through the existing normal, PD-brake, zero-torque, or recovery
routes.

### 15.4 Ordinary same-epoch network outage

If the newest producer timestamp becomes older than 0.5 seconds but the source
epoch has not changed, the base phase remains `LIVE` and link status becomes
`HOLD`:

1. retain `accepted_arm_epoch=armed_source_epoch` without sending another arm
   command, so same-session data may resume automatically;
2. take the newest row of the last accepted live human reference;
3. tile its `term1_local`, `root_quat`, wrist, and optional anchor values into
   a static 10-frame hold reference;
4. continue running SONIC inference with that static human reference and live
   robot proprioception;
5. use the existing 0.4-second blend from the last live target toward the held
   reference target;
6. preserve the existing yaw alignment;
7. hold for as long as `sonic_xsens` remains active, unless data recovers or an
   existing operator exit/emergency route is used.

The adapter does not freeze one raw 29-DOF motor command. Continuing SONIC
inference lets the policy react to measured robot position, velocity,
orientation, angular velocity, and gravity while the requested human pose is
stationary.

The hold object is internal to the policy. It does not rewrite the last
`frame_index`, producer timestamp, or epoch and therefore cannot make stale
transport data appear fresh. On the stale edge the source clears only its
recovery/output window; the policy keeps the separately owned hold object.

When ten strictly increasing, valid, fresh frames return with the same source
epoch:

1. require all ten frames to have been received after the stale edge and to
   belong to the armed epoch; forward counter gaps are allowed, but old rows are
   not;
2. latch the first status reporting that armed epoch,
   `accepted_arm_epoch=armed_source_epoch`, `source_stale=false`,
   `recovery_frames=10`, and `reference_window_ready=true`;
3. wait for the same-epoch canonical reference whose
   `source_newest_frame_index` reaches the latched recovery frame, without a
   stillness requirement;
4. change link status from `HOLD` to `FRESH` automatically;
5. preserve yaw alignment;
6. blend from the current held-reference target to the recovered live target
   over 0.4 seconds.

No `LT + RT + Y` press is required for ordinary network recovery, regardless
of how long the same-epoch gap lasted.

### 15.5 Source/MVN session restart

A changed `source_epoch` means the adapter process restarted or an observable
upstream MVN session reset was detected. If the state was `FRESH`, it first
creates the same static hold object from the last old-epoch row. If it was
already `HOLD`, it keeps that object. Automatic resume is disabled:

1. change link status to `LIVE/HOLD_REARM_REQUIRED`;
2. confirm that the new-epoch status reports `accepted_arm_epoch=0`; treat the
   epoch commit/startup itself as the disarm barrier and do not send a redundant
   disarm command that would discard its first readiness frames;
3. keep running SONIC against the old static hold reference;
4. require the new source to pass the loose readiness gate and build a complete
   window;
5. remain in base phase `LIVE` and expose whether the newest pending epoch is
   still collecting readiness evidence or is re-arm ready; re-arm readiness
   automatically clears again if any readiness condition regresses;
6. log the new-session READY prompt only after all Section 14.1 checks pass;
7. reject an early `btn_10=11` event with the current readiness reason;
8. on a new neutral-latched `btn_10=11` event, recheck every readiness
   condition, freshness, and epoch; record the status sequence, generate a
   fresh command ID, send `[command_id, pending_epoch, pending_epoch]`, and keep
   the old hold active with `rearm_pending=true`;
9. require the same post-command status acknowledgement and matching canonical
   reference as initial arm; only then bind the epoch, reset yaw alignment, and
   start the 0.4-second blend to its live target;
10. on a same-epoch regression or 0.5-second ACK/reference-join timeout, cancel
    pending re-arm, enter `disarm_pending`, generate a fresh command ID, send
    `[command_id, pending_epoch, 0]` once, and continue using the old hold; the
    state ignores the freshly reset post-command readiness candidate until a
    newer status echoes that exact command and reports
    `accepted_arm_epoch=0`;
11. if another epoch appears while re-arm is pending, cancel the pending join,
    retain the old hold, and accept the newest epoch's
    `accepted_arm_epoch=0` status as the disarm barrier without sending another
    disarm command; any in-flight command targeted at an older epoch is rejected
    without clearing the newest epoch's readiness frames.

The re-arm-ready log text is:

```text
Xsens new session READY — release controls, then press LT+RT+Y to re-arm
```

If another epoch appears before re-arm, its readiness/window evidence is reset
again, the original old-epoch hold remains active, and only the newest epoch may
be armed.

If no reference has ever reached `LIVE`, there is nothing to hold; initial
`WAITING_FOR_DATA` and `READY` continue to use the packaged idle reference.

## 16. Policy gating and entry smoothing

The shared `SonicTeleopPolicy` gains a live-reference enable flag, an
`armed_source_epoch`, and an optional held-reference path. It receives both
canonical-reference and source-status snapshots while disabled so
`sonic_xsens` can detect initial/re-arm readiness and command acknowledgement.
An unacknowledged epoch cannot select live data. The joined observable snapshot
contains status sequence/health/reason/progress, source and accepted-arm epochs,
the latest command ID/target/request receipt, producer time, newest frame index,
window readiness, and the canonical reference. This joined snapshot is the
single source of truth for state phase decisions.

On every `sonic_xsens` control update, phase logic polls and validates producer
freshness and source epoch before policy inference:

- fresh, armed, matching-epoch input selects the current live reference only
  after the required complete post-edge window exists;
- stale input with the armed epoch selects the static held reference;
- a new epoch cannot be consumed until another manual arm.

The held reference is created once on the `FRESH -> HOLD` edge from the newest
row of the last valid reference, or equivalently on a direct `FRESH ->
HOLD_REARM_REQUIRED` edge. The old hold is cleared only after the full
0.4-second recovery/re-arm blend finishes successfully, or on state exit/full
policy reset. Here, full policy reset does not mean the operator's
`reset_alignment` action. If input fails again during a blend, the old hold
remains available.

Yaw context follows the same ownership. The policy stores
`active_yaw_offset`, captures `hold_yaw_offset` with the old hold, and computes
a separate `pending_yaw_offset` for a newly acknowledged epoch. During a re-arm
blend, old-hold inference always uses `hold_yaw_offset` while the destination
uses `pending_yaw_offset`. The pending offset becomes active and the old hold is
cleared only when the full blend succeeds.

Every source transition captures the currently commanded blended SONIC target
as its next blend start. If input becomes stale or changes epoch during a
recovery/re-arm blend, that blend is cancelled, the old hold remains, and a new
0.4-second blend starts from the current commanded target toward the hold
target. A later recovery again starts from the then-current commanded target.
For a same-epoch stale/status interruption during re-arm blend, the pending
offset remains bound to that armed epoch for automatic recovery, but the hold
target is evaluated with `hold_yaw_offset`. A further epoch change discards the
pending offset. Preemption therefore never jumps back to an earlier blend
origin or silently reinterprets the old hold under a new heading.

Existing PICO and ZeroLab states configure the flag enabled and retain their
current automatic live-reference and idle-fallback behavior. The hold/epoch
rules are enabled only for `sonic_xsens`.

The Xsens state also seeds its initial SONIC target from the current measured
robot joint frame before the first idle inference. This prevents the existing
20 ms `soft_switch` hold from ending at an unrelated SONIC default target. The
entry seed is Xsens-state-specific; it does not change existing PICO or ZeroLab
entry behavior.

Two smoothing mechanisms remain distinct:

- `soft_switch`: holds the previous motor frame for 20 ms during framework
  state entry/exit;
- `source_blend_seconds=0.4`: smoothstep interpolation of SONIC's 29 target
  joints for initial idle/live, live/hold, hold/recovered-live, and newly armed
  epoch transitions.

Each transition edge starts exactly one blend; repeated transport publication
does not restart it. Hold transitions do not call the existing generic
live/idle yaw reset. Yaw is reset explicitly only on the first arm and a
new-epoch re-arm; `FRESH -> HOLD -> FRESH` preserves the same yaw offset.
This restriction concerns automatic transitions only.

The existing operator-requested `reset_alignment` action remains available but
has explicit Xsens phase semantics. It never changes phase, epoch authorization,
readiness, or the held canonical reference:

- in `WAITING_FOR_DATA`, `READY`, `arm_pending`, or `rearm_pending`, the state
  handles but rejects the action with a rate-limited diagnostic and changes no
  yaw context;
- while any 0.4-second source or alignment blend is active, the state likewise
  rejects it, so one transition cannot rewrite another transition's source and
  destination yaw contexts;
- while `pending_yaw_offset` is retained after a same-epoch interruption of a
  re-arm blend, the state rejects it even after the blend back to hold has
  completed; successful automatic recovery must first promote that pending yaw,
  or a later epoch must discard it;
- in stable `LIVE/FRESH`, it recaptures `active_yaw_offset` from the newest
  authorized reference and current robot anchor;
- in stable same-epoch `LIVE/HOLD`, it recaptures from the static held reference
  and writes the result to both `hold_yaw_offset` and `active_yaw_offset`, so
  automatic recovery preserves the requested alignment;
- in stable `LIVE/HOLD_REARM_REQUIRED`, it recaptures only
  `hold_yaw_offset`; a later new-epoch re-arm still computes its independent
  `pending_yaw_offset` from the new reference.

Each accepted manual reset captures the current commanded target, starts one
0.4-second smoothstep blend toward the currently selected fresh or held
reference under the replacement offset, and rejects further reset actions
until that blend completes. A stale or epoch edge preempts this alignment blend
under the normal source-transition rule: capture the then-current commanded
target and blend toward the appropriate hold without reverting to the earlier
blend origin. Existing PICO and ZeroLab `reset_alignment` behavior is
unchanged.

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
| `source_epoch` | `int64 [1]` | nonzero adapter/upstream session generation |

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
producer_monotonic_ns     int64 [1]
source_epoch              int64 [1]
source_newest_frame_index int64 [1]  # exact input frame_index[-1]
```

The policy uses the producer timestamp for Xsens freshness instead of treating
each bridge re-publication as a new source frame. Sources without this optional
field continue to use local policy-receive time.

For Xsens metadata, the bridge treats an exact structurally validated 10-row
chunk as the authoritative window. It forwards
`source_ready=calibration_ready` even when false, clears its Xsens accumulator
on a producer gap over 0.5 seconds or an epoch change, and does not add the
existing three-progressing-message debounce after the source has already
supplied the complete post-edge window. This makes the recovery boundary
exactly ten new source frames while leaving PICO and ZeroLab readiness behavior
unchanged.

`authoritative_input_window=true` also bypasses the generic gap-merger/clamping
path: the bridge converts the ten input rows one-for-one, in order, and never
mixes them with a prior message. The legacy scalar `frame_index` keeps its
existing meaning for other sources; Xsens status/reference joining uses only
the explicit `source_newest_frame_index`.

### 17.3 Immediate Xsens status and arm acknowledgement

The source also publishes packed topic `xsens_status` on port 5559. The
Xsens-configured bridge forwards it unchanged on port 5557 immediately and
independently of `pose`/`smpl_ref` windows:

| Field | Dtype and shape | Meaning |
|---|---|---|
| `status_sequence` | `int64 [1]` | strictly increasing in one source process |
| `status_monotonic_ns` | `int64 [1]` | status production time |
| `source_epoch` | `int64 [1]` | current observable source session |
| `last_arm_command_id` | `int64 [1]` | most recent first-seen valid command ID, or zero |
| `last_arm_target_epoch` | `int64 [1]` | target carried by that command, or zero |
| `last_requested_arm_epoch` | `int64 [1]` | requested epoch carried by that command, or zero |
| `accepted_arm_epoch` | `int64 [1]` | zero or the currently accepted epoch |
| `producer_monotonic_ns` | `int64 [1]` | newest accepted UDP receive time, or zero |
| `newest_frame_index` | `int64 [1]` | newest epoch-local frame, or -1 |
| `ready` | `bool [1]` | current epoch is eligible under the arming rule below |
| `reference_window_ready` | `bool [1]` | complete same-epoch 10-row window exists |
| `source_stale` | `bool [1]` | producer age is greater than 0.5 seconds |
| `ready_frames` | `int32 [1]` | readiness progress, 0 through 30 |
| `recovery_frames` | `int32 [1]` | post-stale recovery progress, 0 through 10 |
| `reason_code` | `int32 [1]` | current readiness/link reason enum |

While unarmed, `ready=true` means every Section 14.1 check passes. With an
an accepted arm command matching the current epoch, only the stillness check is
bypassed; packet validity, freshness, epoch match, and the complete reference
window remain mandatory.

The reason enum is stable:

| Value | Name |
|---:|---|
| 0 | `NO_DATA` |
| 1 | `COLLECTING_WINDOW` |
| 2 | `COLLECTING_STABILITY` |
| 3 | `PELVIS_UNSTABLE` |
| 4 | `SEGMENT_UNSTABLE` |
| 5 | `READY` |
| 6 | `ARMED_FRESH` |
| 7 | `ARMED_STALE` |
| 8 | `ARMED_RECOVERING` |
| 9 | `ARM_COMMAND_MISMATCH` |
| 10 | `SESSION_RESET` |
| 11 | `INVALID_INPUT` |

Status publishes on source start, epoch/stale/readiness/recovery changes, every
first-seen arm-command receipt, and every 50 Hz source tick as a heartbeat. An
identical duplicate command does not mutate state, but the current echoed
receipt remains visible in heartbeats if it is still the latest command. Epoch
change and command invalidation are reflected in one atomic status snapshot
before any new-epoch pose window can publish. The bridge branch is selected
explicitly by `source_kind: xsens` and exact topic name, not inferred merely
from optional metadata fields.

## 18. Ports and lifecycle ownership

| Purpose | Port |
|---|---:|
| MVN Xsens UDP input | 9763 |
| existing PICO pose ZMQ | 5556 |
| existing ZeroLab pose ZMQ | 5558 |
| new Xsens pose/status ZMQ | 5559 |
| shared SONIC `smpl_ref`/Xsens-status ZMQ | 5557 |

`xsens_source` is the sole owner of UDP 9763 and ZMQ 5559 while
`sonic_xsens` is active. `xsens_bridge` is the sole owner of ZMQ 5557 in that
state. All sockets use zero linger and close deterministically on state exit.

Entry to, and switching among, PICO, ZeroLab, and Xsens live-source states goes
through basic `normal`. There are no direct routes among those three states
because the framework may start target-state nodes before old-state nodes have
released port 5557. Existing normal, recovery, PD-brake, and zero-torque exits
remain available.

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

The exact-zero latch also makes one minimal framework addition:

```text
framework/runtime/state_machine.py  # latest value per remote slot
framework/runtime/controller.py     # read-only remote_slot_value(slot)
framework/mod_api/context.py        # expose that method to states
```

This is a level snapshot, not a queued action. It prevents a `btn_10=0` release
that arrives during the 20 ms soft-switch from being lost before
`sonic_xsens.on_enter()`.

The development-only keyboard binding makes one small remote-controller schema
addition:

```text
remote_controller/config.hpp      # optional requires_driver_filter on binding
remote_controller/config.cpp      # filter binding during config load
remote_controller/main.cpp        # pass explicit CLI driver filter to loader
config/xbox_default.yaml           # physical binding plus gated keyboard binding
```

Responsibilities:

- `protocol.py`: immutable wire structures and `MXTP02` parsing only;
- `udp_receiver.py`: nonblocking socket ownership, timestamps, sender filter,
  and receive/drop statistics;
- `converter.py`: quaternion/basis conversion, segment mapping, fixed FK, and
  wrist derivation as pure functions/state;
- `source_core.py`: counter unwrapping, readiness, 60-to-50 selection,
  source epoch, staleness, arm-command validation/status, and the 10-frame
  window;
- `source_node.py`: ROS/ZMQ lifecycle, targeted arm-command subscription, and
  `pose`/`xsens_status` publication.

The state configuration is:

```text
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
```

The source configuration is:

```text
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

The Xsens bridge instance is configured explicitly:

```text
source_kind: xsens
input_pose_topic: pose
input_status_topic: xsens_status
output_reference_topic: smpl_ref
output_status_topic: xsens_status
authoritative_input_window: true
readiness_debounce_messages: 1
```

`status_timeout_s` applies only to the local source/bridge heartbeat, not to
MVN UDP packet spacing. A missing local status while live selects hold; it
cannot make an unacknowledged reference live.

## 20. Remote and keyboard event mapping

`xbox_default.yaml` adds two same-value bindings:

```text
btn_10=11 when:
  - LT pressed
  - RT pressed
  - Y pressed
  - LB released
  - RB released

btn_10=11 when:
  - keyboard r
  requires_driver_filter: keyboard
```

The optional `requires_driver_filter` binding is loaded only when the process
was explicitly started with `--keyboard` or `--driver keyboard`. The default
hardware launch supplies no such filter, so this binding is absent even if
device selection later falls back to the keyboard. The physical binding remains
available to gamepad and CRSF. Same slot/value bindings are valid and do not
conflict.

Physical triggers must cross their configured pressed thresholds. Existing
single-trigger combinations require the opposite trigger released, so they do
not collide with this dual-trigger rule.

This combination also works with the current CRSF channel model: LT and RT are
independent channels and Y is in button group A. `LB + RB` was not selected
because both shoulder buttons share one CRSF enum channel and cannot be
represented simultaneously there.

The framework emits an action when the mapped value changes into
`btn_10=11`; that alone is not a strict physical-release guarantee.
`sonic_xsens` therefore keeps a state-local neutral latch. After state entry
and after every accepted or rejected arm attempt, the state must observe an
exact `btn_10=0` before another `btn_10=11` action can be accepted.
Transitions such as `11 -> 2 -> 11` do not satisfy the latch. Holding the
combination while the first press enters the state cannot also enable `LIVE`.
Every `activate_xsens` action inside the state consumes/closes the latch,
including phase-ignored presses in `LIVE/FRESH` or `LIVE/HOLD`. State reset/exit
clears the latch; a reused state object initializes it again from the current
slot snapshot on entry.

`RemoteEventAdapter` stores the latest level once per physical slot, independent
of which state's route currently receives its edge events. On state entry and
each update, `sonic_xsens` reads `remote_slot_value("btn_10")` and sets the
latch only when that level is exactly zero. Thus a release received in
`normal` or during soft-switch is visible at Xsens state entry. `mod.yaml` needs
only the `activate_xsens: btn_10=11` event: from `normal` it routes into the
state, and inside `sonic_xsens` it invokes arm/re-arm after the latch check.
Reading an undeclared slot raises an error rather than returning a false neutral
zero.

The keyboard mapping is a non-safety-rated MuJoCo/development convenience. Its
terminal driver has synthetic key expiry rather than true key-up events, so
held-key and OS auto-repeat behavior is explicitly unsupported and is not used
as proof of two intentional presses. Physical-robot arming uses the default
launch, in which only controller/CRSF can generate this event.

## 21. Error handling and network behavior

- Wrong UDP length/identifier/counts: reject the whole packet and count it.
- Missing/duplicate segment ID: reject the whole packet and count it.
- Non-finite value or invalid quaternion norm: reject the whole packet.
- Duplicate/out-of-order sample: ignore and count; do not regress the window or
  change epoch from one isolated packet.
- Forward sample gap: accept the new frame and count missing samples.
- Missing/old local `xsens_status` for more than 0.2 seconds keeps an unarmed
  state waiting; after arm it selects `LIVE/HOLD` but does not by itself invent
  a new epoch or require manual re-arm. A healthy same-epoch status ACK plus a
  matching current canonical window resumes automatically with a 0.4-second
  blend; this local-status case does not require a new ten-frame UDP window if
  the source never declared UDP stale.
- Receive age up to 0.5 seconds: retain the last accepted reference and do not
  change phase/link status because of one ordinary delayed packet.
- Receive age over 0.5 seconds before the first arm: clear readiness/output
  windows, remain or return to `WAITING_FOR_DATA`, and continue using idle.
- Receive age over 0.5 seconds after arm with the same epoch: remain in
  `LIVE/HOLD`, clear only the recovery/output window, retain the matching
  accepted arm epoch, and run the static last-human-pose reference. Zero
  through nine recovered frames remain `HOLD`; the tenth valid same-epoch frame
  plus its matching status/canonical join starts exactly one automatic
  0.4-second recovery blend.
- Source epoch change before the first arm: clear readiness and wait for the new
  session. Source epoch change after arm: disarm atomically, enter
  `LIVE/HOLD_REARM_REQUIRED`, keep the old human hold reference, and refuse the
  new epoch until it passes readiness and a new eligible manual arm.
- An arm command never opens the policy gate by itself. Within its target
  epoch, a negative/mismatched correlated ACK, readiness/reference regression,
  or 0.5-second ACK/reference-join timeout cancels pending arm and sends one
  fresh-ID `[command_id, target_epoch, 0]` request. An epoch change cancels the
  join but uses the new epoch's atomic zero authorization status; an old-target
  request still in flight is rejected without resetting the new epoch.
- ZMQ bind failure: fail node preparation visibly; never choose another port.
- Conversion/FK failure: reject the frame and never publish partial arrays.
- Repeated invalid data eventually becomes stale through the same timeout.
- State exit: if a current source epoch is known, send one fresh-ID targeted
  disarm request, then close UDP/ZMQ resources deterministically without
  waiting for its ACK.

Neither stale path requests PD brake or zero torque. An armed Xsens session
continues SONIC closed-loop inference from the held human reference and current
robot proprioception; it does not select packaged idle and does not freeze a raw
motor target. Existing operator emergency controls and the hardware command
watchdog remain available and unchanged.

## 22. Diagnostics

Logs are emitted only on state/reason changes and periodic summaries, not once
per packet. They include:

- source bind address and expected protocol;
- sender address;
- received, accepted, malformed, duplicate, out-of-order, and inferred-drop
  counters;
- measured packet rate and newest source age;
- readiness progress, same-epoch recovery progress `0..10`, re-arm readiness,
  and the active rejection reason;
- base phase and link status: `WAITING_FOR_DATA`, `READY`, `LIVE/FRESH`,
  `LIVE/HOLD`, or `LIVE/HOLD_REARM_REQUIRED`;
- status sequence/age/reason, current source epoch, requested/accepted/armed
  epochs, latest command ID/target receipt, newest joined frame index, and
  locked sender identity;
- manual enable/re-arm pending, acknowledged, timed out, or rejected, including
  neutral-latch rejection;
- stale, hold, source-epoch, automatic-resume, and re-arm events;
- one-shot blend start/completion and whether yaw was preserved or reset;
- ZMQ publication drops and lifecycle shutdown.

The phase prompt tells the operator exactly when the initial or re-arm control
press is allowed. Diagnostic logs never print full motion frames.

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

- Accept normal increments and uint32 wrap-around using the half-range rule,
  including a natural wrap immediately after a stale gap.
- Classify an exact `2^31` sample/time-code delta as ambiguous, never forward.
- Ignore duplicate, out-of-order, and isolated late post-stale samples.
- Accept forward gaps and report the missing count.
- Use monotonic receive time rather than Xsens time code for freshness.
- Inject epoch draws `0, current, new` and prove generation retries until a
  nonzero/different value; adapter restart uses a new epoch even when the Xsens
  counter continues.
- Prove stale plus a matching-sender forward packet cancels any candidate and
  becomes same-epoch recovery frame one.
- With active counter `1000`, prove anomaly `900` followed by active-compatible
  `1001` cancels the candidate and accepts `1001` into the active epoch.
- Require two coherent session-reset-candidate packets before a backward or
  ambiguous counter, advancing-time-code regression, or eligible changed
  sender changes epoch.
- Cover candidate duplicate, incompatible replacement, malformed packet,
  active-compatible cancellation, and 0.25-second timeout.
- Prove candidate packets never refresh active freshness or enter its window.
- Prove committed candidate frames become new-epoch frames one and two, retain
  receive timestamps, and count toward both 10-frame output and 30-frame
  readiness windows; observing the new epoch's atomic unarmed status does not
  make the state send a redundant disarm or discard those frames.
- Do not treat time-code natural wrap, initial zero/constant, or
  constant-to-progressing as a restart; do treat advancing-to-non-wrap-zero then
  small-positive progression as a candidate sequence.
- Treat a coherent sender-port replacement after stale as a new epoch while
  continuing to filter the configured sender IP; the same replacement while
  active is not eligible to take over.
- Document in a test that a simulated restart with forward counter/time code and
  unchanged sender is unobservable and therefore remains the same epoch.
- Clear unwrap, quaternion, readiness, and output-window session data on an
  epoch change.
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
- A counter gap does not reset readiness.
- With a fake monotonic clock, `0.5-epsilon` and exactly `0.5` seconds remain
  fresh; `0.5+epsilon` resets readiness before arm/new epoch, while an armed
  same-epoch source instead clears only the recovery/output window.
- A valid arm command whose target and requested epochs both equal
  `source_epoch` disables the stillness requirement but not validity,
  freshness, or complete-window checks; disarm or mismatched-target commands
  cannot.
- A delayed old-target arm command after source restart/reset is rejected and
  cannot make the new epoch ready; ordinary same-epoch stale/recovery retains
  its matching accepted authorization without another command.
- After an armed stale edge, zero through nine new frames produce no live
  window; the tenth strictly increasing same-epoch frame produces one complete
  window containing only post-edge rows.
- 60 Hz input produces progressing newest-frame publication at 50 Hz without
  duplicate frame indices.

### 24.5 Contract and bridge tests

- Every pose field has the exact dtype and shape.
- Every `xsens_status` field/reason has the exact dtype, shape, sentinel, and
  range, including all three command-receipt echoes; status sequence increases
  within a process.
- Reject a nonempty arm-command layout, malformed data lengths, nonpositive
  command IDs/targets, and a requested epoch other than zero or the target.
  Inject command-ID draws zero, an already issued value, and a fresh value to
  prove deterministic redraw.
- Existing packed-message decoding accepts Xsens output.
- The bridge produces exact `term1_local [10,72]`, `root_quat [10,4]`, and
  `wrist [10,6]`.
- Producer timestamp, source epoch, and exact input `frame_index[-1]` as
  `source_newest_frame_index` survive the bridge unchanged.
- A repeated bridge publication cannot extend producer freshness.
- The Xsens bridge accumulator resets on stale/epoch edges and never mixes rows
  across them.
- The first structurally valid post-edge 10-row Xsens chunk is publishable
  without the generic three-message debounce.
- Authoritative Xsens input bypasses merger/fill/clamping one-for-one, while the
  legacy scalar `frame_index` behavior remains unchanged for other sources.
- A current `calibration_ready=false` becomes `source_ready=false` immediately,
  so old readiness cannot survive operator motion until freshness timeout.
- Explicit `source_kind: xsens` selects the authoritative-window/status fast
  path; optional metadata alone does not.
- Status forwards without a pose window, and an epoch/command invalidation
  reaches policy status before the first new-epoch `smpl_ref`.
- Source start/change/heartbeat/first-seen command receipt produces status;
  bridge passthrough preserves every field and does not apply pose debounce.
- A distinct matching-target disarm, including one received while already
  unarmed, echoes its command ID and both epoch fields, atomically reports
  `accepted_arm_epoch=0`, clears all readiness evidence, and cannot report
  ready again before 30 subsequently accepted stable frames.
- Inject a heartbeat already in flight with `accepted_arm_epoch=0` but an older
  command ID; it cannot acknowledge a newer cancellation or release
  `disarm_pending`.
- An exact duplicate command ID/payload is idempotent and does not restart the
  30-frame window; reuse of an ID with different fields is rejected.
- Commit a new epoch while an old-target disarm is in flight, then deliver that
  command. Its target mismatch is echoed/rejected without clearing the two
  replayed candidate frames or either new-epoch window.
- Existing PICO and ZeroLab messages without optional metadata remain accepted
  with unchanged freshness semantics.

### 24.6 State and policy tests

- `sonic_xsens` can be entered without pre-existing UDP data.
- State entry waits for a current epoch, sends one targeted disarm baseline,
  and cannot honor readiness from an older command receipt; an epoch change
  before the baseline ACK substitutes its atomic unarmed status without a
  second disarm.
- First `btn_10=11` from normal enters the state; `11 -> 11` and `11 -> 2 ->
  11` cannot arm it, while an observed `11 -> 0 -> 11` can.
- A release to zero received during the 20 ms normal-to-Xsens soft-switch is
  retained in the slot-level snapshot and initializes the latch on state entry;
  a held `11` or any other nonzero level does not.
- `WAITING_FOR_DATA` uses idle and rejects a live-enable action.
- All readiness checks change waiting to `READY` while the policy still uses
  idle.
- Motion, staleness, or epoch regression after `READY` makes an arm attempt fail
  and return to waiting; the failed attempt closes the latch until another
  exact zero.
- A second neutral-latched `btn_10=11` sends a fresh-ID command targeted at and
  requesting the current epoch but remains `READY/arm_pending`; cached
  reference/status alone cannot change it to live.
- A newer status echoing the exact command ID, target, and request with matching
  accepted arm epoch, plus a same-epoch canonical reference at least as new by
  `source_newest_frame_index`, changes pending to `LIVE/FRESH`, resets yaw, and
  starts one 0.4-second source blend.
- Epoch change between button recheck and command receipt rejects the old-target
  command; policy never consumes cached old READY data, no false LIVE is
  reported, and the state accepts the new epoch's
  `accepted_arm_epoch=0` status without sending a redundant disarm.
- Readiness regression or a 0.5-second ACK/reference-join timeout cancels
  pending arm, enters disarm-pending, and requires both a newer status echoing
  the exact cancellation command with `accepted_arm_epoch=0` and another
  30-frame readiness window before another exact-zero-latched attempt.
- If the source ACKs arm just before cancellation, cached/readiness-bypassed
  status cannot advertise READY while disarm is in flight. Exactly one fresh-ID
  disarm is sent; its correlated receipt resets the 30-frame candidate, and a
  later matching status/heartbeat makes completion observable without a retry
  loop.
- If cancellation occurs while `accepted_arm_epoch` is already zero, an
  in-flight older unarmed heartbeat cannot ACK it. The exact fresh command must
  be echoed, and only 30 frames received after its source-side receipt may make
  the state ready again.
- Entry target seeding starts from the current measured robot joint frame.
- Same-epoch stale input changes `LIVE/FRESH` to `LIVE/HOLD`, tiles exactly the
  newest `term1_local`, `root_quat`, wrist, and optional anchor row ten times,
  and preserves the armed epoch and yaw offset.
- During `HOLD`, inference runs every control tick; changing robot
  proprioception changes the constructed observation while the canonical human
  reference remains static. No raw 29-DOF target is frozen.
- `FRESH -> HOLD` starts exactly one 0.4-second blend. Zero through nine
  recovered frames remain `HOLD`; frame ten alone is insufficient until its
  recovery-complete status and canonical frame index join, which then starts
  exactly one automatic `HOLD -> FRESH` blend without a button press or yaw
  reset.
- Repeated identical references do not restart a blend. A second stale edge
  during recovery cancels the blend, captures the current commanded target as
  the new start, and preserves the old hold until a later blend completes.
- A changed epoch atomically invalidates the accepted arm epoch, enters
  `LIVE/HOLD_REARM_REQUIRED`, keeps the old hold, and refuses all new-epoch
  references while collecting readiness.
- An early re-arm press is rejected and requires a new exact-zero latch.
  Thirty stable frames make the newest epoch re-arm ready; an accepted press
  remains on hold until a post-command status ACK plus matching canonical
  window binds that epoch, resets yaw, and starts one 0.4-second blend.
- A stale/mismatched ACK, local 0.2-second status-heartbeat timeout, or
  0.5-second ACK/reference-join timeout never opens the policy gate.
- Fake-clock boundaries keep exact 0.2/0.5 seconds valid and time out only
  above the configured status/ACK thresholds.
- A local status timeout while live selects hold; healthy same-epoch status plus
  its matching current canonical window auto-resumes without manual re-arm or
  a fabricated UDP recovery count.
- A second epoch change before re-arm resets pending-epoch readiness but does not
  replace the old hold.
- With deliberately different old/new yaw offsets, a same-epoch interruption
  during re-arm blend evaluates the old hold only with its saved hold yaw,
  retains the pending yaw only for that armed epoch, and resumes from the
  current commanded target; another epoch discards the pending yaw.
- A phase-ignored press in `LIVE/FRESH` or `LIVE/HOLD` closes the latch; a later
  epoch change plus `11 -> 2 -> 11` cannot re-arm until an exact zero occurs.
- Exit/re-entry resets latch state and initializes it only from the latest slot
  level, never from the previous Xsens session.
- Stable `LIVE/FRESH` manual alignment reset changes only the active yaw and
  starts one 0.4-second blend from the current commanded target.
- Stable same-epoch `LIVE/HOLD` manual reset updates both active and hold yaw;
  later automatic recovery preserves that replacement offset.
- Stable `LIVE/HOLD_REARM_REQUIRED` manual reset changes only the old hold yaw;
  later re-arm computes an independent pending yaw for the new epoch.
- `reset_alignment` in waiting/ready, arm/re-arm pending, or any active source
  or alignment blend is rejected without changing a yaw context or restarting
  a blend. A stale/epoch edge during an accepted alignment blend preempts from
  the current commanded target.
- After a same-epoch interruption preserves `pending_yaw_offset`, a stable-hold
  reset is still rejected until automatic recovery promotes that pending yaw;
  the reset cannot overwrite active/hold while leaving a conflicting pending
  destination.
- No stale condition requests PD brake or zero torque.
- Existing normal, recover, PD-brake, and zero-torque routes still work.
- Existing PICO and ZeroLab states retain automatic reference behavior.

### 24.7 Mapping and lifecycle tests

- `LT + RT + Y` emits `btn_10=11` and physical release emits exact zero for
  gamepad and CRSF mappings.
- Under the default hardware launch, keyboard fallback plus `r` remains
  `btn_10=0` because the development binding was not loaded.
- Under explicit `--keyboard`/`--driver keyboard`, one `r` byte emits
  `btn_10=11` followed by synthetic zero; held/auto-repeat behavior is
  unsupported rather than treated as an intentional-arm guarantee.
- The optional binding filter parses, rejects unknown filter names, and does not
  alter any existing ungated mapping.
- The new rule does not emit existing single-trigger actions.
- The framework level snapshot updates even when the current state has no rule
  for that edge, survives a transition handoff, and rejects unknown slot names.
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

5. Verify `btn_10=11` and full-release `btn_10=0` with
   `ros2 topic echo /motion_commands` for both gamepad and CRSF.
6. Run the complete chain in MuJoCo: waiting, ready, manual live, normal exit,
   then two separate fault cases:
   - interrupt UDP without restarting the adapter and verify same-epoch hold
     plus button-free recovery;
   - restart MVN/adapter to produce a new epoch and verify hold, new readiness,
     and manual re-arm.
7. Run the physical robot first with a safety operator at PD brake, a second
   operator wearing Xsens, limited motion amplitude, and clear surroundings.
8. Expand motions only after yaw, left/right limbs, wrists, feet, same-epoch
   hold/recovery, and new-epoch re-arm have been observed correctly.

## 26. Operator procedure

```text
1. Start MVN Analyze and confirm Network Streamer configuration.
2. Start playback/live capture and confirm MXTP02 packets reach UDP 9763.
3. Put the robot in pd_brake, then enter normal using the existing controls.
4. Stand approximately in the recommended neutral/image posture and remain
   briefly still; exact imitation is not required.
5. Press LT+RT+Y once, then release, to enter sonic_xsens.
6. Continue holding the approximate posture after state entry. Confirm the
   robot remains on SONIC idle and wait for the READY log.
7. Press LT+RT+Y a second time, then release, to request LIVE. Confirm the
   accepted-arm/LIVE/FRESH log; a button press without acknowledgement is not
   live.
8. Wait through the 0.4-second blend and begin with small, slow movements.
9. For an ordinary same-session outage, stop moving and observe LIVE/HOLD. Do
   not press the arm control. When ten same-epoch frames return, observe the
   automatic 0.4-second blend back to LIVE/FRESH before moving again.
10. If the log reports LIVE/HOLD_REARM_REQUIRED, keep the approximate posture
    until the new-session READY prompt, ensure the control has returned to exact
    zero, then press LT+RT+Y once. Wait for its accepted-arm log; this resets
    yaw for the new epoch and blends from the old held pose over 0.4 seconds.
11. Use the existing normal/recovery/PD-brake/zero-torque controls to leave or
    stop.
```

Only an explicit `--keyboard`/`--driver keyboard` development launch loads
`r`. Two intentional taps follow the same initial behavior; held/auto-repeat
behavior is unsupported. The default physical-robot launch does not load this
binding.

## 27. Acceptance criteria

The first release is accepted when:

1. All new and existing relevant automated tests pass.
2. Real `MXTP02` packets parse continuously at approximately 60 Hz with the
   verified 760-byte/23-segment structure.
3. Synthetic and real quaternion sign flips do not create converted-pose jumps.
4. The adapter reaches `READY` from an approximate neutral posture without a
   T-pose or strict body-angle test.
5. The robot/MuJoCo remains on the packaged idle reference after the second
   neutral-latched `btn_10=11` event until a newer source ACK echoes the exact
   command ID/target/request and its joined canonical reference arrives;
   stale/cached acknowledgement cannot open the gate.
6. Acknowledged manual enable establishes yaw alignment and a finite live
   10-frame reference, then applies the configured 0.4-second blend.
7. Left/right arms, legs, feet, toes, torso, head, and wrist controls have the
   expected direction in MuJoCo.
8. An armed same-epoch source older than 0.5 seconds remains `LIVE/HOLD`,
   holds the newest human pose as a static canonical window, and continues
   closed-loop inference with current robot proprioception rather than freezing
   a motor target or selecting idle.
9. The ninth same-epoch recovery frame remains `HOLD`; the tenth plus its
   matching recovery-complete status/canonical join starts exactly one automatic
   0.4-second blend to `FRESH` without a button press or yaw reset.
10. An observable source-epoch change remains on the old hold, cannot consume
    the new reference, and requires new readiness plus a neutral-latched manual
    arm plus exact-ID/epoch post-command ACK/reference; only then does re-arm
    reset yaw and blend over 0.4 seconds.
11. Normal, PD-brake, zero-torque, and recovery exits remain available.
12. Repeated state entry/exit leaves no process or socket owning ports 9763,
    5559, or 5557.
13. PICO, ZeroLab, their ports, and their runtime behavior remain unchanged.
14. No user motion recording is committed to the repository.

## 28. Vendor, software, and hardware confirmations

Implementation can begin from the verified stream and official MVN manual.
The following confirmations improve hardening but do not block the first
parser/converter implementation:

- the official Real-Time Network Streaming Protocol document for MVN 2025,
  especially datagram-counter, sample-counter, time-code, wrap, playback-seek,
  and UDP source-port semantics;
- whether MVN Analyze or Network Streamer exposes a stable session/take
  identifier, restart notification, or another signal outside `MXTP02` that the
  adapter can consume;
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

`MXTP02` itself has no verified session identifier. Therefore the first
implementation guarantees re-arm for adapter restarts and for the observable
reset evidence in Section 8.4, but it cannot guarantee detection of an MVN
restart whose sender, counter, and time code all continue forward. Vendor
confirmation is non-blocking for parser/converter development, but a
deterministic external session signal is required before claiming that every
possible MVN restart is distinguished from an ordinary outage.

Any vendor answer that contradicts the verified byte layout, quaternion order,
or standardized segment-frame assumption requires a design-spec revision
before physical-robot validation; it must not be patched with an undocumented
axis guess.
