# ZeroLab/PICO Offline Evaluation Design

## Goal

Build a reproducible, offline evaluation pipeline for a paired PICO + ZeroLab
capture. The first implementation stops at human-side evaluation: it decodes
both raw streams, produces the same canonical pose representation, estimates
residual timing offset, resamples both sources to a 50 Hz common timeline, and
reports cross-device agreement, timing quality, smoothness, and low-motion
stability. Sonic/MuJoCo replay will consume the canonical output in a later
phase and is intentionally outside this design.

The first validation capture is `/tmp/pair-static-013`. It is a smoke-test
dataset, not a final benchmark: PICO emitted rate warnings and ZeroLab's
effective receive rate was below the nominal 50 Hz.

## Non-goals

- No online controller or state-machine changes.
- No simultaneous PICO/ZeroLab input to Sonic.
- No claim of absolute tracking accuracy; without Vicon/OptiTrack ground truth,
  results are reported as cross-device agreement/consistency.
- No inference of action labels or manual action boundaries in the first version.

## Architecture

The implementation is split into two deterministic command-line stages:

```text
pair capture directory
        |
        +--> zerolab.canonical_cli
        |       decode PICO and ZeroLab
        |       recover timestamps and deduplicate frames
        |       calibrate ZeroLab rest pose
        |       crop overlap, estimate residual delay
        |       resample to 50 Hz
        |
        +--> pico_canonical.npz
        +--> zerolab_canonical.npz
        +--> alignment.json
                        |
                        +--> zerolab.metrics_cli
                                metrics and quality flags
                                report.json / metrics.csv / summary.md
```

The modules have single responsibilities:

- `zerolab/evaluation_io.py`: read the pair manifest, PICO batch files, and
  ZeroLab raw recording; validate and read/write canonical files.
- `zerolab/alignment.py`: reconstruct PICO frame times, crop to overlap,
  estimate residual delay, and perform 50 Hz interpolation.
- `zerolab/metrics.py`: pure NumPy/SciPy metric functions and quality flags.
- `zerolab/canonical_cli.py`: first-stage command-line orchestration.
- `zerolab/metrics_cli.py`: second-stage report generation.

Existing `protocol.py`, `recording.py`, and `converter.py` remain the source of
truth for the UDP wire format, raw-record container, and ZeroLab-to-SMPL/ELF3
conversion. Online ZeroLab and PICO behavior is not modified.

## Canonical data contract

Each canonical file is a compressed NumPy archive containing:

```text
time_s                 float64 [N]
frame_index            int64   [N]
smpl_pose_axis_angle   float32 [N, 21, 3]
smpl_joints_local      float32 [N, 24, 3]
body_quat_w            float32 [N, 4]      # scalar-last xyzw
joint_pos              float32 [N, 29]
source                 string metadata
rate_hz                float64 scalar
```

`time_s` is seconds on the Ubuntu monotonic-clock domain. Both sources use the
same semantic joint order and coordinate convention as the existing Sonic
bridge.

### PICO decoding and time recovery

PICO recording files contain ten output frames and adjacent files overlap by
nine frames. The decoder:

1. validates required fields (`smpl_pose`, `smpl_joints`, `body_quat_w`,
   `joint_pos`, `frame_index`, `timestamp_monotonic`);
2. associates a file's `timestamp_monotonic` with the last frame in that
   batch, matching the writer's behavior;
3. backfills the preceding nine frame timestamps at the nominal 50 Hz period;
4. deduplicates by `frame_index`, preserving the first complete occurrence;
5. converts the recorded seconds to the canonical `time_s` representation.

The decoder records this assumption in `alignment.json` so that a later
recorder format can be handled explicitly rather than silently reinterpreted.

### ZeroLab decoding and calibration

The decoder iterates `<QQ992s>` records, validates the 992-byte payload with
`parse_zerolab_packet`, and uses `ZeroLabMotionConverter` to apply the existing
Unity-to-XRT transform, rest-pose alignment, SMPL-24 mapping, FK, and ELF3
joint mapping. Calibration uses the initial stable T-pose window. If the
required window cannot be collected, canonical generation fails with a clear
error and does not emit partial output.

## Time alignment

1. Convert ZeroLab `receive_timestamp_ns` to seconds and use PICO's recovered
   monotonic seconds directly.
2. Compute the closed interval where both sources have samples; discard
   ZeroLab samples outside that interval. If the overlap is less than one
   second, fail.
3. Interpolate both sources to a common 50 Hz grid. Axis-angle pose and
   position values use linear interpolation; quaternion values use normalized
   shortest-path interpolation.
4. Compute a motion-energy signal from root, both shoulders, both elbows, and
   both knees. The signal is the mean normalized angular-speed magnitude of
   these joints.
5. Search residual lag in `[-1.5, +1.5]` seconds at one 50 Hz sample step and
   select the lag with maximum normalized cross-correlation.
6. Define positive lag as “ZeroLab is later than PICO”. Shift the ZeroLab
   canonical time axis earlier by the estimated positive lag and resample the
   final aligned pair on the intersection after shifting.

`alignment.json` contains overlap bounds, grid rate, search range, estimated
lag, correlation coefficient, sign convention, timestamp reconstruction
assumption, and quality flags. A best correlation below `0.35` produces
`alignment_low_confidence`; it does not invent a zero delay.

## Metrics

### Timing quality (per source)

- sample count and duration;
- mean rate, mean/std/P95/max inter-sample interval;
- gap count and gap ratio, with a gap defined as an interval over 30 ms;
- PICO frame-index gaps and ZeroLab malformed/truncated-record counts.

### Cross-device pose agreement

For each of the 21 SMPL pose joints, convert axis-angle to scalar-last
quaternions and compute the sign-invariant geodesic error:

```text
error_deg = 2 * acos(clamp(abs(dot(q_pico, q_zerolab)), 0, 1))
```

Report mean, RMS, P95, and maximum per joint and overall. Root orientation is
reported separately.

### Position and Sonic-target agreement

- Euclidean RMSE/P95 for each of the 24 local SMPL joints and overall;
- RMSE/P95 for each of the 29 ELF3 joint targets and overall.

These are agreement measures between the two converted streams, not absolute
errors against a reference body.

### Dynamic agreement

At the common 50 Hz rate, finite differences produce angular and positional
velocities. Report pose angular-velocity RMSE (degrees/second) and SMPL joint
position-velocity RMSE (meters/second).

### Smoothness and low-motion stability (per source)

Report RMS pose velocity, acceleration, jerk, and joint-position acceleration.
For stability, find contiguous windows of at least one second whose aggregate
angular speed remains below 5 degrees/second. In those windows report root
orientation standard deviation, pose-angle standard deviation, and SMPL joint
position standard deviation. If no window qualifies, use
`insufficient_low_motion_window` rather than zero.

## Command-line interface and outputs

Canonical generation:

```bash
python3 -m zerolab.canonical_cli \
  --capture /tmp/pair-static-013 \
  --output /tmp/pair-static-013-eval \
  --rate-hz 50 \
  --delay-search-s 1.5
```

Outputs:

```text
/tmp/pair-static-013-eval/pico_canonical.npz
/tmp/pair-static-013-eval/zerolab_canonical.npz
/tmp/pair-static-013-eval/alignment.json
```

Metric generation:

```bash
python3 -m zerolab.metrics_cli \
  --pico /tmp/pair-static-013-eval/pico_canonical.npz \
  --zerolab /tmp/pair-static-013-eval/zerolab_canonical.npz \
  --alignment /tmp/pair-static-013-eval/alignment.json \
  --output /tmp/pair-static-013-eval/report
```

Outputs:

```text
report/report.json
report/metrics.csv
report/summary.md
```

`metrics.csv` is a long table with columns
`section,source_or_pair,joint_or_dof,metric,value,unit`. `report.json` is the
complete machine-readable result. `summary.md` presents key values and all
quality warnings, including PICO rate warnings, ZeroLab sub-50 Hz receive rate,
low alignment correlation, and the absence of absolute ground truth.

## Error handling

The commands return a non-zero status and a concise diagnostic for missing
inputs, non-empty output directories, missing PICO fields, unrecoverable frame
indices, malformed/truncated ZeroLab records, failed T-pose calibration, no
time overlap, insufficient overlap samples, and invalid delay-correlation
results. A metric that lacks enough data is represented as an explicit
`insufficient_data` status rather than a fabricated numeric zero.

## Verification strategy

Unit tests use synthetic data to cover:

- PICO ten-frame overlap removal and timestamp reconstruction;
- ZeroLab raw-record decoding;
- recovery of a known 120 ms synthetic delay;
- quaternion sign equivalence (`q` and `-q`);
- position, velocity, jitter, and stability calculations;
- no-overlap, calibration-failure, and truncated-record errors.

The first real-data smoke test runs both commands on `/tmp/pair-static-013`,
checks that all canonical/report files are produced, and checks that its known
rate warnings appear as quality flags. It is not promoted to final benchmark
evidence until a capture with nominal rates and a clearly visible sync motion
is recorded.

