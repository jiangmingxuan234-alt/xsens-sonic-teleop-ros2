# ZeroLab Real-Time MuJoCo Teleoperation Design

## Goal

Use the existing `com.bxi.sonic/sonic_zerolab` state to drive the MuJoCo ELF3
model from live ZeroLab F2 Pro data. This stage must not connect to or control a
physical ELF3.

## Existing Data Path

```text
MotionCaptureMaster
  -> UDP 992-byte packets at 50 Hz on Ubuntu port 18000
  -> zerolab_source
  -> ZMQ pose stream on 127.0.0.1:5558
  -> zerolab_bridge
  -> ZMQ smpl_ref stream on 127.0.0.1:5557
  -> SonicTeleopPolicy
  -> 29 ELF3 target joint positions
  -> MuJoCo ELF3
```

The state lifecycle owns `zerolab_source` and `zerolab_bridge`. They start when
the controller prepares `sonic_zerolab` and stop after leaving that state. No
standalone source or bridge process is required.

## Operating Sequence

1. Start `example_demo.launch.py`, which launches MuJoCo and the simulated ELF3
   controller. The configured initial state is `zero_torque`.
2. Move from `zero_torque` to `pd_brake`, then from `pd_brake` to `normal`.
3. Enable continuous 992-byte, 50 Hz UDP Stream Output in MotionCaptureMaster.
4. Before requesting `sonic_zerolab`, the operator assumes a correct T-pose.
5. Send `btn_10=4` from `normal` to enter `sonic_zerolab`.
6. Keep the T-pose stable for at least three seconds. The converter uses the
   first 100 stable packets for rest calibration, then fills the 10-frame live
   window. At 50 Hz this needs about 2.2 seconds; three seconds provides margin.
7. Start moving only after the controller log reports
   `ZeroLab stream ready; frame=...`.
8. Send `btn_1=1` to return normally to `normal`. Use `btn_3=1` for `pd_brake`
   or `btn_2=1` for `zero_torque` when an immediate simulation stop is needed.

The calibration stability check does not recognize the anatomical pose. It
only checks that rotations remain stable. Therefore the operator must already
be in T-pose when `btn_10=4` is sent.

## Runtime Verification

The run is accepted when all of the following hold:

- UDP port `18000` is owned by the state-managed Python source while
  `sonic_zerolab` is active.
- The controller reaches `com.bxi.sonic/sonic_zerolab` without a transition in
  progress.
- The source logs `ZeroLab stream ready` after the stable T-pose.
- MuJoCo responds continuously to live movements without using a recorded UDP
  replay.
- Returning to `normal` releases UDP port `18000`.

## Failure Handling

- If UDP port `18000` is already occupied, stop the record-only CLI or replay
  sender before entering `sonic_zerolab`.
- If no ready log appears, return to `normal`, verify live 992-byte packets,
  then re-enter while holding T-pose before the transition.
- If the source reports stale input, stop moving, restore the Windows stream,
  and restart the ZeroLab state so calibration is unambiguous.
- This validation does not authorize use of `example_demo_hw.launch.py` or a
  physical robot.
