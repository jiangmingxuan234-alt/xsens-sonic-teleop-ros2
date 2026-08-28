# Xsens MVN to SONIC operation

## Stream settings

Configure MVN to send UDP to `127.0.0.1:9763` at 60 Hz using Position +
Orientation (Quaternion), with one FullBody actor, 23 segments, and no props
or fingers.

## Robot sequence

Robot sequence: `pd_brake -> normal -> first LT+RT+Y -> release -> READY ->
second LT+RT+Y -> exact ACK/reference join -> 0.4 s blend -> small motion`.

For a same-epoch outage, SONIC enters HOLD. Do not press a button; it performs
ten-frame joined auto-recovery. For a new epoch, wait for the old HOLD and new
READY, observe exact zero, then use one `LT+RT+Y` re-arm.

Exit using the existing normal, recovery, PD-brake, or zero-torque controls.
`status_timeout_s=0.2` covers the local source/bridge heartbeat, not MVN UDP
spacing. Keyboard hold/auto-repeat is unsupported, and `r` is absent from the
default hardware launch.

## Vendor and deployed-hardware confirmations

- Official MVN 2025 datagram/sample/time-code/wrap/playback-seek/source-port semantics.
- Stable session/take ID or restart signal outside MXTP02.
- Standardized segment-frame meaning evidenced by the identity MVNX frame.
- Whether quaternion representatives may flip sign.
- Heading/origin setting effects on the global frame.
- Metres plus wxyz for this streamer selection.
- BattleDragon LT axis 5, RT axis 4, Y button 4.
- CRSF CH7/CH3/CH8-Y threshold delivery.

MXTP02 cannot detect a restart whose sender tuple, sample counter, and usable
time code all continue forward.

If a vendor answer contradicts the verified 760-byte layout, wxyz order, or
standardized segment-frame assumption, stop before physical-robot motion and
revise the approved design instead of introducing an undocumented axis guess.
