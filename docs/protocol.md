# Tactile-UMI Wire Protocol (MCU -> Host)

Single source of truth for the binary stream produced by the RP2040 firmware
(`firmware/src/protocol.h`) and consumed by the ROS 2 package
(`ros2_ws/src/tactile_umi/tactile_umi/protocol.py`). Both implementations must
agree with this document; the Python tests in
`ros2_ws/src/tactile_umi/test/test_protocol.py` and the host-side firmware test
in `firmware/test/test_protocol.cpp` encode the same constants.

## Transport

| Property        | Value                                      |
|-----------------|--------------------------------------------|
| Link            | USB CDC-ACM (RP2040 native USB, `/dev/ttyACM0`) |
| Nominal baud    | 115200 (ignored by USB CDC; see note below) |
| Frame rate      | 1000 Hz (hardware `repeating_timer` on the MCU) |
| Byte order      | Little-endian                              |
| Frame length    | 30 bytes                                   |

Note on bandwidth: 30 B x 1000 Hz = 240 kbit/s, which exceeds the nominal
115200 baud named in the blueprint. USB CDC does not enforce the advertised
baud rate, so the RP2040 native USB link sustains this comfortably. If the
stream is ever routed through a real UART (e.g. via an FTDI bridge), raise the
UART baud to >= 460800.

## Frame layout

```
Offset  Size  Type       Field        Unit    Notes
------  ----  ---------  -----------  ------  ---------------------------------
0       1     uint8      SYNC0        -       0xAA
1       1     uint8      SYNC1        -       0x55
2       4     float32    force_x      N       Wrist F/T, sensor frame
6       4     float32    force_y      N
10      4     float32    force_z      N
14      4     float32    torque_x     N*m
18      4     float32    torque_y     N*m
22      4     float32    torque_z     N*m
26      4     float32    encoder      rad     AS5048A angle, unwrapped, multi-turn
```

Payload is exactly the Python struct format `<fffffff` (28 bytes) preceded by the
two-byte header `AA 55`.

## Encoder convention

* The firmware sends the *unwrapped* magnet angle in radians (a continuous
  value that may exceed +/- 2*pi across turns). The firmware zeroes the angle
  at boot.
* The host converts angle to jaw opening width with a linear calibration:
  `width_m = encoder_width_scale * angle_rad + encoder_width_offset`
  (ROS parameters on `visuo_tactile_publisher`). For a rack-and-pinion or lever
  drive this is exact; for other kinematics replace the map in
  `tactile_umi/calibration.py`.

## Synchronisation and error handling (host side)

`FrameParser.feed(bytes)` is a byte-oriented state machine:

1. Scan for `0xAA`; then require `0x55` as the next byte, otherwise restart the
   scan at the byte after the `0xAA` (handles `AA AA 55 ...`).
2. Collect 28 payload bytes and emit a `Frame`.
3. Any byte discarded while hunting for a header increments `resync_bytes`.
   Every time the scanner leaves the "in payload" or "header found" state
   without emitting a frame increments `resyncs`.

The parser never blocks and tolerates arbitrary chunk boundaries (USB CDC
reads return whatever is in the FIFO).

A payload float that is NaN or +/-Inf marks the frame as `valid=False`; the
frame is still emitted so callers can count them.

## Timestamps

The blueprint's design stamps each frame with the host ROS clock at arrival.
This protocol therefore carries no MCU timestamp or sequence number. The
recommended future extension (not implemented, to keep the 28-byte payload
specified by the blueprint) is a `v2` frame with a 4-byte `uint32` MCU
microsecond counter and a 2-byte sequence number appended after the encoder
field, which would allow true hardware timestamping and exact drop detection.
