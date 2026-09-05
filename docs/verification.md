# Hardware & Pipeline Verification Checklist

Blueprint section 7, mapped to concrete procedures. Items marked *software*
are fully automated in this repository; *manual* items need the physical
device and are documented as procedures with pass criteria.

## 1. Timestamp synchronization (software)

**Criterion.** Image header stamps and F/T wrench stamps align within < 1.0 ms
(Foxglove inspection in the blueprint; automated here).

**Procedure.**

```bash
# simulated device, 12 s recording, includes unit tests
docker compose -f docker/docker-compose.yml run --rm ros /ws/docker/e2e.sh
# on a real session
python3 tools/sync_check.py /data/sessions/session_YYYYMMDD_HHMMSS
```

`sync_check.py` prints per-topic rate, inter-message jitter (p50/p99/max
deviation from the nominal period), estimated drops, and for each tactile
stream the offset to the nearest `/wrist/wrench` sample with a histogram. It
exits 0 only when p99 |offset| < 1.0 ms for both tactile streams. The Docker
sim E2E (`docker/e2e.sh`) relaxes that to 2.0 ms: every *recorded* image is
still inside the recorder's gate, but host-thread timestamps inside a 4-core
VM cannot honestly claim the 1 ms hardware criterion.

Live: `ros2 topic echo /system/sync_status` shows the same metrics at 10 Hz
(`tactile_umi/align/...` entries; OK < 1 ms, WARN < 2 ms, ERROR otherwise).

Fault injection: `docker/e2e.sh 12 500 5` stalls the emulated serial link for
5 ms every 500 frames; the recorder must report `dropped_images > 0` and the
recorded (gated) bag must still pass at 2 ms.

Foxglove cross-check: open the `.mcap`, add a Plot panel with
`/wrist/wrench.header.stamp` and `/tactile/left/image_raw.header.stamp`
message-path receive-time deltas, or use the Raw Messages panel to compare
stamps of adjacent messages.

## 2. Serial link integrity (software)

**Criterion.** No parser resyncs, no inter-frame gaps > 3 ms over a 5 minute
capture.

**Procedure.** The publisher logs every 5 s:
`serial frames=N resyncs=R gaps=G max_gap=X ms`. `resyncs` must stay 0 with
real firmware (it is non-zero only when the emulator injects garbage); `gaps`
counts inter-frame intervals above 3 nominal periods.

## 3. Thermal validation (manual)

**Criterion.** LED rings inside the optical fingers do not heat the gel past
45 C during a 30 minute continuous capture.

**Procedure.**

1. Run `ros2 launch tactile_umi collect.launch.py record:=false` for 30 minutes
   with LEDs at operating current.
2. Every 5 minutes measure the gel surface with an IR thermometer or a K-type
   probe touched to the gel edge (avoid the imaging area). Record ambient.
3. Pass if every reading is <= 45 C and the temperature has plateaued (delta
   between the 25 and 30 minute readings < 1 C).
4. Mitigations if failed: reduce LED current (PWM), add copper tape from the
   LED ring to the finger body, increase `wall` in `cad/common.scad` and add
   vent channels.

## 4. Optics & focus (manual)

**Criterion.** Sub-millimetre texture resolution: the ridge lines of a U.S.
quarter are clearly visible in the gel image feed.

**Procedure.**

1. `ros2 run rqt_image_view rqt_image_view /tactile/left/image_raw` (or Foxglove
   Image panel).
2. Press a quarter into the gel at roughly 5 N.
3. Pass if the reeded edge (119 ridges, ~0.6 mm pitch) resolves as distinct
   lines and the letters of "LIBERTY" are legible.
4. If soft: adjust the camera standoff shim (design value `cam_standoff = 35 mm`
   in `cad/common.scad`), confirm the lens is focused at 35 mm, and check the
   gel coating thickness (10-20 um; too thick blurs fine texture).

## 5. Structural deflection (manual)

**Criterion.** Jaws deflect <= 0.5 mm at 40 N pinch force.

**Procedure.**

1. Mount a rigid 20 mm block between the fingers; place a dial indicator on
   the outer face of one finger module, perpendicular to the jaw axis.
2. Apply 40 N with the trigger (verify via `/wrist/wrench` force.z or a
   calibrated scale in series).
3. Pass if indicator reading <= 0.5 mm and returns to 0 +/- 0.05 mm on release.
4. Verify `/gripper/state` position changes by less than 0.5 mm during the load
   (encoder sees carriage motion, indicator sees finger flex; both must be
   within limit).
5. If failed: print `carriage_adapter` and `finger_body` in PA-CF, increase
   `wall`, or add a second MGN7H carriage per jaw.

## 6. Encoder calibration (manual, once per build)

1. Close the jaws fully; note `/gripper/state` position (should be ~0).
2. Insert an 85 mm gauge block; note the raw encoder angle
   (`ros2 topic echo /wrist/wrench` does not carry it - temporarily set
   `encoder_width_scale: 1.0, encoder_width_offset: 0.0` to read the raw angle
   in `/gripper/state.position`).
3. Fit `EncoderCalibration.from_two_points(angle_closed, 0.0, angle_open, 0.085)`
   and write `scale`/`offset` into `config/params.yaml`.

## 7. Force/torque calibration (manual, once per build)

Apply known dead weights along each axis; solve the 6x6 gain matrix (or at
least per-channel gains) and enter them in the `ChannelCal` table in
`firmware/src/main.cpp`. The boot-time zero-offset capture assumes the gripper
is unloaded and stationary at power-up.
