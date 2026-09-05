# Tactile-UMI: Handheld Visuo-Tactile Collection Device

Open-source hardware + software for collecting synchronized visuo-tactile
manipulation data with a handheld parallel-jaw gripper: dual GelSight-style
optical fingers (120 fps), a 6-axis wrist force/torque sensor and jaw-width
encoder (1 kHz), a wide-angle wrist camera (60 fps), all logged to MCAP through
ROS 2 Humble. Specification: [blueprint.html](blueprint.html).

## Repository layout

| Path | Contents |
|------|----------|
| `firmware/` | RP2040 (Pico SDK, C++) firmware: AS5048A encoder + 6-ch strain ADC at 1 kHz over USB CDC. [README](firmware/README.md) |
| `ros2_ws/src/tactile_umi/` | ROS 2 Humble package: `visuo_tactile_publisher`, `sync_monitor`, `mcap_recorder`, `firmware_emulator` |
| `tools/sync_check.py` | Offline MCAP verifier: rates, jitter, image-to-wrench alignment (pass/fail at 1 ms p99) |
| `cad/` | OpenSCAD sources for the finger module, rail base, carriage adapters, trigger, F/T plate |
| `docker/` | ROS 2 Humble image, compose file, `e2e.sh` end-to-end test |
| `docs/protocol.md` | MCU-to-host wire protocol (single source of truth) |
| `docs/verification.md` | Blueprint section 7 checklist mapped to concrete procedures |

## Architecture

```
 gripper                         on-body logging unit (Jetson / PC)
 --------                        -------------------------------------------------------
 gel cam L  --USB3-->  \         visuo_tactile_publisher
 gel cam R  --USB3-->   >------> per-camera capture threads -> /tactile/{left,right}/image_raw
 wrist cam  --USB3-->  /         serial reader thread       -> /wrist/wrench, /gripper/state
 RP2040     --USB CDC 30 B @1kHz->        (encoder angle -> jaw width via calibration)
                                            |
                                            v
                                 sync_monitor  -> /system/sync_status (DiagnosticArray, 10 Hz)
                                 mcap_recorder -> /data/sessions/session_YYYYMMDD_HHMMSS/trajectory/*.mcap
                                                  (2 ms image<->wrench jitter gate)
```

### Topics

| Topic | Type | Rate | Source |
|-------|------|------|--------|
| `/tactile/left/image_raw` | `sensor_msgs/Image` (bgr8, 640x480) | 120 Hz | left GelSight camera |
| `/tactile/right/image_raw` | `sensor_msgs/Image` (bgr8, 640x480) | 120 Hz | right GelSight camera |
| `/wrist/image_raw` | `sensor_msgs/Image` (bgr8) | 60 Hz | wide-angle wrist camera |
| `/wrist/wrench` | `geometry_msgs/WrenchStamped` | 1000 Hz | MCU frame, `ft_sensor_frame` |
| `/gripper/state` | `sensor_msgs/JointState` | 1000 Hz | MCU frame, `gripper_finger_joint` width [m], velocity, effort (-Fz) |
| `/pose/end_effector` | `geometry_msgs/PoseStamped` | 200 Hz | external MoCap/SLAM node (synthetic in sim) |
| `/system/sync_status` | `diagnostic_msgs/DiagnosticArray` | 10 Hz | `sync_monitor` |

## Quick start

### 1. ROS 2 stack (Docker, no hardware needed)

```bash
docker compose -f docker/docker-compose.yml build
# full end-to-end: build, unit tests, 12 s simulated recording, sync_check
docker compose -f docker/docker-compose.yml run --rm ros /ws/docker/e2e.sh
# same, with injected 5 ms serial stalls every 500 frames -> gate must drop images
docker compose -f docker/docker-compose.yml run --rm ros /ws/docker/e2e.sh 12 500 5
```

Interactive:

```bash
docker compose -f docker/docker-compose.yml run --rm ros
colcon build --symlink-install && source install/setup.bash     # inside, cwd /ws/ros2_ws
ros2 launch tactile_umi collect.launch.py sim:=true record:=true session_root:=/data/sessions
ros2 topic echo /system/sync_status
python3 /ws/tools/sync_check.py /data/sessions/<session>
```

Real hardware (Linux host): edit `ros2_ws/src/tactile_umi/config/params.yaml`
(camera indices, serial port, encoder calibration), uncomment the `devices:`
block in `docker/docker-compose.yml`, and launch with `sim:=false`.

Recorded bags open directly in [Foxglove Studio](https://foxglove.dev).

### 2. Firmware

```bash
brew install cmake picotool && brew install --cask gcc-arm-embedded
cmake -S firmware -B firmware/build && cmake --build firmware/build -j   # -> tactile_umi_fw.uf2
# host-side protocol tests (no SDK)
cmake -S firmware/test -B firmware/build-host && cmake --build firmware/build-host && ctest --test-dir firmware/build-host
# if the Homebrew cask cannot install (needs sudo):
docker compose -f docker/docker-compose.yml run --rm ros /ws/firmware/docker-build.sh
```

Pinout, timing budget and flashing: [firmware/README.md](firmware/README.md).

### 3. CAD

```bash
brew install --cask openscad@snapshot      # the stable "openscad" cask is disabled (Gatekeeper)
make -C cad check                          # renders cad/stl/*.stl and validates bounding boxes
openscad cad/assembly.scad                 # interactive preview; -D opening=<mm> sets jaw gap
```

Parts: `finger_body`, `finger_camera_cap`, `finger_gel_frame` (GelSight module:
Pi Camera Module 3 at 35 mm from a 20x15 mm gel window, 4 LED channels at a
30 degree grazing angle, male quick-swap dovetail), `rail_base` (MGN7H rail
bracket, 85 mm stroke, carbon rod bores, F/T-plate dovetail), `carriage_adapter`
(female dovetail + M4 thumb screw, encoder magnet pocket), `trigger`, `ft_plate`.
All screw bosses are M2.5 heat-set inserts (`common.scad`).

### 4. Unit tests without ROS

The protocol parser, alignment gate and encoder calibration are plain Python:

```bash
cd ros2_ws/src/tactile_umi && python3 -m pytest test/test_protocol.py test/test_alignment.py test/test_encoder_calibration.py
```

## Design decisions and deviations from the blueprint

* **Frame header.** The firmware prompt specifies a `0xAA 0x55` header; the
  blueprint's Python sketch reads a bare 28-byte struct. The parser
  (`tactile_umi/protocol.py`) resynchronises on the header. See `docs/protocol.md`.
* **Six strain channels on an RP2040.** The chip has four ADC inputs (three free
  on a Pico), so the reference design multiplexes one ADC through a CD74HC4051.
  `StrainFrontEnd` has a stub for an external SPI ADC (ADS131M06).
* **Timestamps** are host receive-time (blueprint design). A v2 frame carrying an
  MCU microsecond counter and sequence number is the recommended follow-up for
  true hardware timestamps; it is documented but not implemented to keep the
  specified 28-byte payload.
* **Camera capture** runs one thread per device; the blueprint's single 120 Hz
  timer with two blocking `read()` calls cannot sustain 120 fps.
* **Jitter gate.** At a healthy 1 kHz reference, the nearest wrench sample is
  always within 0.5 ms, so the 2 ms gate fires only on F/T stalls. Gated images
  are dropped and counted; all other streams are recorded unconditionally.
* **`/pose/end_effector`** has no source in the BOM; the recorder subscribes to
  it and the simulator publishes a synthetic 200 Hz trajectory so the path is
  exercised.
* **Sim mode.** Every device sits behind a `sim` parameter (synthetic cameras and
  a pty-backed firmware emulator) because Docker on macOS cannot access USB
  cameras or serial ports. Real-hardware bring-up happens on the Jetson/PC.
