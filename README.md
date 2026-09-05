# Squid

Software and CAD for a handheld parallel-jaw gripper that records synchronized visuo-tactile data: two GelSight-style fingers (120 fps), a 6-axis wrist force/torque sensor and jaw-width encoder (1 kHz), and a wrist camera (60 fps). Streams go through ROS 2 Humble and land in MCAP.

Full specification: [blueprint.html](blueprint.html).

## Layout

| Path | What it is |
|------|------------|
| `firmware/` | RP2040 firmware: AS5048A + 6-ch strain at 1 kHz over USB CDC ([pinout](firmware/README.md)) |
| `ros2_ws/src/tactile_umi/` | ROS 2 package: publisher, sync monitor, MCAP recorder, sim sources |
| `tools/sync_check.py` | Offline MCAP check: rates, jitter, image-to-wrench alignment |
| `cad/` | OpenSCAD gripper and GelSight finger |
| `docker/` | Humble image and `e2e.sh` |
| `docs/` | [Wire protocol](docs/protocol.md), [verification checklist](docs/verification.md) |

## Quick start

No hardware required for the sim path:

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml run --rm ros /ws/docker/e2e.sh
```

That builds the package, runs unit tests, records ~12 s of simulated data, and runs `sync_check.py`. Inject serial stalls (gate must drop images):

```bash
docker compose -f docker/docker-compose.yml run --rm ros /ws/docker/e2e.sh 12 500 5
```

Interactive stack:

```bash
docker compose -f docker/docker-compose.yml run --rm ros
# inside, cwd /ws/ros2_ws
colcon build --symlink-install && source install/setup.bash
ros2 launch tactile_umi collect.launch.py sim:=true record:=true session_root:=/data/sessions
python3 /ws/tools/sync_check.py /data/sessions/<session>
```

On a Linux host with cameras and a Pico, edit `ros2_ws/src/tactile_umi/config/params.yaml`, uncomment `devices:` in `docker/docker-compose.yml`, and launch with `sim:=false`. Bags open in [Foxglove](https://foxglove.dev).

**Firmware** (UF2 at `firmware/build/tactile_umi_fw.uf2`):

```bash
# host ARM GCC
cmake -S firmware -B firmware/build && cmake --build firmware/build -j
# or inside Docker
docker compose -f docker/docker-compose.yml run --rm ros /ws/firmware/docker-build.sh
# protocol tests, no Pico SDK
cmake -S firmware/test -B firmware/build-host && cmake --build firmware/build-host && ctest --test-dir firmware/build-host
```

**CAD:**

```bash
make -C cad check    # renders cad/stl/*.stl and checks bounding boxes
```

**Python-only tests:** `cd ros2_ws/src/tactile_umi && python3 -m pytest test/test_protocol.py test/test_alignment.py test/test_encoder_calibration.py`

## Topics

| Topic | Type | Rate |
|-------|------|------|
| `/tactile/left/image_raw` | `sensor_msgs/Image` bgr8 640×480 | 120 Hz |
| `/tactile/right/image_raw` | `sensor_msgs/Image` bgr8 640×480 | 120 Hz |
| `/wrist/image_raw` | `sensor_msgs/Image` bgr8 | 60 Hz |
| `/wrist/wrench` | `geometry_msgs/WrenchStamped` | 1 kHz |
| `/gripper/state` | `sensor_msgs/JointState` (width m) | 1 kHz |
| `/pose/end_effector` | `geometry_msgs/PoseStamped` | 200 Hz (external / sim) |
| `/system/sync_status` | `diagnostic_msgs/DiagnosticArray` | 10 Hz |

Sessions: `/data/sessions/session_YYYYMMDD_HHMMSS/trajectory/*.mcap`. Images more than 2 ms from the nearest wrench sample are dropped.

## Notes vs the blueprint

- Frames are `0xAA 0x55` + 28-byte `<fffffff`; the parser resyncs on the header ([`docs/protocol.md`](docs/protocol.md)).
- Six strain channels use a CD74HC4051 into one RP2040 ADC (the chip does not have six analog pins).
- Stamps are host receive time. One capture thread per camera. `sim:=true` is required in Docker on macOS (no USB passthrough).
