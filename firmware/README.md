# Tactile-UMI RP2040 Firmware

Samples the AS5048A jaw-width encoder (SPI) and six strain-gauge channels
(muxed ADC) at 1 kHz and streams 30-byte binary frames over USB CDC. Wire
format: [../docs/protocol.md](../docs/protocol.md).

## Pinout (Raspberry Pi Pico)

| Signal                    | Pico pin | RP2040 GPIO | Notes                                  |
|---------------------------|----------|-------------|----------------------------------------|
| AS5048A SCK               | 24       | GP18        | SPI0 SCK, 1 MHz                        |
| AS5048A MOSI (SDI)        | 25       | GP19        | SPI0 TX                                |
| AS5048A MISO (SDO)        | 21       | GP16        | SPI0 RX                                |
| AS5048A CSn               | 22       | GP17        | GPIO, active low                       |
| Strain mux output (4051 COM) | 31    | GP26 / ADC0 | 0-3.3 V after amplifier                |
| 4051 S0 (A)               | 14       | GP10        | channel select bit 0                   |
| 4051 S1 (B)               | 15       | GP11        | channel select bit 1                   |
| 4051 S2 (C)               | 16       | GP12        | channel select bit 2                   |
| Status LED                | -        | GP25        | on-board LED heartbeat                 |
| USB                       | micro-B  | -           | CDC-ACM data + 5 V power               |

Mux channel map: Y0..Y5 = Fx, Fy, Fz, Tx, Ty, Tz amplifier outputs. Y6/Y7 are
spare. Tie the 4051 `INH` (enable) pin to GND. The AS5048A runs at 3.3 V (tie
`3V3` to VDD3 and leave VDD5 floating, per its datasheet).

Why a mux: the RP2040 has four ADC inputs (three free on the Pico), so six
strain channels cannot be read directly. The `StrainFrontEnd` interface in
`src/strain_frontend.h` has a `SpiAdcFrontEnd` stub for upgrading to an
external delta-sigma ADC (e.g. ADS131M06) without touching `main.cpp`.

## Timing

* A hardware `repeating_timer` at 1000 Hz sets a pending flag; the main loop
  samples all sensors and emits one frame per tick. If the loop falls behind,
  it counts overruns and blinks the LED at 2 Hz instead of 1 Hz.
* Per-tick budget: 6 channels x (5 us settle + 4 x ~2 us ADC) ~ 80 us, one
  16-bit SPI transfer at 1 MHz ~ 20 us, USB write of 30 bytes < 100 us. Total
  well under the 1 ms period.
* Frames carry no MCU timestamp (blueprint design; host stamps on arrival).
  See the v2 note in `docs/protocol.md`.

## Build (macOS host)

```bash
brew install cmake picotool
brew install --cask gcc-arm-embedded

cd firmware
cmake -B build            # first run clones pico-sdk 2.1.1 via PICO_SDK_FETCH_FROM_GIT
cmake --build build -j
ls build/tactile_umi_fw.uf2
```

To use an existing SDK checkout instead: `cmake -B build -DPICO_SDK_PATH=/path/to/pico-sdk`.

If the Homebrew cask cannot run (it needs `sudo` for the `.pkg` installer),
cross-compile inside the repo's Docker image instead:

```bash
docker compose -f docker/docker-compose.yml run --rm ros /ws/firmware/docker-build.sh
```

## Flash

1. Hold BOOTSEL while plugging in the Pico; it mounts as `RPI-RP2`.
2. `cp build/tactile_umi_fw.uf2 /Volumes/RPI-RP2/` or `picotool load -f build/tactile_umi_fw.uf2`.
3. The device re-enumerates as a USB CDC serial port (`/dev/ttyACM0` on Linux,
   `/dev/cu.usbmodem*` on macOS). The LED blinks at 1 Hz when healthy.

## Host tests (no SDK needed)

```bash
cmake -S firmware/test -B firmware/build-host
cmake --build firmware/build-host
ctest --test-dir firmware/build-host --output-on-failure
```

Covers frame layout/endianness, AS5048A parity over all 16-bit values, read
command encoding, and multi-turn unwrap.

## Calibration

`main.cpp` captures zero-load ADC offsets at boot (64 samples per channel) and
uses unity gain. Replace the `gain` values in the `ChannelCal` table with the
results of a dead-weight calibration (N/V or N*m/V) for your amplifier board.
