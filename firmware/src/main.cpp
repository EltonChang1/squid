// Tactile-UMI RP2040 firmware: 1 kHz AS5048A encoder + 6ch strain sampling,
// streamed over USB CDC as 30-byte frames (docs/protocol.md).

#include <stdio.h>

#include "as5048a.h"
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "protocol.h"
#include "strain_frontend.h"

namespace {

volatile uint32_t g_ticks = 0;  // incremented by the 1 kHz timer ISR
volatile uint32_t g_pending = 0;

bool on_tick(repeating_timer_t*) {
  ++g_ticks;
  ++g_pending;
  return true;
}

}  // namespace

int main() {
  using namespace tactile_umi;

  stdio_usb_init();

  gpio_init(cfg::kLedPin);
  gpio_set_dir(cfg::kLedPin, GPIO_OUT);

  AS5048A encoder(spi0, cfg::kEncPinSck, cfg::kEncPinMosi, cfg::kEncPinMiso,
                  cfg::kEncPinCs, cfg::kEncSpiBaud);
  encoder.init();

  MuxedAdcFrontEnd strain(cfg::kStrainAdcPin, cfg::kStrainAdcInput, cfg::kMuxPinS0,
                          cfg::kMuxPinS1, cfg::kMuxPinS2, cfg::kMuxSettleUs,
                          cfg::kAdcOversample);
  strain.init();
  // Zero-load offsets are captured at boot; gains are board-specific and
  // should be replaced with values from a dead-weight calibration.
  {
    ChannelCal cal[kStrainChannels];
    for (int ch = 0; ch < kStrainChannels; ++ch) {
      float acc = 0.0f;
      for (int i = 0; i < 64; ++i) acc += strain.read_channel_volts(ch);
      cal[ch].offset_volts = acc / 64.0f;
      cal[ch].gain = 1.0f;
    }
    strain.set_calibration(cal);
  }

  repeating_timer_t timer;
  add_repeating_timer_us(-static_cast<int64_t>(1'000'000 / cfg::kSampleRateHz), on_tick,
                         nullptr, &timer);

  uint8_t frame[kFrameSize];
  Payload payload{};
  float strain_vals[kStrainChannels];
  uint32_t overruns = 0;

  while (true) {
    if (g_pending == 0) {
      tight_loop_contents();
      continue;
    }
    // Consume exactly one tick; if the loop fell behind, count the overrun and
    // catch up by emitting frames back-to-back (host stamps on arrival).
    uint32_t pending = g_pending;
    if (pending > 1) overruns += pending - 1;
    g_pending = 0;

    strain.read6(strain_vals);
    payload.force_x = strain_vals[0];
    payload.force_y = strain_vals[1];
    payload.force_z = strain_vals[2];
    payload.torque_x = strain_vals[3];
    payload.torque_y = strain_vals[4];
    payload.torque_z = strain_vals[5];
    payload.encoder = encoder.read_angle_rad();

    pack_frame(payload, frame);
    if (stdio_usb_connected()) {
      fwrite(frame, 1, kFrameSize, stdout);
      fflush(stdout);
    }

    // Heartbeat: 1 Hz blink; double-rate if overruns or encoder errors accumulate.
    const uint32_t period = (overruns > 0 || encoder.error_count() > 100) ? 250 : 500;
    gpio_put(cfg::kLedPin, (g_ticks / period) & 1);
  }
}
