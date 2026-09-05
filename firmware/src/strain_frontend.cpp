#include "strain_frontend.h"

#include "config.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

namespace tactile_umi {

void StrainFrontEnd::set_calibration(const ChannelCal cal[kStrainChannels]) {
  for (int i = 0; i < kStrainChannels; ++i) cal_[i] = cal[i];
}

MuxedAdcFrontEnd::MuxedAdcFrontEnd(uint32_t adc_pin, uint32_t adc_input, uint32_t s0,
                                   uint32_t s1, uint32_t s2, uint32_t settle_us,
                                   uint32_t oversample)
    : adc_pin_(adc_pin), adc_input_(adc_input), s0_(s0), s1_(s1), s2_(s2),
      settle_us_(settle_us), oversample_(oversample == 0 ? 1 : oversample) {}

void MuxedAdcFrontEnd::init() {
  adc_init();
  adc_gpio_init(adc_pin_);
  adc_select_input(adc_input_);
  const uint32_t pins[3] = {s0_, s1_, s2_};
  for (uint32_t p : pins) {
    gpio_init(p);
    gpio_set_dir(p, GPIO_OUT);
    gpio_put(p, 0);
  }
}

void MuxedAdcFrontEnd::select(int ch) {
  gpio_put(s0_, (ch >> 0) & 1);
  gpio_put(s1_, (ch >> 1) & 1);
  gpio_put(s2_, (ch >> 2) & 1);
}

float MuxedAdcFrontEnd::read_channel_volts(int ch) {
  select(ch);
  busy_wait_us_32(settle_us_);
  uint32_t acc = 0;
  for (uint32_t i = 0; i < oversample_; ++i) acc += adc_read();
  const float counts = static_cast<float>(acc) / static_cast<float>(oversample_);
  return counts * cfg::kAdcCountsToVolts;
}

void MuxedAdcFrontEnd::read6(float out[kStrainChannels]) {
  for (int ch = 0; ch < kStrainChannels; ++ch) {
    out[ch] = apply_cal(ch, read_channel_volts(ch));
  }
}

}  // namespace tactile_umi
