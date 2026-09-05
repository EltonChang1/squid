// Six-channel strain-gauge front-end abstraction.
//
// The RP2040 exposes only four ADC inputs (three free on a Pico board, GP26-28;
// GP29 measures VSYS), so the blueprint's "6 analog channels" cannot be read
// directly. The default implementation multiplexes one ADC pin through a
// CD74HC4051 8:1 analog switch. A stub for an external SPI delta-sigma ADC
// (e.g. TI ADS131M06) is provided as the upgrade path for higher resolution.
#pragma once

#include <stdint.h>

namespace tactile_umi {

constexpr int kStrainChannels = 6;

struct ChannelCal {
  float offset_volts;  // measured at zero load
  float gain;          // engineering units (N or N*m) per volt
};

class StrainFrontEnd {
 public:
  virtual ~StrainFrontEnd() = default;
  virtual void init() = 0;
  // Fills out[0..5] with calibrated values (Fx Fy Fz Tx Ty Tz).
  virtual void read6(float out[kStrainChannels]) = 0;
  void set_calibration(const ChannelCal cal[kStrainChannels]);

 protected:
  ChannelCal cal_[kStrainChannels] = {
      {0.0f, 1.0f}, {0.0f, 1.0f}, {0.0f, 1.0f}, {0.0f, 1.0f}, {0.0f, 1.0f}, {0.0f, 1.0f}};
  float apply_cal(int ch, float volts) const {
    return (volts - cal_[ch].offset_volts) * cal_[ch].gain;
  }
};

// CD74HC4051 mux -> single RP2040 ADC input.
class MuxedAdcFrontEnd : public StrainFrontEnd {
 public:
  MuxedAdcFrontEnd(uint32_t adc_pin, uint32_t adc_input, uint32_t s0, uint32_t s1,
                   uint32_t s2, uint32_t settle_us, uint32_t oversample);
  void init() override;
  void read6(float out[kStrainChannels]) override;
  float read_channel_volts(int ch);

 private:
  void select(int ch);
  uint32_t adc_pin_, adc_input_, s0_, s1_, s2_;
  uint32_t settle_us_;
  uint32_t oversample_;
};

// Extension point: external SPI ADC. Not wired on the reference board.
class SpiAdcFrontEnd : public StrainFrontEnd {
 public:
  void init() override {}
  void read6(float out[kStrainChannels]) override {
    for (int i = 0; i < kStrainChannels; ++i) out[i] = 0.0f;
  }
};

}  // namespace tactile_umi
