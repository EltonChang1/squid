// Board wiring for the Tactile-UMI RP2040 (Raspberry Pi Pico) firmware.
// See firmware/README.md for the full pinout table.
#pragma once

#include <stdint.h>

namespace tactile_umi::cfg {

// ---- AS5048A magnetic encoder (SPI0) --------------------------------------
constexpr uint32_t kEncSpiBaud = 1'000'000;  // AS5048A max 10 MHz
constexpr uint32_t kEncPinSck = 18;   // GP18 / SPI0 SCK
constexpr uint32_t kEncPinMosi = 19;  // GP19 / SPI0 TX
constexpr uint32_t kEncPinMiso = 16;  // GP16 / SPI0 RX
constexpr uint32_t kEncPinCs = 17;    // GP17 / SPI0 CSn (driven manually)

// ---- Strain gauge front-end: CD74HC4051 8:1 mux into ADC0 ------------------
constexpr uint32_t kStrainAdcPin = 26;  // GP26 / ADC0
constexpr uint32_t kStrainAdcInput = 0;
constexpr uint32_t kMuxPinS0 = 10;      // GP10 -> 4051 S0 (A)
constexpr uint32_t kMuxPinS1 = 11;      // GP11 -> 4051 S1 (B)
constexpr uint32_t kMuxPinS2 = 12;      // GP12 -> 4051 S2 (C)
constexpr uint32_t kMuxSettleUs = 5;  // switch + RC settle before sampling
constexpr uint32_t kAdcOversample = 4;    // averaged samples per channel
constexpr float kAdcVref = 3.3f;
constexpr float kAdcCountsToVolts = kAdcVref / 4095.0f;

// ---- Timing ----------------------------------------------------------------
constexpr uint32_t kSampleRateHz = 1000;
constexpr uint32_t kLedPin = 25;  // Pico on-board LED heartbeat

}  // namespace tactile_umi::cfg
