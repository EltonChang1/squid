// AS5048A SPI frame helpers. Pure functions, no hardware dependencies, so they
// can be unit-tested on the host (firmware/test/test_protocol.cpp).
#pragma once

#include <stdint.h>

namespace tactile_umi::as5048a_proto {

constexpr uint16_t kCmdRead = 0x4000;
constexpr uint16_t kRegNop = 0x0000;
constexpr uint16_t kRegClearError = 0x0001;
constexpr uint16_t kRegDiagAgc = 0x3FFD;
constexpr uint16_t kRegMagnitude = 0x3FFE;
constexpr uint16_t kRegAngle = 0x3FFF;
constexpr uint16_t kErrorFlag = 0x4000;
constexpr uint16_t kDataMask = 0x3FFF;
constexpr uint16_t kCountsPerRev = 16384;

// Even parity of the low 15 bits, returned as 0 or 1.
inline uint16_t even_parity15(uint16_t v) {
  v &= 0x7FFF;
  v ^= v >> 8;
  v ^= v >> 4;
  v ^= v >> 2;
  v ^= v >> 1;
  return v & 1u;
}

// Build a 16-bit read command frame: PAR | R=1 | 14-bit address.
inline uint16_t make_read_cmd(uint16_t reg) {
  uint16_t cmd = kCmdRead | (reg & kDataMask);
  return cmd | static_cast<uint16_t>(even_parity15(cmd) << 15);
}

// True if a received frame has correct parity (bit 15 = even parity of bits 0-14).
inline bool parity_ok(uint16_t rx) {
  return ((rx >> 15) & 1u) == even_parity15(rx);
}

// Multi-turn unwrap step: given previous and current 14-bit raw readings,
// returns -1, 0 or +1 turn delta.
inline int32_t turn_delta(uint16_t prev_raw, uint16_t raw) {
  const int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(prev_raw);
  if (delta > kCountsPerRev / 2) return -1;
  if (delta < -kCountsPerRev / 2) return 1;
  return 0;
}

}  // namespace tactile_umi::as5048a_proto
