// Tactile-UMI MCU -> host wire protocol.
// Mirrors docs/protocol.md and ros2_ws/src/tactile_umi/tactile_umi/protocol.py.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace tactile_umi {

constexpr uint8_t kSync0 = 0xAA;
constexpr uint8_t kSync1 = 0x55;
constexpr size_t kPayloadFloats = 7;
constexpr size_t kPayloadSize = kPayloadFloats * sizeof(float);  // 28
constexpr size_t kHeaderSize = 2;
constexpr size_t kFrameSize = kHeaderSize + kPayloadSize;  // 30
constexpr uint32_t kFrameRateHz = 1000;

static_assert(sizeof(float) == 4, "float must be IEEE754 binary32");
static_assert(kPayloadSize == 28, "payload must be 28 bytes (<fffffff)");
static_assert(kFrameSize == 30, "frame must be 30 bytes");

struct __attribute__((packed)) Payload {
  float force_x;   // N
  float force_y;   // N
  float force_z;   // N
  float torque_x;  // N*m
  float torque_y;  // N*m
  float torque_z;  // N*m
  float encoder;   // rad, unwrapped
};
static_assert(sizeof(Payload) == kPayloadSize, "Payload layout mismatch");

struct __attribute__((packed)) Frame {
  uint8_t sync0;
  uint8_t sync1;
  Payload payload;
};
static_assert(sizeof(Frame) == kFrameSize, "Frame layout mismatch");

// Serialise a frame into `out` (must hold kFrameSize bytes). Little-endian is
// assumed: both RP2040 (Cortex-M0+) and x86/arm64 hosts are little-endian.
inline void pack_frame(const Payload& p, uint8_t out[kFrameSize]) {
  out[0] = kSync0;
  out[1] = kSync1;
  memcpy(out + kHeaderSize, &p, kPayloadSize);
}

}  // namespace tactile_umi
