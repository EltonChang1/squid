// Host-side unit tests for the wire protocol and AS5048A frame helpers.
// Built with the host compiler via firmware/test/CMakeLists.txt (ctest).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "as5048a_proto.h"
#include "protocol.h"

using namespace tactile_umi;
using namespace tactile_umi::as5048a_proto;

static int failures = 0;
#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++failures;                                                         \
    }                                                                     \
  } while (0)

static void test_frame_layout() {
  CHECK(kFrameSize == 30);
  CHECK(kPayloadSize == 28);
  Payload p{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
  uint8_t buf[kFrameSize];
  pack_frame(p, buf);
  CHECK(buf[0] == 0xAA);
  CHECK(buf[1] == 0x55);
  float f;
  std::memcpy(&f, buf + 2, 4);
  CHECK(f == 1.0f);
  std::memcpy(&f, buf + 26, 4);
  CHECK(f == 7.0f);
  // Little-endian check on 1.0f = 0x3F800000 -> 00 00 80 3F
  CHECK(buf[2] == 0x00 && buf[3] == 0x00 && buf[4] == 0x80 && buf[5] == 0x3F);
}

static uint16_t slow_parity15(uint16_t v) {
  int cnt = 0;
  for (int i = 0; i < 15; ++i) cnt += (v >> i) & 1;
  return cnt & 1;
}

static void test_parity() {
  for (uint32_t v = 0; v < 0x10000; ++v) {
    CHECK(even_parity15(static_cast<uint16_t>(v)) == slow_parity15(static_cast<uint16_t>(v)));
  }
  // Known value from the AS5048 application note: READ ANGLE = 0x3FFF | 0x4000
  // = 0x7FFF has 15 set bits (odd) -> parity bit 1 -> 0xFFFF.
  CHECK(make_read_cmd(kRegAngle) == 0xFFFF);
  // READ AGC 0x7FFD: 14 set bits -> parity 0.
  CHECK(make_read_cmd(kRegDiagAgc) == 0x7FFD);
  // CLEAR ERROR 0x4001: 2 set bits -> parity 0.
  CHECK(make_read_cmd(kRegClearError) == 0x4001);
  CHECK(parity_ok(0xFFFF));
  CHECK(parity_ok(0x7FFD));
  CHECK(!parity_ok(0x7FFF));
  CHECK((0xFFFF & kDataMask) == 0x3FFF);
}

static void test_unwrap() {
  CHECK(turn_delta(100, 200) == 0);
  CHECK(turn_delta(16380, 5) == 1);     // forward across zero
  CHECK(turn_delta(5, 16380) == -1);    // backward across zero
  CHECK(turn_delta(0, 8191) == 0);      // just under half a turn
  CHECK(turn_delta(0, 8193) == -1);     // just over half a turn -> ambiguous, treated as backward
}

int main() {
  test_frame_layout();
  test_parity();
  test_unwrap();
  if (failures) {
    std::fprintf(stderr, "%d failure(s)\n", failures);
    return 1;
  }
  std::puts("all firmware host tests passed");
  return 0;
}
