// AS5048A 14-bit magnetic rotary encoder, SPI interface.
// Datasheet: https://www.infineon.com/.../infineon-as5048a-as5048b-datasheet-en.pdf
#pragma once

#include <stdint.h>

#include "as5048a_proto.h"
#include "hardware/spi.h"

namespace tactile_umi {

class AS5048A {
 public:
  AS5048A(spi_inst_t* spi, uint pin_sck, uint pin_mosi, uint pin_miso, uint pin_cs,
          uint32_t baud);

  void init();

  // Raw 14-bit angle (0..16383). Returns false on SPI error flag / parity error.
  bool read_raw(uint16_t* raw);

  // Unwrapped angle in radians relative to boot zero. Falls back to the last
  // good value on error.
  float read_angle_rad();

  void zero();

  uint32_t error_count() const { return errors_; }

 private:
  uint16_t transfer16(uint16_t tx);

  spi_inst_t* spi_;
  uint pin_sck_, pin_mosi_, pin_miso_, pin_cs_;
  uint32_t baud_;
  uint16_t last_raw_ = 0;
  int32_t turns_ = 0;
  int32_t zero_counts_ = 0;
  bool have_last_ = false;
  uint32_t errors_ = 0;
};

}  // namespace tactile_umi
