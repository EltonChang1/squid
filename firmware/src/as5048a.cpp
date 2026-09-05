#include "as5048a.h"

#include <math.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

namespace tactile_umi {

using namespace as5048a_proto;

AS5048A::AS5048A(spi_inst_t* spi, uint pin_sck, uint pin_mosi, uint pin_miso,
                 uint pin_cs, uint32_t baud)
    : spi_(spi), pin_sck_(pin_sck), pin_mosi_(pin_mosi), pin_miso_(pin_miso),
      pin_cs_(pin_cs), baud_(baud) {}

void AS5048A::init() {
  spi_init(spi_, baud_);
  // AS5048A: CPOL=0, CPHA=1 (data sampled on falling edge), 16-bit frames.
  spi_set_format(spi_, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(pin_sck_, GPIO_FUNC_SPI);
  gpio_set_function(pin_mosi_, GPIO_FUNC_SPI);
  gpio_set_function(pin_miso_, GPIO_FUNC_SPI);
  gpio_init(pin_cs_);
  gpio_set_dir(pin_cs_, GPIO_OUT);
  gpio_put(pin_cs_, 1);

  // Flush the pipeline: clear error flag, then prime an angle read.
  transfer16(make_read_cmd(kRegClearError));
  transfer16(make_read_cmd(kRegAngle));
  zero();
}

uint16_t AS5048A::transfer16(uint16_t tx) {
  uint16_t rx = 0;
  gpio_put(pin_cs_, 0);
  spi_write16_read16_blocking(spi_, &tx, &rx, 1);
  gpio_put(pin_cs_, 1);
  // CS must stay high >= 350 ns between frames; a couple of NOPs suffice at 125 MHz
  // but busy_wait keeps it explicit and portable.
  busy_wait_us_32(1);
  return rx;
}

bool AS5048A::read_raw(uint16_t* raw) {
  // Two-transfer protocol: the reply to a READ arrives during the *next*
  // transfer. Sending READ ANGLE every time means each transfer returns the
  // angle requested in the previous call, so throughput is one angle/transfer.
  uint16_t rx = transfer16(make_read_cmd(kRegAngle));
  if (rx & kErrorFlag) {
    ++errors_;
    transfer16(make_read_cmd(kRegClearError));
    transfer16(make_read_cmd(kRegAngle));  // re-prime
    return false;
  }
  if (!parity_ok(rx)) {
    ++errors_;
    return false;
  }
  *raw = rx & kDataMask;
  return true;
}

void AS5048A::zero() {
  uint16_t raw = 0;
  if (read_raw(&raw)) {
    zero_counts_ = raw;
    last_raw_ = raw;
    turns_ = 0;
    have_last_ = true;
  }
}

float AS5048A::read_angle_rad() {
  uint16_t raw = 0;
  if (read_raw(&raw)) {
    if (have_last_) {
      turns_ += turn_delta(last_raw_, raw);
    }
    last_raw_ = raw;
    have_last_ = true;
  }
  const int32_t counts =
      turns_ * static_cast<int32_t>(kCountsPerRev) + static_cast<int32_t>(last_raw_) -
      zero_counts_;
  return static_cast<float>(counts) * (2.0f * static_cast<float>(M_PI) / kCountsPerRev);
}

}  // namespace tactile_umi
