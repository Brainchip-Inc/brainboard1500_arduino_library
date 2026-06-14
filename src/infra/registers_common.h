#pragma once

#include <cassert>
#include <cstdint>

namespace akida {

struct RegDetail {
  uint32_t offset;
  uint32_t nb_bits;

  explicit constexpr RegDetail(uint32_t first, uint32_t last)
      : offset(first), nb_bits(last - first + 1) {}

  explicit constexpr RegDetail(uint32_t first) : offset(first), nb_bits(1) {}
};

// Util function to set a range of bit to a value
inline void set_field(uint32_t* bits, const RegDetail& field, uint32_t value) {
  uint32_t max_value = (1 << field.nb_bits) - 1;
  assert(value <= max_value);
  // Mask value to avoid writing outside the field
  value &= max_value;
  // first clear bits
  *bits &= ~(max_value << field.offset);
  // Then set bits to value
  *bits |= value << field.offset;
}

inline uint32_t get_field(const uint32_t& bits, const RegDetail& field) {
  // create a mask
  uint32_t max_value = (1 << field.nb_bits) - 1;
  // shift and mask the value
  uint32_t ret = (bits >> field.offset) & max_value;
  return ret;
}

}  // namespace akida
