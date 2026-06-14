#pragma once

#include <cstddef>
#include <cstdint>

namespace akida {

template<typename type>
struct span {
  const type* data;
  size_t size;

  span<type>() = delete;
  span<type>(const span<type>&) = default;
  span<type>(const type* d, size_t s) : data(d), size(s) {}

  span<type>& operator=(const span<type>&) = default;
};

}  // namespace akida
