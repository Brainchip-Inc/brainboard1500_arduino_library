#pragma once

#include <cstddef>
#include <cstdint>

namespace akida {
namespace compat {

size_t serialized_program_info_size(const uint8_t* serialized_program,
                                    size_t total_size);

bool validate_size_prefixed_program_info(const uint8_t* serialized_program,
                                         size_t serialized_program_size,
                                         size_t* program_info_size_out);

}  // namespace compat
}  // namespace akida
