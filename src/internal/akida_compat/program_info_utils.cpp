#include "internal/akida_compat/program_info_utils.h"

#include <cstring>

#include "akida/version.h"
#include "engine/akida_program_info_generated.h"
#include "flatbuffers/base.h"

namespace akida {
namespace compat {

size_t serialized_program_info_size(const uint8_t* serialized_program,
                                    size_t total_size) {
  if (serialized_program == nullptr ||
      total_size < sizeof(flatbuffers::uoffset_t)) {
    return 0u;
  }

  const size_t size =
      flatbuffers::ReadScalar<flatbuffers::uoffset_t>(serialized_program) +
      sizeof(flatbuffers::uoffset_t);
  return (size <= total_size) ? size : 0u;
}

bool validate_size_prefixed_program_info(const uint8_t* serialized_program,
                                         size_t serialized_program_size,
                                         size_t* program_info_size_out) {
  if (program_info_size_out != nullptr) {
    *program_info_size_out = 0u;
  }

  const size_t program_info_size =
      serialized_program_info_size(serialized_program, serialized_program_size);
  if (program_info_size == 0u || program_info_size > serialized_program_size) {
    return false;
  }

  flatbuffers::Verifier verifier(serialized_program, program_info_size);
  if (!akida::fb::VerifySizePrefixedProgramInfoBuffer(verifier)) {
    return false;
  }

  const auto* program_info =
      akida::fb::GetSizePrefixedProgramInfo(serialized_program);
  if (program_info == nullptr || program_info->version() == nullptr ||
      program_info->device_version() == nullptr) {
    return false;
  }

  if (std::strcmp(program_info->version()->c_str(), akida::version()) != 0) {
    return false;
  }

  if (program_info_size_out != nullptr) {
    *program_info_size_out = program_info_size;
  }
  return true;
}

}  // namespace compat
}  // namespace akida
