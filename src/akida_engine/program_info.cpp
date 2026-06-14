#include "akida/program_info.h"

#include <cstring>

#include "akida/hw_version.h"
#include "akida/shape.h"
#include "akida/version.h"

#include "engine/dma.h"
#include "infra/system.h"

#include "flatbuffers/base.h"

#include "engine/akida_device_program_fb_generated.h"
#include "engine/akida_program_info_generated.h"

namespace akida {

#ifndef AKIDA_NICLA_PROGRAM_INFO_TRACE
#define AKIDA_NICLA_PROGRAM_INFO_TRACE 0
#endif

#if AKIDA_NICLA_PROGRAM_INFO_TRACE
#define AKIDA_NICLA_PROGRAM_INFO_LOG(...) std::printf(__VA_ARGS__)
#else
#define AKIDA_NICLA_PROGRAM_INFO_LOG(...) ((void)0)
#endif

ProgramInfo::ProgramInfo()
    : program_data_{nullptr, 0},
      program_data_address_(0),
      program_(nullptr),
      program_info_(nullptr) {}

ProgramInfo::ProgramInfo(const uint8_t* serialized_program_info,
                         [[maybe_unused]] size_t program_info_size,
                         uint32_t program_data_address)
    : ProgramInfo() {
  if (!serialized_program_info) {
    panic("Program info is null");
  }

  const auto program_info_fb_size =
      flatbuffers::ReadScalar<flatbuffers::uoffset_t>(serialized_program_info) +
      sizeof(flatbuffers::uoffset_t);

  AKIDA_NICLA_PROGRAM_INFO_LOG(
      "[AKIDA][PROGRAM_INFO] ext ctor declared_size=%lu fb_size=%lu ext_addr=0x%08lX\r\n",
      static_cast<unsigned long>(program_info_size),
      static_cast<unsigned long>(program_info_fb_size),
      static_cast<unsigned long>(program_data_address));

  flatbuffers::Verifier program_info_verifier(serialized_program_info,
                                              program_info_fb_size);
  AKIDA_NICLA_PROGRAM_INFO_LOG(
      "[AKIDA][PROGRAM_INFO] ext ctor verifier constructed\r\n");
  auto* program_info =
      fb::GetSizePrefixedProgramInfo(serialized_program_info);
  AKIDA_NICLA_PROGRAM_INFO_LOG("[AKIDA][PROGRAM_INFO] ext ctor root=%p\r\n",
                               static_cast<const void*>(program_info));

  size_t vtable_entries = 0;
  if (program_info != nullptr) {
    const auto* table_ptr =
        reinterpret_cast<const uint8_t*>(program_info);
    const auto vtable_offset =
        flatbuffers::ReadScalar<flatbuffers::soffset_t>(table_ptr);
    const auto* vtable_ptr = table_ptr - vtable_offset;
    const auto vtable_size =
        flatbuffers::ReadScalar<flatbuffers::voffset_t>(vtable_ptr);
    if (vtable_size >= 4) {
      vtable_entries = (vtable_size - 4u) / 2u;
    }
    AKIDA_NICLA_PROGRAM_INFO_LOG(
        "[AKIDA][PROGRAM_INFO] ext ctor vtable_entries=%lu local_schema_fields=15\r\n",
        static_cast<unsigned long>(vtable_entries));
  }

  const bool verified =
      fb::VerifySizePrefixedProgramInfoBuffer(program_info_verifier);
  AKIDA_NICLA_PROGRAM_INFO_LOG("[AKIDA][PROGRAM_INFO] ext ctor verify=%s\r\n",
                               verified ? "YES" : "NO");
  if (program_info == nullptr || verified == false) {
    panic(
        "Unable to parse program info; likely schema/version mismatch "
        "(export Akida 2.18.2 vs local engine %s, vtable_entries=%lu)",
        version(), static_cast<unsigned long>(vtable_entries));
  }

  AKIDA_NICLA_PROGRAM_INFO_LOG(
      "[AKIDA][PROGRAM_INFO] ext ctor field access begin\r\n");
  if (program_info->version() == nullptr) {
    panic("Program info is missing version");
  }
  if (program_info->device_version() == nullptr) {
    panic("Program info is missing device_version");
  }

  const auto& program_version = program_info->version()->c_str();
  const auto& lib_version = version();
  AKIDA_NICLA_PROGRAM_INFO_LOG(
      "[AKIDA][PROGRAM_INFO] ext ctor version prog=%s lib=%s\r\n",
      program_version, lib_version);
  if (strcmp(program_version, lib_version) != 0) {
    panic("Program version [%s] does not match library version [%s]",
          program_version, lib_version);
  }

  program_data_address_ = program_data_address;
  program_info_ = program_info;
  AKIDA_NICLA_PROGRAM_INFO_LOG("[AKIDA][PROGRAM_INFO] ext ctor complete\r\n");
}

ProgramInfo::ProgramInfo(const uint8_t* serialized_program_buffer,
                         [[maybe_unused]] size_t serialized_program_size)
    : ProgramInfo() {
  if (!serialized_program_buffer) {
    panic("Program is null");
  }
  // Serialized program is now split in 2 parts: program_info and program
  // (data). 1st part in the buffer is program_info, then program. Both are size
  // prefixed, that means there is a uoffset_t word at the begining of the
  // buffer that contains the size of the actual flatbuffer data. The full size
  // is then this size + sizeof(uoffset_t).
  const auto program_info_size =
      flatbuffers::ReadScalar<flatbuffers::uoffset_t>(
          serialized_program_buffer) +
      sizeof(flatbuffers::uoffset_t);
  // 1st verify program info
  flatbuffers::Verifier program_info_verifier(serialized_program_buffer,
                                              program_info_size);
  auto* program_info =
      fb::GetSizePrefixedProgramInfo(serialized_program_buffer);
  if (program_info == nullptr ||
      fb::VerifySizePrefixedProgramInfoBuffer(program_info_verifier) == false) {
    panic("Unable to parse program info");
  }

  // Then verify program buffer that is located after the program_info part
  const auto* program_data_ptr = serialized_program_buffer + program_info_size;
  // Program flatbuffer is size prefixed, that means there is a uoffset_t word
  // at the begining of the buffer that contains the size of the actual
  // flatbuffer data. The full size is then this size + sizeof(uoffset_t)
  const auto program_data_size =
      flatbuffers::ReadScalar<flatbuffers::uoffset_t>(program_data_ptr) +
      sizeof(flatbuffers::uoffset_t);
  flatbuffers::Verifier program_verifier(program_data_ptr, program_data_size);
  if (fb::VerifySizePrefixedProgramBuffer(program_verifier) == false) {
    panic("Unable to parse program");
  }

  // size of buffer should be equal to the sum of both flatbuffers size (+ size
  // words)
  assert((program_info_size + program_data_size == serialized_program_size) &&
         "Unexpected program buffer size");

  auto* program = fb::GetSizePrefixedProgram(program_data_ptr);
  // Check that the akida version this program was compiled with matches the
  // current version.
  const auto& program_version = program_info->version()->c_str();
  const auto& lib_version = version();
  if (strcmp(program_version, lib_version) != 0) {
    panic("Program version [%s] does not match library version [%s]",
          program_version, lib_version);
  }

  // store info about the program parts
  program_data_ = {program_data_ptr, program_data_size};
  program_data_address_ = 0;
  program_ = program;
  program_info_ = program_info;
}

HwVersion ProgramInfo::device_version() const {
  if (program_info_ == nullptr) {
    panic("Program info is not initialized");
  }
  const auto* fb_device_version = program_info_->device_version();
  if (fb_device_version == nullptr) {
    panic("Program info device_version is null");
  }

  return HwVersion{
      fb_device_version->vendor_id(), fb_device_version->product_id(),
      fb_device_version->major_rev(), fb_device_version->minor_rev()};
}

const uint32_t* ProgramInfo::input_dims() const {
  return program_info_->input_dims()->data();
}

Shape ProgramInfo::output_dims() const {
  const auto& output_dims = *program_info_->output_dims();
  return Shape{output_dims[0], output_dims[1], output_dims[2]};
}

bool ProgramInfo::input_is_dense() const {
  return program_info_->input_type() == fb::IoType_dense;
}

bool ProgramInfo::output_is_dense() const {
  return program_info_->output_type() == fb::IoType_dense;
}

bool ProgramInfo::activation_enabled() const {
  return program_info_->activation();
}

uint32_t ProgramInfo::dense_input_window_width() const {
  return program_info_->dense_window_w();
}

uint32_t ProgramInfo::dense_input_window_height() const {
  return program_info_->dense_window_h();
}

bool ProgramInfo::can_learn() const {
  return program_info_->learning_layer_span() != nullptr;
}

uint32_t ProgramInfo::learn_weights_word_size() const {
  const auto* learning_layer = program_info_->learning_layer_span();
  if (learning_layer == nullptr) {
    return 0;
  }

  const auto* ram_span = learning_layer->ram_span();
  if (ram_span == nullptr) {
    return 0;
  }

  if (ram_span->fnp2_track() != nullptr) {
    return ram_span->fnp2_track()->track().word_size();
  }

  const auto* tracks = ram_span->tracks();
  if (tracks == nullptr || tracks->size() == 0) {
    return 0;
  }

  assert(tracks->size() == 1);
  return tracks->Get(0)->word_size();
}

uint8_t ProgramInfo::number_of_descriptors_per_pass() const {
  return program_info_->max_num_desc();
}

uint32_t ProgramInfo::number_of_passes() const {
  return program_info_->pass_spans()->size();
}

uint32_t ProgramInfo::number_of_program_descriptors_required() const {
  const auto nb_passes = number_of_passes();
  return nb_passes > 1 ? nb_passes * number_of_descriptors_per_pass()
                       : dma::kMinNbDescriptors;
}

uint32_t ProgramInfo::number_of_extra_program_descriptors_required() const {
  // There is an extra descriptor if leaning is running on FNP3 during a
  // multipass program
  return (number_of_passes() > 1 && learning_on_fnp3()) ? 1 : 0;
}

bool ProgramInfo::learning_on_fnp3() const {
  return program_info_->learning_layer_span() != nullptr &&
         program_info_->learning_layer_span()->ram_span() != nullptr &&
         program_info_->learning_layer_span()->ram_span()->tracks() != nullptr &&
         program_info_->learning_layer_span()->ram_span()->fnp2_track() == nullptr;
}

static inline size_t track_bytes_size(const fb::TrackSpan& track) {
  return track.word_size() * sizeof(uint32_t);
}

static size_t record_np_tracks_byte_size(const fb::RecordSpans& record) {
  size_t result = 0;
  const auto* tracks = record.tracks();
  if (tracks == nullptr) {
    return 0;
  }
  for (const auto* np_track : *tracks) {
    result += track_bytes_size(*np_track);
  }
  return result;
}

static size_t largest_np_track_byte_size(const fb::RecordSpans& record) {
  size_t result = 0;
  const auto* tracks = record.tracks();
  if (tracks == nullptr) {
    return 0;
  }
  for (const auto* np_track : *tracks) {
    result = std::max(result, track_bytes_size(*np_track));
  }
  return result;
}

size_t ProgramInfo::program_data_required_memory() const {
  size_t result = 0;
  const auto nb_passes = number_of_passes();
  if (nb_passes > 1) {
    for (const auto* pass : *program_info_->pass_spans()) {
      for (const auto* record : *pass->records()) {
        result += record_np_tracks_byte_size(*record);
      }
    }
    const auto* learn = program_info_->learning_layer_span();
    if (learn) {
      result += track_bytes_size(*learn->inference_registers_span());
      result += track_bytes_size(*learn->learning_registers_span());
      result += record_np_tracks_byte_size(*learn->ram_span());
    }
  } else {
    const auto* pass = (*program_info_->pass_spans())[0];
    for (const auto* record : *pass->records()) {
      result = std::max(result, largest_np_track_byte_size(*record));
    }
    const auto* learn = program_info_->learning_layer_span();
    if (learn) {
      result =
          std::max(result, track_bytes_size(*learn->inference_registers_span()));
      result =
          std::max(result, track_bytes_size(*learn->learning_registers_span()));
      result = std::max(result, largest_np_track_byte_size(*learn->ram_span()));
    }
  }
  return result;
}

size_t ProgramInfo::fnp2_required_memory() const {
  size_t result = 0;

  for (const auto* pass : *program_info_->pass_spans()) {
    for (const auto* record : *pass->records()) {
      if (record->fnp2_track()) {
        result += track_bytes_size(record->fnp2_track()->track());
      }
    }
  }
  const auto* learn = program_info_->learning_layer_span();
  if (learn) {
    if (learn->ram_span() != nullptr && learn->ram_span()->fnp2_track()) {
      result += track_bytes_size(learn->ram_span()->fnp2_track()->track());
    }
  }
  return result;
}

akida::span<int32_t> akida::ProgramInfo::shifts() const {
  const auto* shifts_vector = program_info_->shifts();
  return {shifts_vector->data(), shifts_vector->size()};
}

akida::span<float> ProgramInfo::scales() const {
  const auto* scales_vector = program_info_->scales();
  return {scales_vector->data(), scales_vector->size()};
}

fb::IoType ProgramInfo::inputs_type() const {
  return program_info_->input_type();
}

fb::IoType ProgramInfo::outputs_type() const {
  return program_info_->output_type();
}

bool ProgramInfo::is_valid() const { return program_info_ != nullptr; }

}  // namespace akida
