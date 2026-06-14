#include "program_play.h"

#include <Arduino.h>

#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <map>

#include "akida/hardware_device.h"
#include "akida/shape.h"
#include "engine/akida_device_program_fb_generated.h"
#include "engine/akida_program_info_generated.h"
#include "engine/dma_config_ops.h"
#include "flatbuffers/flatbuffers.h"
#include "infra/hardware_driver.h"

#include "dma_desc_format.h"
#include "dma_engine_ops.h"
#include "external_mem_mgr.h"
#include "fnp2_mem_conf_reg.h"

namespace akida {
namespace program {

#ifndef AKIDA_NICLA_PROGRAM_PLAY_TRACE
#define AKIDA_NICLA_PROGRAM_PLAY_TRACE 0
#endif

namespace {

inline void akida_program_play_trace_log(const char* format, ...) {
#if AKIDA_NICLA_PROGRAM_PLAY_TRACE
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
#ifdef ARDUINO
  Serial.print(buffer);
#else
  std::printf("%s", buffer);
#endif
#else
  (void)format;
#endif
}

#if AKIDA_NICLA_PROGRAM_PLAY_TRACE
#define AKIDA_NICLA_PROGRAM_PLAY_LOG(...) akida_program_play_trace_log(__VA_ARGS__)
#else
#define AKIDA_NICLA_PROGRAM_PLAY_LOG(...) ((void)0)
#endif

ExternalTrackReader g_external_track_reader = nullptr;
ProgressHook g_progress_hook = nullptr;
constexpr size_t kExternalStageChunkBytes = 16384u;
constexpr uint32_t kExternalModelAliasBase = 0x80000000u;
constexpr uint32_t kExternalModelWindowSize = 0x00800000u;
std::map<uint64_t, dma::addr> g_persistent_external_staged_tracks;

uint32_t external_program_device_address(uint32_t address) {
  if (address >= kExternalModelAliasBase &&
      address < (kExternalModelAliasBase + kExternalModelWindowSize)) {
    // Program descriptors use the public external-flash alias range. This is
    // the last known-good shape for Nicla external execution.
    return address;
  }
  return address;
}

}  // namespace

void set_external_track_reader(ExternalTrackReader reader) {
  g_external_track_reader = reader;
}

void set_progress_hook(ProgressHook hook) {
  g_progress_hook = hook;
}

static bool uses_external_program_data(const ProgramInfo& program_info) {
  return program_info.program_data_address() != 0u;
}

static uint64_t external_track_key(const ProgramInfo& program_info,
                                   const fb::TrackSpan& track_span) {
  return (static_cast<uint64_t>(
              external_program_device_address(program_info.program_data_address()))
          << 32) |
         static_cast<uint64_t>(track_span.offset_address());
}

static dma::addr stage_external_track_on_device(
    HardwareDeviceImpl* device, const ProgramInfo& program_info,
    const fb::TrackSpan& track_span, bool persistent) {
  const auto key = external_track_key(program_info, track_span);
  if (persistent) {
    const auto existing = g_persistent_external_staged_tracks.find(key);
    if (existing != g_persistent_external_staged_tracks.end()) {
      AKIDA_NICLA_PROGRAM_PLAY_LOG(
          "[AKIDA][TRACK] external-staged-fnp2 reuse offset=0x%08lX device_addr=0x%08lX\r\n",
          static_cast<unsigned long>(track_span.offset_address()),
          static_cast<unsigned long>(existing->second));
      return existing->second;
    }
  }

  const auto track_bytes = track_span.word_size() * sizeof(uint32_t);
  const auto staged_address = device->mem()->alloc(track_bytes);

  AKIDA_NICLA_PROGRAM_PLAY_LOG(
      persistent
          ? "[AKIDA][TRACK] external-staged-fnp2 begin offset=0x%08lX words=%lu staged_addr=0x%08lX bytes=%lu\r\n"
          : "[AKIDA][TRACK] external-staged begin offset=0x%08lX words=%lu staged_addr=0x%08lX bytes=%lu\r\n",
      static_cast<unsigned long>(track_span.offset_address()),
      static_cast<unsigned long>(track_span.word_size()),
      static_cast<unsigned long>(staged_address),
      static_cast<unsigned long>(track_bytes));

  std::vector<uint8_t> chunk(kExternalStageChunkBytes, 0u);
  uint32_t copied = 0u;
  while (copied < track_bytes) {
    const size_t chunk_size =
        std::min(static_cast<size_t>(kExternalStageChunkBytes),
                 static_cast<size_t>(track_bytes - copied));
    AKIDA_NICLA_PROGRAM_PLAY_LOG(
        persistent
            ? "[AKIDA][TRACK] external-staged-fnp2 chunk-read offset=0x%08lX size=%lu\r\n"
            : "[AKIDA][TRACK] external-staged chunk-read offset=0x%08lX size=%lu\r\n",
        static_cast<unsigned long>(track_span.offset_address() + copied),
        static_cast<unsigned long>(chunk_size));
    if (!g_external_track_reader(track_span.offset_address() + copied,
                                 chunk.data(), chunk_size)) {
      panic("External staged track read failed at offset %u",
            static_cast<unsigned>(track_span.offset_address() + copied));
    }
    AKIDA_NICLA_PROGRAM_PLAY_LOG(
        persistent
            ? "[AKIDA][TRACK] external-staged-fnp2 chunk-read-done offset=0x%08lX size=%lu\r\n"
            : "[AKIDA][TRACK] external-staged chunk-read-done offset=0x%08lX size=%lu\r\n",
        static_cast<unsigned long>(track_span.offset_address() + copied),
        static_cast<unsigned long>(chunk_size));
    device->driver()->write(staged_address + copied, chunk.data(), chunk_size);
    AKIDA_NICLA_PROGRAM_PLAY_LOG(
        persistent
            ? "[AKIDA][TRACK] external-staged-fnp2 chunk-write-done addr=0x%08lX size=%lu\r\n"
            : "[AKIDA][TRACK] external-staged chunk-write-done addr=0x%08lX size=%lu\r\n",
        static_cast<unsigned long>(staged_address + copied),
        static_cast<unsigned long>(chunk_size));
    copied += static_cast<uint32_t>(chunk_size);
    if (g_progress_hook != nullptr) {
      g_progress_hook(millis());
    }
    yield();
  }

  AKIDA_NICLA_PROGRAM_PLAY_LOG(
      persistent
          ? "[AKIDA][TRACK] external-staged-fnp2 offset=0x%08lX words=%lu device_addr=0x%08lX\r\n"
          : "[AKIDA][TRACK] external-staged offset=0x%08lX words=%lu device_addr=0x%08lX\r\n",
      static_cast<unsigned long>(track_span.offset_address()),
      static_cast<unsigned long>(track_span.word_size()),
      static_cast<unsigned long>(staged_address));

  if (persistent) {
    g_persistent_external_staged_tracks[key] = staged_address;
  }
  return staged_address;
}

static dma::addr track_address_on_device(HardwareDeviceImpl* device,
                                         const ProgramInfo& program_info,
                                         const fb::TrackSpan& track_span,
                                         const uint8_t** local_address) {
  if (uses_external_program_data(program_info)) {
    if (local_address != nullptr) {
      *local_address = nullptr;
    }
    return external_program_device_address(program_info.program_data_address()) +
           track_span.offset_address();
  }

  const auto* track_buffer = program_info.program().data + track_span.offset_address();
  if (local_address != nullptr) {
    *local_address = track_buffer;
  }
  const auto track_bytes_size = track_span.word_size() * sizeof(uint32_t);
  return device->external_mem()->track_and_put_on_device_if_required(track_buffer,
                                                                     track_bytes_size);
}

static void rewind_fnp2_track(HardwareDeviceImpl* device,
                              const ProgramInfo& program_info,
                              const fb::FNP2TrackInfo& track) {
  if (uses_external_program_data(program_info)) {
    return;
  }

  device->external_mem()->release(program_info.program().data +
                                  track.track().offset_address());
}

static void rewind_np_track(HardwareDeviceImpl* device,
                            const ProgramInfo& program_info,
                            const fb::TrackSpan& track, bool multi_pass) {
  if (uses_external_program_data(program_info)) {
    return;
  }

  if (multi_pass) {
    // in multi pass, free config header allocated with track as id
    device->external_mem()->release(program_info.program().data +
                                    track.offset_address());
  }
}

static void rewind_record(HardwareDeviceImpl* device,
                          const ProgramInfo& program_info,
                          const fb::RecordSpans& record, bool multi_pass) {
  // rewind fnp2 track if it is there
  const auto* fnp2_track = record.fnp2_track();
  if (fnp2_track) {
    rewind_fnp2_track(device, program_info, *fnp2_track);
  }
  // rewind all normal tracks
  const auto* np_tracks = record.tracks();
  if (np_tracks != nullptr) {
    for (auto np_track = np_tracks->rbegin(); np_track != np_tracks->rend();
         ++np_track) {
      rewind_np_track(device, program_info, **np_track, multi_pass);
    }
  }
}

static void write_np_track_descriptor(HardwareDriver* driver,
                                      dma::addr track_addr_on_device,
                                      uint32_t track_word_size,
                                      dma::addr descriptor_address) {
  // format descriptor
  constexpr uint32_t output_addr = 0;  // not used for write
  auto descriptor = dma::format_config_desc(dma::kDescConfigDirectionWrite,
                                            track_addr_on_device, output_addr,
                                            track_word_size);
  // write descriptor in its place
  driver->write(descriptor_address, descriptor.data(),
                descriptor.size() * sizeof(dma::Descriptor::value_type));
}

static void generate_reading_descriptor_from_np_track(
    HardwareDeviceImpl* device, const ProgramInfo& program_info,
    const fb::TrackSpan& track) {
  const uint8_t* local_address = nullptr;
  const auto track_addr =
      track_address_on_device(device, program_info, track, &local_address);
  const auto input_addr = track_addr;
  const auto output_addr = track_addr + dma::kConfigWritePacketOffset;

  // format descriptor
  const auto descriptor =
      dma::format_config_desc(dma::kDescConfigDirectionRead, input_addr,
                              output_addr, dma::kConfigWriteHdrWordLen);
  // enqueue it at extra descriptor location
  dma::enqueue_extra_descriptor(device->driver(), device->dma_config(),
                                descriptor);
}

struct TrackedSpan {
  const uint8_t* local_address;
  dma::addr device_address;
  size_t byte_size;
  bool staged_external;
};

static inline TrackedSpan write_track_on_device(
    HardwareDeviceImpl* device, const ProgramInfo& program_info,
    const fb::TrackSpan& track_span) {
  TrackedSpan result{};
  result.local_address = nullptr;
  result.byte_size = track_span.word_size() * sizeof(uint32_t);
  result.staged_external = false;

  if (uses_external_program_data(program_info) &&
      g_external_track_reader != nullptr) {
    result.staged_external = true;
    result.device_address =
        stage_external_track_on_device(device, program_info, track_span, false);
  } else {
    result.device_address = track_address_on_device(device, program_info,
                                                    track_span,
                                                    &result.local_address);
  }

  if (uses_external_program_data(program_info) && !result.staged_external) {
    AKIDA_NICLA_PROGRAM_PLAY_LOG(
        "[AKIDA][TRACK] external offset=0x%08lX words=%lu device_addr=0x%08lX\r\n",
        static_cast<unsigned long>(track_span.offset_address()),
        static_cast<unsigned long>(track_span.word_size()),
        static_cast<unsigned long>(result.device_address));
  } else if (!result.staged_external) {
    AKIDA_NICLA_PROGRAM_PLAY_LOG(
        "[AKIDA][TRACK] host offset=0x%08lX words=%lu device_addr=0x%08lX local=%p\r\n",
        static_cast<unsigned long>(track_span.offset_address()),
        static_cast<unsigned long>(track_span.word_size()),
        static_cast<unsigned long>(result.device_address),
        static_cast<const void*>(result.local_address));
  }
  return result;
}

static dma::addr play_track(HardwareDeviceImpl* device,
                            const ProgramInfo& program_info,
                            const fb::TrackSpan& track_span, bool single_pass) {
  // 1st write track data, put buffer on device, and get its address
  const auto tracked_span = write_track_on_device(device, program_info, track_span);

  // generate descriptor for the track
  const auto descriptor = dma::format_config_desc(
      dma::kDescConfigDirectionWrite, tracked_span.device_address, 0,
      track_span.word_size());
  // enqueue descriptor
  const auto descriptor_address = dma::enqueue_descriptor(
      device->driver(), device->dma_config().engine, descriptor);

  if (single_pass) {
    // in single pass, we need to wait for descriptor to complete
    dma::wait_config_dma_descriptor_complete(device->driver(),
                                             device->dma_config());
    // then release track data
    if (tracked_span.staged_external) {
      device->mem()->free(tracked_span.device_address);
    } else if (tracked_span.local_address != nullptr) {
      device->external_mem()->release(tracked_span.local_address);
    }
  }
  return descriptor_address;
}

static void play_fnp2_track(HardwareDeviceImpl* device,
                            const ProgramInfo& program_info,
                            const fb::FNP2TrackInfo& track_info) {
  auto* driver = device->driver();
  const uint8_t* local_address = nullptr;
  uint32_t address =
      track_address_on_device(device, program_info, track_info.track(), &local_address);
  if (uses_external_program_data(program_info) &&
      g_external_track_reader != nullptr) {
    address =
        stage_external_track_on_device(device, program_info, track_info.track(), true);
  }

  // Now write external address used for this NP in the dedicated conf register
  // Note that there are 4 registers where the weights adress can be stored.
  // This works because currently existing mesh designs only contain one node
  // with 4 FNPs. Each NP will use the content of the register indexed by the ID
  // of the NP.
  // If at some point a mesh is created with a different layout, this might
  // raise an issue.
  // Also, this means that in multipass this register can only be used once per
  // program, and the FNP2 cannot be reused later, because that would require
  // updating the register value with another address, and there is no way to do
  // that.
  const auto np_id = track_info.np().id();
  AKIDA_NICLA_PROGRAM_PLAY_LOG(
      "[AKIDA][TRACK] fnp2 np_id=%lu offset=0x%08lX device_addr=0x%08lX\r\n",
      static_cast<unsigned long>(np_id),
      static_cast<unsigned long>(track_info.track().offset_address()),
      static_cast<unsigned long>(address));
  auto fnp2_mem_conf_reg_addr =
      fnp2_memory_conf(driver->top_level_reg(), np_id);
  driver->write32(fnp2_mem_conf_reg_addr, address);
}

static dma::addr play_record(HardwareDeviceImpl* device,
                             const ProgramInfo& program_info,
                             const fb::RecordSpans& record_spans,
                             bool single_pass) {
  dma::addr descriptor_address = 0;
  // play all np tracks
  const auto* tracks_spans = record_spans.tracks();
  if (tracks_spans != nullptr) {
    for (const auto* track_span : *tracks_spans) {
      descriptor_address = play_track(device, program_info, *track_span, single_pass);
    }
  }

  // play FNP2 track if there is one
  const auto* fnp2_track = record_spans.fnp2_track();
  if (fnp2_track != nullptr) {
    play_fnp2_track(device, program_info, *fnp2_track);
  }

  return descriptor_address;
}

static void generate_reading_descriptor_from_record(
    HardwareDeviceImpl* device, const ProgramInfo& program_info,
    const fb::RecordSpans& record) {
  if (record.fnp2_track())
    panic("Cannot use descriptors to read the FNP2 memory.");
  // play all normal tracks
  const auto* np_tracks = record.tracks();
  assert(np_tracks->size() == 1 &&
         "Learning should use a single NP, so there should be a single track");
  generate_reading_descriptor_from_np_track(device, program_info, *np_tracks->Get(0));
}

static void play_epg_track(HardwareDriver* driver, uint32_t epg_base,
                           uint32_t address, uint32_t data) {
  driver->write32(epg_base + address, data);
}

void rewind(HardwareDeviceImpl* device, const ProgramInfo& program_info) {
  if (uses_external_program_data(program_info)) {
    for (auto iter = g_persistent_external_staged_tracks.rbegin();
         iter != g_persistent_external_staged_tracks.rend(); ++iter) {
      device->mem()->free(iter->second);
    }
    g_persistent_external_staged_tracks.clear();
    return;
  }

  const auto* program_info_fb = program_info.program_info_;
  const auto* program_base = program_info.program().data;

  const auto& passes = *program_info_fb->pass_spans();
  int passes_size = passes.size();
  bool multi_pass = passes_size > 1;

  const auto learn = program_info_fb->learning_layer_span();
  if (learn) {
    if (multi_pass) {
      // in multi pass, both learning & inference registers are written to the
      // device
      rewind_np_track(device, program_info, *learn->learning_registers_span(),
                      multi_pass);
      rewind_np_track(device, program_info, *learn->inference_registers_span(),
                      multi_pass);
    } else {
      if (device->learn_enabled()) {
        rewind_np_track(device, program_info, *learn->learning_registers_span(),
                        multi_pass);
      } else {
        rewind_np_track(device, program_info,
                        *learn->inference_registers_span(), multi_pass);
      }
    }
    rewind_record(device, program_info, *learn->ram_span(), multi_pass);
  }

  // rewind in reverse order
  for (auto pass = passes.rbegin(); pass != passes.rend(); ++pass) {
    const auto& layer_records = *pass->records();
    for (auto record = layer_records.rbegin(); record != layer_records.rend();
         ++record) {
      rewind_record(device, program_info, **record, multi_pass);
    }
  }

  if (multi_pass) {
    // free up dummy config header
    device->external_mem()->release(program_info.program_->dummy_desc_hdr());
  }
}

static dma::addr dma_config_header_dummy(HardwareDeviceImpl* device,
                                         const akida::fb::Program* program) {
  auto dummy_header = program->dummy_desc_hdr();
  // make sure flatbuffer struct is the same size as header
  static_assert(sizeof(*dummy_header) == dma::kConfigWritePacketOffset,
                "DmaConfigHeader should be the same size as "
                "kConfigWriteHdrWordLen");
  // put buffer on device, and get its address
  auto mem = device->external_mem()->track_and_put_on_device_if_required(
      dummy_header, sizeof(*dummy_header));

  return mem;
}

static void write_dummy_descs(HardwareDeviceImpl* device, dma::addr dummy_input,
                              dma::addr dummy_output,
                              uint32_t num_dummy_descs) {
  // Dummy descriptor is a read of size 1. Descriptor size is the header size
  auto dummy_desc =
      dma::format_config_desc(dma::kDescConfigDirectionRead, dummy_input,
                              dummy_output, dma::kConfigWriteHdrWordLen);
  for (uint32_t j = 0; j < num_dummy_descs; j++) {
    dma::enqueue_descriptor(device->driver(), device->dma_config().engine,
                            dummy_desc);
  }
}

static inline uint32_t epg_reg_base(const uint32_t top_level_reg_base) {
  constexpr uint32_t EPG_REG_BASE = 0x00040000;
  return top_level_reg_base + EPG_REG_BASE;
}

static void play_epg(HardwareDeviceImpl* device,
                     const fb::ProgramInfo* program_info) {
  // Apply EPG program
  const auto* epg_tracks = program_info->epg_tracks();
  if (epg_tracks) {
    auto epg_tracks_size = epg_tracks->size();
    auto driver = device->driver();
    auto epg_base = epg_reg_base(driver->top_level_reg());
    for (uint32_t i = 0; i < epg_tracks_size; i++) {
      const auto& epg_track = epg_tracks->Get(i);
      play_epg_track(driver, epg_base, epg_track->register_offset(),
                     epg_track->value());
    }
  }
}

void play_single_pass(HardwareDeviceImpl* device,
                      const ProgramInfo& program_info) {
  const auto* program_info_fb = program_info.program_info_;
  assert(program_info_fb->pass_spans()->size() == 1);
  const auto* records_spans = program_info_fb->pass_spans()->Get(0)->records();
  const auto records_count = records_spans->size();

  AKIDA_NICLA_PROGRAM_PLAY_LOG(
      "[AKIDA][PROGRAM] single-pass begin records=%lu\r\n",
      static_cast<unsigned long>(records_count));

  // play all records
  for (uint32_t i = 0; i < records_count; i++) {
    AKIDA_NICLA_PROGRAM_PLAY_LOG(
        "[AKIDA][PROGRAM] single-pass record %lu/%lu\r\n",
        static_cast<unsigned long>(i + 1),
        static_cast<unsigned long>(records_count));
    play_record(device, program_info, *records_spans->Get(i), true);
  }

  const auto* learn = program_info_fb->learning_layer_span();
  if (learn) {
    AKIDA_NICLA_PROGRAM_PLAY_LOG(
        "[AKIDA][PROGRAM] single-pass learning config\r\n");
    // In single pass, we just play the correct register track, depending on
    // learning activated or not, then play the weights record
    const auto* registers_span = device->learn_enabled()
                                     ? learn->learning_registers_span()
                                     : learn->inference_registers_span();
    play_track(device, program_info, *registers_span, true);
    play_record(device, program_info, *learn->ram_span(), true);
  }
  play_epg(device, program_info.program_info_);
  AKIDA_NICLA_PROGRAM_PLAY_LOG("[AKIDA][PROGRAM] single-pass done\r\n");
}

void play_multi_pass(HardwareDeviceImpl* device,
                     const ProgramInfo& program_info,
                     MultiPassMemory* multipass_memory) {
  const auto* program = program_info.program_;
  const auto* program_info_fb = program_info.program_info_;
  const auto* learn = program_info_fb->learning_layer_span();
  const auto& passes = *program_info_fb->pass_spans();
  uint32_t passes_size = passes.size();
  // In multi pass mode, there will always be at least 2 passes
  assert(passes_size >= 2);
  if (uses_external_program_data(program_info)) {
    panic("program_external_data checkpoint 2 does not support multipass programs yet");
  }

  // estimate memory required to hold passes descriptors.
  const auto max_num_desc_pass = program_info.number_of_descriptors_per_pass();

  // use program to allocate dummy config, input and output space
  auto dummy_input = dma_config_header_dummy(device, program);

  uint32_t np_tracks_played = 0;

  // now that we have the memory, we can fill the descriptors
  for (uint32_t i = 0; i < passes_size; i++) {
    const auto& layer_records = *passes[i]->records();
    uint32_t records_size = layer_records.size();
    np_tracks_played = 0;
    for (uint32_t j = 0; j < records_size; j++) {
      const auto* record_span = layer_records[j];

      // get number of NP tracks (corresponding to number of DMA descriptors).
      uint32_t np_tracks_size = record_span->tracks()->size();
      play_record(device, program_info, *record_span, false);
      np_tracks_played += np_tracks_size;
    }

    if (i == passes_size - 1 && learn) {
      const auto* inference_registers = learn->inference_registers_span();
      const auto* learn_registers = learn->learning_registers_span();
      const auto* ram = learn->ram_span();

      // number of descriptors generated is always 1 + another one if ram has NP
      // track
      auto np_tracks_size = 1 + (ram->tracks() ? ram->tracks()->size() : 0);

      auto learn_desc_address = play_track(device, program_info,
                                           *inference_registers, false);
      // store the address of descriptor that correspond to the learning layer
      // registers because we will need to edit this descriptor to make it point
      // to the learning registers or inference registers when enable/disable
      // learning
      multipass_memory->update_learn_descriptor_addr(learn_desc_address);
      write_track_on_device(device, program_info, *learn_registers);
      play_record(device, program_info, *ram, false);
      np_tracks_played += np_tracks_size;
    }

    // fill unused pass descriptors with "dummy" descriptors for this pass
    assert(max_num_desc_pass >= np_tracks_played);
    uint32_t num_dummy_descs = max_num_desc_pass - np_tracks_played;
    write_dummy_descs(device, dummy_input, multipass_memory->dummy_output_addr,
                      num_dummy_descs);
  }

  // Add an extra descriptor to copy the learned memory
  if (program_info.learning_on_fnp3()) {
    generate_reading_descriptor_from_record(device, program_info, *learn->ram_span());
  }
  play_epg(device, program_info.program_info_);
}

void configure_learning_mode_single_pass(HardwareDeviceImpl* device,
                                         const ProgramInfo& program_info,
                                         bool learn_en) {
  const auto* program_info_fb = program_info.program_info_;
  assert(program_info_fb->pass_spans()->size() == 1);

  const auto learn = program_info_fb->learning_layer_span();
  assert(learn);
  const auto& new_register = learn_en ? *learn->learning_registers_span()
                                      : *learn->inference_registers_span();
  // we can just play the new track, there is no need to rewind anything in
  // single pass
  play_track(device, program_info, new_register, true);
}

void configure_learning_mode_multi_pass(HardwareDeviceImpl* device,
                                        const ProgramInfo& program_info,
                                        const MultiPassMemory& multipass_memory,
                                        bool learn_en) {
  const auto* program_info_fb = program_info.program_info_;
  assert(program_info_fb->pass_spans()->size() > 1);
  if (uses_external_program_data(program_info)) {
    panic("program_external_data checkpoint 2 does not support multipass learning config yet");
  }
  // This function will edit the descriptor at learn_descriptor_addr to make it
  // point to either learning or inference registers
  assert(multipass_memory.learn_descriptor_addr != 0);
  const auto learn = program_info_fb->learning_layer_span();
  assert(learn);

  // get the correct track depending on learning
  const auto* registers_track = learn_en ? learn->learning_registers_span()
                                         : learn->inference_registers_span();
  const auto registers_address = device->external_mem()->tracked(
      program_info.program().data + registers_track->offset_address());
  // Overwrite the descriptor
  write_np_track_descriptor(device->driver(), registers_address,
                            registers_track->word_size(),
                            multipass_memory.learn_descriptor_addr);
}

void update_learn_mem(HardwareDeviceImpl* device, const uint8_t* program_data,
                      const uint32_t* ram_dump) {
  auto* program = fb::GetSizePrefixedProgram(program_data);
  auto learning_layer = program->learning_layer();
  if (!learning_layer) {
    panic(
        "Learn memory update requires a device programmed with learning "
        "layers");
  }

  // detect ram size
  auto size = learning_layer->learn_mem_size();
  // detect if FNP2 or FNP3
  auto record = learning_layer->ram();
  assert(record);
  auto fnp2_track = record->fnp2_track();
  if (fnp2_track) {
    // get memory address for this track
    auto mem_addr = device->external_mem()->tracked(fnp2_track->data()->data());
    // update memory
    device->driver()->write(mem_addr, ram_dump, size * sizeof(uint32_t));
  } else {
    auto update_learn_mem_hdr = learning_layer->update_learn_mem_hdr();
    // Note: for now a copy is necessary to update learn memory, to have config
    // header placed just before memory. In the future, a possible optimization
    // could be returning the header when reading the memory.
    std::vector<dma::w32> sram;
    sram.reserve(dma::kConfigWriteHdrWordLen + size);
    // first copy header in vector
    sram.push_back(update_learn_mem_hdr->w1());
    sram.push_back(update_learn_mem_hdr->w2());
    // then copy ram dump
    sram.insert(sram.begin() + dma::kConfigWriteHdrWordLen, ram_dump,
                ram_dump + size);
    assert(sram.size() == dma::kConfigWriteHdrWordLen + size &&
           "SRAM learn memory size mismatch");
    // now do transfer
    device->dma_config_write(sram.data(), sram.size());
  }
}

void learn_mem(HardwareDeviceImpl* device, const uint8_t* program_data,
               uint32_t* ram_dump) {
  auto* program = fb::GetSizePrefixedProgram(program_data);
  auto learning_layer = program->learning_layer();
  if (!learning_layer) {
    panic("Learn memory retrieval requires a program from learning layers");
  }

  // detect ram size
  auto size = learning_layer->learn_mem_size();
  // detect if FNP2 or FNP3
  auto record = learning_layer->ram();
  assert(record);
  auto fnp2_track = record->fnp2_track();
  if (fnp2_track) {
    // get memory address for this track
    auto mem_addr = device->external_mem()->tracked(fnp2_track->data()->data());
    device->driver()->read(mem_addr, ram_dump, size * sizeof(dma::w32));
  } else {
    // In multi pass we can directly read in the program.
    if (program->passes()->size() > 1) {
      auto* tracks = record->np_tracks();
      assert(tracks->size() == 1);
      auto ram_addr =
          device->external_mem()->tracked(tracks->Get(0)->data()->data());
      // Skip the 2 words of DMA read header
      device->driver()->read(ram_addr + dma::kConfigWritePacketOffset, ram_dump,
                             size * sizeof(dma::w32));
    } else {
      // in single pass when record is FNP3: read SRAM
      auto np = learning_layer->np();
      np::Ident ident{np->col(), np->row(), np->id()};
      device->dma_config_read(ram_dump, ident, dma::Target::FnpWeights, 0,
                              size);
    }
  }
}

}  // namespace program
}  // namespace akida
