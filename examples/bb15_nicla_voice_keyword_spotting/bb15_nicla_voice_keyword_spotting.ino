#include <Arduino.h>

#include <NDP.h>

// nicla_voice and nicla_sense share the Arduino NICLA variant, so this guard
// can only reject other families. Building this for Nicla Sense ME compiles but
// fails at NDP.begin(), because that board has no NDP120.
#ifndef ARDUINO_NICLA
#error "bb15_nicla_voice_keyword_spotting requires Arduino Nicla Voice."
#endif

namespace {

// This example owns microphone capture, block framing and USB framing. The
// Nicla Voice microphone reaches the NDP120 only, never the nRF52832, so the
// bundled NDP library is the single route to audio (Nicla Voice datasheet
// section 4.7). BB15 inference arrives in a later stage of this demo.
constexpr uint32_t kSerialBaud = 921600u;
constexpr uint32_t kBootSettleMs = 250u;

// The NDP120 firmware packages already stored in the board's on-board QSPI
// flash. All three are required: with only the MCU and DSP packages loaded the
// audio holding tank never advances, so extractData keeps returning the same
// stale chunk. The neural-network package carries the DSP audio flow
// configuration that starts the tank; this demo uses none of its classes.
constexpr const char* kNdpMcuFirmware = "mcu_fw_120_v90.synpkg";
constexpr const char* kNdpDspFirmware = "dsp_firmware_v90.synpkg";
constexpr const char* kNdpAudioFlowPackage = "alexa_334_NDP120_B0_v11_v90.synpkg";

// The NDP120 publishes one 24 ms chunk of 16 kHz mono PCM at a time and holds
// it until the next one replaces it, so polling must be paced: too slow loses
// audio, too fast burns SPI time re-reading a chunk already taken. Each chunk
// carries a four-byte annotation after the samples whose third byte is a
// monotonic counter, which is what makes a fresh chunk recognizable.
constexpr size_t kChunkBufferBytes = 1024u;
constexpr size_t kAnnotationBytes = 4u;
constexpr size_t kAnnotationCounterOffset = 2u;
constexpr uint32_t kChunkPeriodMs = 24u;
constexpr uint32_t kChunkPollLeadMs = 3u;
constexpr uint32_t kChunkRetryMs = 2u;
// A single failed read is unremarkable. A run of them is a real transport fault
// worth telling the desktop tool about, rather than leaving it to guess from a
// stalled view.
constexpr uint8_t kChunkFailureLimit = 25u;

// Front-end values taken from spark and hard-coded here on purpose: this demo
// has no parameter surface. Sample rate, hop and block size come from
// spark/source/Kconfig (SAMPLING_RATE, MFCC_SAMPLE_COUNT, AUDIO_BLOCK_MS); the
// gating and decision values come from spark/source/core/common/kws_config.c.
constexpr uint32_t kSampleRateHz = 16000u;
constexpr uint16_t kBlockSamples = 960u;
constexpr uint16_t kSpectrogramFrames = 49u;
constexpr uint8_t kSpectrogramCoefficients = 10u;
constexpr uint16_t kRmsThreshold = 550u;
constexpr uint16_t kSpeechActiveTimeMs = 1300u;
constexpr uint16_t kSmoothingAlphaQ15 = 22938u;
constexpr uint16_t kScoreThresholdQ15 = 16384u;
constexpr uint16_t kDebounceMs = 300u;
constexpr uint8_t kChimingThreshold = 3u;

// spark's DC blocking high-pass, from dc_block_process() in
// spark/source/core/interface/audio/pdm_mic.c. Its output is what spark takes
// the RMS of and what it feeds the MFCC, so the RMS threshold above only means
// the same thing here if the same filter runs first.
constexpr int32_t kDcBlockAlphaQ15 = 32700;

// One min/max pair per 10 samples. spark sends the same shape of envelope to
// its phone app, at 32 pairs per block; the extra resolution here is affordable
// because this link is a 921600 baud UART rather than BLE.
constexpr uint16_t kWaveformPoints = 96u;
constexpr uint16_t kWaveformWindowSamples = kBlockSamples / kWaveformPoints;

// USB protocol v1. The header is the one the Nicla Vision human-detection demo
// defines: "BB15", version, type, payload size (uint32 little-endian). The
// three host commands are identical; the two device message types are new so a
// mismatched desktop tool fails to recognize them instead of misreading a
// camera payload as audio.
constexpr uint8_t kProtocolMagic[] = {'B', 'B', '1', '5'};
constexpr uint8_t kProtocolVersion = 1u;
enum class PacketType : uint8_t {
  StartStream = 1u,
  StopStream = 2u,
  RequestConfig = 3u,
  Error = 0x83u,
  AudioConfig = 0x84u,
  AudioResult = 0x85u,
};
constexpr size_t kPacketHeaderBytes = 10u;
constexpr size_t kAudioConfigBytes = 24u;
constexpr size_t kAudioResultMetadataBytes = 28u;

// Prediction is not wired up at this stage of the demo. The fields exist in the
// packet so the layout is final, and carry these placeholders until they do.
constexpr uint8_t kNoPrediction = 0xFFu;
constexpr uint8_t kNoScores = 0u;
constexpr uint8_t kNoMfccFrames = 0u;
constexpr uint8_t kSpeechGatingInactive = 0u;
constexpr uint8_t kNoChiming = 0u;
constexpr uint16_t kNoTimingMs = 0u;
constexpr uint8_t kStatusOk = 0u;
constexpr uint8_t kReservedByte = 0u;

constexpr const char* kSketchName = "bb15_nicla_voice_keyword_spotting";
constexpr const char* kLogPrefix = "[bb15_nicla_voice_keyword_spotting]";

/** @brief One completed 60 ms audio block, ready to stream to the desktop tool. */
struct AudioBlock {
  uint32_t sequence = 0u;
  uint32_t deviceMs = 0u;
  uint16_t rms = 0u;
  uint16_t peak = 0u;
  uint16_t captureMs = 0u;
};

/**
 * @brief How far chunk synchronization has got since the last stream start.
 *
 * The NDP120's holding tank keeps running while no stream is being served, so
 * the first counter gaps a fresh stream sees say nothing about continuity.
 */
enum class ChunkSync : uint8_t {
  NoCounter,
  Seeded,
  Running,
};

/** @brief Running state of spark's first-order DC blocking high-pass. */
struct DcBlockState {
  int32_t previousInput = 0;
  int32_t previousOutput = 0;
};

alignas(4) uint8_t g_chunk[kChunkBufferBytes];
int16_t g_block[kBlockSamples];
int16_t g_waveform[2u * kWaveformPoints];
DcBlockState g_dc_block;
size_t g_block_fill = 0u;
int64_t g_block_energy = 0;
uint16_t g_block_peak = 0u;
uint32_t g_block_capture_ms = 0u;
uint8_t g_last_chunk_counter = 0u;
uint8_t g_chunk_failures = 0u;
ChunkSync g_chunk_sync = ChunkSync::NoCounter;
uint32_t g_next_chunk_poll_ms = 0u;
uint16_t g_dropped_chunks = 0u;
uint32_t g_sequence = 0u;
bool g_streaming = false;

/**
 * @brief Clamp a millisecond duration into the packet's 16-bit field.
 *
 * @param value  Duration in milliseconds.
 * @return The value, saturated at 65535.
 */
uint16_t clamp_u16(uint32_t value) {
  return value > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(value);
}

/**
 * @brief Show the demo's state on the RGB LED.
 *
 * @param color  Colour to display, or `off` to clear it.
 */
void set_led(RGBColors color) { nicla::leds.setColor(color); }

/**
 * @brief Report a fatal setup failure and blink the LED forever.
 *
 * @param stage  Name of the setup step that failed.
 */
void halt_forever(const char* stage) {
  Serial.print(kLogPrefix);
  Serial.print(" result=FAIL stage=");
  Serial.println(stage);
  for (;;) {
    set_led(red);
    delay(50);
    set_led(off);
    delay(950);
  }
}

/**
 * @brief Write a little-endian unsigned 16-bit value to the host.
 *
 * @param value  Value to write.
 */
void write_u16(uint16_t value) {
  Serial.write(static_cast<uint8_t>(value & 0xFFu));
  Serial.write(static_cast<uint8_t>((value >> 8) & 0xFFu));
}

/**
 * @brief Write a little-endian unsigned 32-bit value to the host.
 *
 * @param value  Value to write.
 */
void write_u32(uint32_t value) {
  for (uint8_t shift = 0u; shift < 32u; shift += 8u) {
    Serial.write(static_cast<uint8_t>((value >> shift) & 0xFFu));
  }
}

/**
 * @brief Write a little-endian signed 16-bit value to the host.
 *
 * @param value  Value to write.
 */
void write_i16(int16_t value) { write_u16(static_cast<uint16_t>(value)); }

/**
 * @brief Write the fixed ten-byte protocol header.
 *
 * @param type           Message type that follows.
 * @param payload_bytes  Size of the payload after the header.
 */
void write_packet_header(PacketType type, uint32_t payload_bytes) {
  Serial.write(kProtocolMagic, sizeof(kProtocolMagic));
  Serial.write(kProtocolVersion);
  Serial.write(static_cast<uint8_t>(type));
  write_u32(payload_bytes);
}

/**
 * @brief Send the pipeline description the desktop tool needs to draw itself.
 */
void send_audio_config_packet() {
  write_packet_header(PacketType::AudioConfig,
                      static_cast<uint32_t>(kAudioConfigBytes));
  write_u32(kSampleRateHz);
  write_u16(kBlockSamples);
  write_u16(kWaveformPoints);
  write_u16(kSpectrogramFrames);
  Serial.write(kSpectrogramCoefficients);
  Serial.write(kNoScores);
  write_u16(kRmsThreshold);
  write_u16(kSpeechActiveTimeMs);
  write_u16(kSmoothingAlphaQ15);
  write_u16(kScoreThresholdQ15);
  write_u16(kDebounceMs);
  Serial.write(kChimingThreshold);
  Serial.write(kReservedByte);
  Serial.flush();
}

/**
 * @brief Report a device-side failure to the desktop tool.
 *
 * @param status  Syntiant interface library status for the failed operation.
 */
void send_error_packet(uint8_t status) {
  write_packet_header(PacketType::Error, 1u);
  Serial.write(status);
  Serial.flush();
}

/**
 * @brief Send one completed audio block and its waveform envelope.
 *
 * @param block  Statistics for the block held in `g_waveform`.
 */
void send_audio_result_packet(const AudioBlock& block) {
  const uint32_t waveform_bytes =
      static_cast<uint32_t>(sizeof(int16_t)) * 2u * kWaveformPoints;
  write_packet_header(PacketType::AudioResult,
                      static_cast<uint32_t>(kAudioResultMetadataBytes) +
                          waveform_bytes);
  write_u32(block.sequence);
  write_u32(block.deviceMs);
  write_u16(block.rms);
  write_u16(block.peak);
  write_u16(g_dropped_chunks);
  write_u16(block.captureMs);
  write_u16(kNoTimingMs);
  write_u16(kNoTimingMs);
  Serial.write(kSpeechGatingInactive);
  Serial.write(kStatusOk);
  Serial.write(kNoPrediction);
  Serial.write(kNoScores);
  Serial.write(static_cast<uint8_t>(kWaveformPoints));
  Serial.write(kNoMfccFrames);
  Serial.write(kNoChiming);
  Serial.write(kReservedByte);
  for (uint16_t point = 0u; point < 2u * kWaveformPoints; ++point) {
    write_i16(g_waveform[point]);
  }
  Serial.flush();
}

/**
 * @brief Read at most one host command from the serial input.
 *
 * Commands are zero-payload packets. Parsing byte by byte lets the boot banner
 * and the NDP library's own progress output pass through harmlessly when they
 * are still buffered as the desktop tool opens the port.
 *
 * @return The command received, or type 0 when no complete command is pending.
 */
PacketType poll_host_command() {
  static uint8_t header[kPacketHeaderBytes];
  static size_t header_size = 0u;
  while (Serial.available() > 0) {
    const uint8_t value = static_cast<uint8_t>(Serial.read());
    if (header_size < sizeof(kProtocolMagic)) {
      if (value == kProtocolMagic[header_size]) {
        header[header_size++] = value;
      } else {
        header_size = value == kProtocolMagic[0] ? 1u : 0u;
        if (header_size == 1u) {
          header[0] = value;
        }
      }
      continue;
    }
    header[header_size++] = value;
    if (header_size != kPacketHeaderBytes) {
      continue;
    }
    header_size = 0u;
    if (header[4] != kProtocolVersion) {
      continue;
    }
    const uint32_t payload = static_cast<uint32_t>(header[6]) |
                             (static_cast<uint32_t>(header[7]) << 8) |
                             (static_cast<uint32_t>(header[8]) << 16) |
                             (static_cast<uint32_t>(header[9]) << 24);
    if (payload != 0u) {
      continue;
    }
    return static_cast<PacketType>(header[5]);
  }
  return static_cast<PacketType>(0u);
}

/**
 * @brief Apply spark's DC blocking high-pass to captured samples in place.
 *
 * The filter is a first-order IIR carrying state between calls, so filtering
 * the stream in 24 ms chunks gives the same result as spark filtering it in
 * 60 ms blocks.
 *
 * @param samples  Samples to filter, overwritten with the filtered signal.
 * @param count    Number of samples to filter.
 */
void remove_dc_offset(int16_t* samples, size_t count) {
  for (size_t index = 0u; index < count; ++index) {
    int32_t output = samples[index] - g_dc_block.previousInput +
                     ((kDcBlockAlphaQ15 * g_dc_block.previousOutput) >> 15);
    g_dc_block.previousInput = samples[index];
    g_dc_block.previousOutput = output;
    if (output > 32767) {
      output = 32767;
    }
    if (output < -32768) {
      output = -32768;
    }
    samples[index] = static_cast<int16_t>(output);
  }
}

/**
 * @brief Reduce the completed block to a min/max envelope for the live view.
 */
void compute_waveform_envelope() {
  for (uint16_t point = 0u; point < kWaveformPoints; ++point) {
    const int16_t* window = &g_block[point * kWaveformWindowSamples];
    int16_t lowest = INT16_MAX;
    int16_t highest = INT16_MIN;
    for (uint16_t index = 0u; index < kWaveformWindowSamples; ++index) {
      if (window[index] < lowest) {
        lowest = window[index];
      }
      if (window[index] > highest) {
        highest = window[index];
      }
    }
    g_waveform[2u * point] = lowest;
    g_waveform[2u * point + 1u] = highest;
  }
}

/**
 * @brief Stream the filled block and start collecting the next one.
 */
void publish_filled_block() {
  AudioBlock block;
  block.sequence = g_sequence++;
  block.deviceMs = millis();
  block.rms = static_cast<uint16_t>(
      sqrtf(static_cast<float>(g_block_energy) / kBlockSamples));
  block.peak = g_block_peak;
  block.captureMs = clamp_u16(g_block_capture_ms);

  compute_waveform_envelope();
  send_audio_result_packet(block);

  g_block_fill = 0u;
  g_block_energy = 0u;
  g_block_peak = 0u;
  g_block_capture_ms = 0u;
}

/**
 * @brief Append filtered samples to the pending block, streaming it when full.
 *
 * @param samples  Filtered samples to append.
 * @param count    Number of samples available.
 */
void accumulate_samples(const int16_t* samples, size_t count) {
  size_t consumed = 0u;
  while (consumed < count) {
    const size_t room = kBlockSamples - g_block_fill;
    const size_t taking = (count - consumed) < room ? (count - consumed) : room;
    for (size_t index = 0u; index < taking; ++index) {
      const int16_t sample = samples[consumed + index];
      g_block[g_block_fill + index] = sample;
      g_block_energy += static_cast<int32_t>(sample) * sample;
      const uint16_t magnitude = static_cast<uint16_t>(sample < 0 ? -sample : sample);
      if (magnitude > g_block_peak) {
        g_block_peak = magnitude;
      }
    }
    g_block_fill += taking;
    consumed += taking;
    if (g_block_fill == kBlockSamples) {
      publish_filled_block();
    }
  }
}

/**
 * @brief Take the next microphone chunk from the NDP120 when one is due.
 *
 * Polls just before the next chunk is expected and retries quickly until the
 * annotation counter changes, so each chunk is taken exactly once. A gap in
 * that counter is counted as dropped audio, which is what lets the desktop tool
 * show whether the stream stayed continuous.
 *
 * @return True when a fresh chunk was consumed.
 */
bool capture_fresh_chunk() {
  if (static_cast<int32_t>(millis() - g_next_chunk_poll_ms) < 0) {
    return false;
  }

  unsigned int length = 0u;
  const uint32_t started_ms = millis();
  const int status = NDP.extractData(g_chunk, &length);
  if (status != 0 || length == 0u ||
      length + kAnnotationBytes > sizeof(g_chunk)) {
    g_next_chunk_poll_ms = millis() + kChunkRetryMs;
    if (++g_chunk_failures >= kChunkFailureLimit) {
      g_chunk_failures = 0u;
      send_error_packet(static_cast<uint8_t>(status));
    }
    return false;
  }
  g_chunk_failures = 0u;

  const uint8_t counter = g_chunk[length + kAnnotationCounterOffset];
  if (g_chunk_sync != ChunkSync::NoCounter && counter == g_last_chunk_counter) {
    g_next_chunk_poll_ms = millis() + kChunkRetryMs;
    return false;
  }
  const uint8_t advance = static_cast<uint8_t>(counter - g_last_chunk_counter);
  g_last_chunk_counter = counter;
  g_next_chunk_poll_ms = millis() + kChunkPeriodMs - kChunkPollLeadMs;

  // Take audio only once two consecutive chunks confirm the stream is being
  // kept up with, so a stream start costs at most two discarded chunks instead
  // of reporting the tank's backlog as dropped audio.
  if (g_chunk_sync != ChunkSync::Running) {
    g_chunk_sync = g_chunk_sync == ChunkSync::Seeded && advance == 1u
                       ? ChunkSync::Running
                       : ChunkSync::Seeded;
    return false;
  }
  g_dropped_chunks = static_cast<uint16_t>(g_dropped_chunks + (advance - 1u));
  g_block_capture_ms += millis() - started_ms;

  int16_t* samples = reinterpret_cast<int16_t*>(g_chunk);
  const size_t count = length / sizeof(int16_t);
  remove_dc_offset(samples, count);
  accumulate_samples(samples, count);
  return true;
}

/**
 * @brief Drop any part-built block and start a stream from a clean state.
 */
void reset_capture_state() {
  g_dc_block = DcBlockState();
  g_block_fill = 0u;
  g_block_energy = 0u;
  g_block_peak = 0u;
  g_block_capture_ms = 0u;
  g_chunk_sync = ChunkSync::NoCounter;
  g_chunk_failures = 0u;
  g_dropped_chunks = 0u;
  g_sequence = 0u;
  g_next_chunk_poll_ms = millis();
}

/**
 * @brief Load the NDP120 firmware and start the microphone.
 *
 * @return True when the microphone is streaming audio chunks.
 */
bool prepare_microphone() {
  if (NDP.begin(kNdpMcuFirmware) != 1 || NDP.load(kNdpDspFirmware) != 1 ||
      NDP.load(kNdpAudioFlowPackage) != 1) {
    return false;
  }
  if (NDP.turnOnMicrophone() != 0) {
    return false;
  }
  return NDP.getAudioChunkSize() > 0;
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  // nicla::disableLDO() must not be called here, unlike in the NDP library's
  // own audio examples. That LDO is the PMIC load switch feeding VDDIO_EXT,
  // which supplies the TXB0108 level translators on every header pin (Nicla
  // Voice datasheet section 4.9), so disabling it cuts off an attached BB15.
  nicla::begin();
  nicla::leds.begin();
  set_led(blue);
  // Serial here is a UART bridged to USB by the on-board SAMD11, not a native
  // USB device, so it is ready immediately and only the host needs a moment.
  delay(kBootSettleMs);

  Serial.println();
  Serial.println(kSketchName);
  Serial.print(kLogPrefix);
  Serial.println(" board=BB15 + Nicla Voice");
  Serial.print(kLogPrefix);
  Serial.print(" audio=");
  Serial.print(kSampleRateHz);
  Serial.print("Hz mono block=");
  Serial.print(kBlockSamples);
  Serial.println(" samples");

  if (!prepare_microphone()) {
    halt_forever("microphone");
  }

  set_led(off);
  Serial.print(kLogPrefix);
  Serial.print(" microphone_ready chunk_bytes=");
  Serial.println(NDP.getAudioChunkSize());
  Serial.print(kLogPrefix);
  Serial.println(" usb_protocol=BB15/v1 waiting_for_start_stream");
}

void loop() {
  const PacketType command = poll_host_command();
  if (command == PacketType::StartStream) {
    reset_capture_state();
    g_streaming = true;
    set_led(green);
    send_audio_config_packet();
    return;
  }
  if (command == PacketType::StopStream) {
    g_streaming = false;
    set_led(off);
    return;
  }
  if (command == PacketType::RequestConfig) {
    send_audio_config_packet();
    return;
  }
  if (!g_streaming) {
    delay(1);
    return;
  }

  if (!capture_fresh_chunk()) {
    delay(1);
  }
}
