#include "ble_model_transfer.h"

#include <ArduinoBLE.h>
#include <akida/version.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <memory>

namespace model {
namespace {

// One slot per application in the BrainBoard's 8 MB external model window.
// Only the keyword slot is used today; a second application takes the next
// slot without disturbing this one.
constexpr uint32_t kModelSlotBytes = 0x80000u;
constexpr uint32_t kKeywordSlotOffset = 0u;

// The slot opens with one flash sector describing what follows, so the record
// sits at a fixed address and the model data stays sector aligned behind it.
constexpr uint32_t kRecordBytes = 4096u;
constexpr uint8_t kRecordVersion = 1u;

// A FlatBuffers size prefix is four bytes, and the smallest serialized
// program is two prefixed buffers, so anything shorter is not one.
constexpr size_t kSizePrefixBytes = 4u;
constexpr size_t kMinProgramBytes = 2u * kSizePrefixBytes;

// Fields the phone folds into its combined CRC, in its order: total length,
// three input dimensions, three output dimensions, flash address, edge flag,
// packed edge classes, info length, MFCC scale, silence class, unknown class
// and inference mode, then the model name padded out.
constexpr size_t kCrcHeaderFields = 15u;
constexpr size_t kCrcHeaderBytes = kCrcHeaderFields * 4u + kMaxNameLength;

constexpr uint8_t kTransferInfo = 0x00u;

constexpr uint8_t kAckEraseDone = 0xEEu;
constexpr uint8_t kAckWriteDone = 0xCCu;
constexpr uint8_t kAckCrcFail = 0xBBu;

constexpr size_t kMaxChunkBytes = 244u;
constexpr size_t kMaxShapeDimensions = 3u;

constexpr const char* kLogPrefix = "[bb15_nicla_vision_connect]";

// The low byte of each characteristic UUID, which is how a write is routed.
constexpr uint8_t kCodeFileTransfer = 0x01u;
constexpr uint8_t kCodeFileSize = 0x04u;
constexpr uint8_t kCodeFileCrc = 0x06u;
constexpr uint8_t kCodeTransferType = 0x07u;
constexpr uint8_t kCodeInputShape = 0x08u;
constexpr uint8_t kCodeOutputShape = 0x09u;
constexpr uint8_t kCodeFlashAddress = 0x0Au;
constexpr uint8_t kCodeTotalLength = 0x0Bu;
constexpr uint8_t kCodeIsEdgeLearned = 0x0Cu;
constexpr uint8_t kCodeNumEdgeClasses = 0x0Du;
constexpr uint8_t kCodeFsName = 0x0Eu;
constexpr uint8_t kCodeMfccFs = 0x0Fu;
constexpr uint8_t kCodeSilenceClass = 0x10u;
constexpr uint8_t kCodeUnknownClass = 0x11u;
constexpr uint8_t kCodeInferenceMode = 0x12u;

BLEService g_service("f000aa00-0451-4000-b000-000000000000");
BLECharacteristic g_file_transfer("f000aa01-0451-4000-b000-000000000000",
                                  BLEWrite | BLEWriteWithoutResponse,
                                  kMaxChunkBytes);
BLECharacteristic g_ack("f000aa02-0451-4000-b000-000000000000", BLENotify, 1);
BLECharacteristic g_file_size("f000aa04-0451-4000-b000-000000000000", BLEWrite,
                              4);
BLECharacteristic g_app_index("f000aa05-0451-4000-b000-000000000000", BLEWrite,
                              1);
BLECharacteristic g_file_crc("f000aa06-0451-4000-b000-000000000000", BLEWrite,
                             4);
BLECharacteristic g_transfer_type("f000aa07-0451-4000-b000-000000000000",
                                  BLEWrite, 1);
BLECharacteristic g_input_shape("f000aa08-0451-4000-b000-000000000000",
                                BLEWrite, 12);
BLECharacteristic g_output_shape("f000aa09-0451-4000-b000-000000000000",
                                 BLEWrite, 12);
BLECharacteristic g_flash_address("f000aa0a-0451-4000-b000-000000000000",
                                  BLEWrite, 4);
BLECharacteristic g_total_length("f000aa0b-0451-4000-b000-000000000000",
                                 BLEWrite, 4);
BLECharacteristic g_is_edge_learned("f000aa0c-0451-4000-b000-000000000000",
                                    BLEWrite, 4);
BLECharacteristic g_num_edge_classes("f000aa0d-0451-4000-b000-000000000000",
                                     BLEWrite, 4);
BLECharacteristic g_fs_name("f000aa0e-0451-4000-b000-000000000000", BLEWrite,
                            kMaxNameLength);
BLECharacteristic g_mfcc_fs("f000aa0f-0451-4000-b000-000000000000", BLEWrite,
                            4);
BLECharacteristic g_silence_class("f000aa10-0451-4000-b000-000000000000",
                                  BLEWrite, 4);
BLECharacteristic g_unknown_class("f000aa11-0451-4000-b000-000000000000",
                                  BLEWrite, 4);
BLECharacteristic g_inference_mode("f000aa12-0451-4000-b000-000000000000",
                                   BLEWrite, 4);

/** @brief Metadata written beside the model so a later boot can load it. */
struct __attribute__((packed)) Record {
  uint8_t magic[4];
  uint32_t version;
  uint32_t recordCrc32;
  uint32_t infoLength;
  uint32_t dataLength;
  uint32_t dataCrc32;
  uint32_t inputShape[kMaxShapeDimensions];
  uint32_t outputShape[kMaxShapeDimensions];
  uint32_t isEdgeLearned;
  uint32_t numEdgeClasses;
  uint32_t mfccFsBits;
  uint32_t silenceClass;
  uint32_t unknownClass;
  uint32_t inferenceMode;
  char name[kMaxNameLength];
};

/** @brief Room left in the record sector for the program info half. */
constexpr size_t kMaxInfoBytes = kRecordBytes - sizeof(Record);

/** @brief What the phone has told us about the transfer in flight. */
struct Incoming {
  uint32_t infoLength = 0u;
  uint32_t dataLength = 0u;
  uint32_t totalLength = 0u;
  uint32_t combinedCrc32 = 0u;
  uint32_t dataCrc32 = 0u;
  uint32_t flashAddress = 0u;
  uint32_t inputShape[kMaxShapeDimensions] = {0u, 0u, 0u};
  uint32_t outputShape[kMaxShapeDimensions] = {0u, 0u, 0u};
  uint32_t isEdgeLearned = 0u;
  uint32_t numEdgeClasses = 0u;
  uint32_t mfccFsBits = 0u;
  uint32_t silenceClass = 0u;
  uint32_t unknownClass = 0u;
  uint32_t inferenceMode = 0u;
  char name[kMaxNameLength] = {0};
  uint8_t phase = kTransferInfo;
  uint32_t received = 0u;
  bool active = false;
};

BB15* g_board = nullptr;
BB15Runner* g_runner = nullptr;
LoadedHandler g_on_loaded = nullptr;
ActivityHandler g_on_activity = nullptr;

Incoming g_incoming;
// Holds [info][record][data] while a transfer runs, then is released.
std::unique_ptr<uint8_t[]> g_staging;
// Holds the whole serialized program for as long as the model stays loaded,
// because the runtime keeps the pointer it was loaded from.
std::unique_ptr<uint8_t[]> g_program;
Loaded g_loaded;

uint8_t g_pending_ack = 0u;
bool g_install_pending = false;

/**
 * @brief The CRC32 the phone sends, which is not the usual one.
 *
 * The phone seeds its crc-32 library with 0xFFFFFFFF and inverts the result,
 * and that library inverts the seed on the way in and the register on the way
 * out, so both cancel: what arrives is the raw CRC register run from zero with
 * no final inversion. Seeding this with 0xFFFFFFFF and inverting the result,
 * which is what CRC32 normally means, produces a different value and the phone
 * rejects the transfer.
 *
 * @param crc    Running register, zero for the first call.
 * @param bytes  Bytes to fold in.
 * @param count  Number of bytes.
 * @return The updated register, which is the CRC itself once the last call
 *         returns.
 */
uint32_t crc32Update(uint32_t crc, const uint8_t* bytes, size_t count) {
  static const uint32_t kNibbleTable[16] = {
      0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
      0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
      0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
      0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu};
  for (size_t index = 0u; index < count; ++index) {
    crc ^= bytes[index];
    crc = (crc >> 4) ^ kNibbleTable[crc & 0x0Fu];
    crc = (crc >> 4) ^ kNibbleTable[crc & 0x0Fu];
  }
  return crc;
}

/**
 * @brief Identify a characteristic of this service by its UUID.
 *
 * Every UUID in the service is f000aaXX-0451-4000-b000-000000000000, so the
 * two hex digits at offset 6 name it. ArduinoBLE gives a shared write handler
 * no other way to tell one characteristic from another.
 *
 * @param characteristic  Characteristic the phone has just written.
 * @return The XX byte, or zero when the UUID is too short to carry one.
 */
uint8_t characteristicCode(const BLECharacteristic& characteristic) {
  const char* uuid = characteristic.uuid();
  if (uuid == nullptr || strlen(uuid) < 8u) {
    return 0u;
  }
  const char code[3] = {uuid[6], uuid[7], '\0'};
  return static_cast<uint8_t>(strtoul(code, nullptr, 16));
}

/**
 * @brief Read a little-endian unsigned 32-bit value from a characteristic.
 *
 * @param characteristic  Characteristic the phone has just written.
 * @return The value, or zero when the write was too short.
 */
uint32_t readU32(const BLECharacteristic& characteristic) {
  if (characteristic.valueLength() < 4) {
    return 0u;
  }
  const uint8_t* bytes = characteristic.value();
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) |
         (static_cast<uint32_t>(bytes[3]) << 24);
}

/**
 * @brief Read up to three little-endian dimensions from a characteristic.
 *
 * @param characteristic  Characteristic the phone has just written.
 * @param out             Receives the dimensions, zero padded.
 */
void readShape(const BLECharacteristic& characteristic, uint32_t* out) {
  const uint8_t* bytes = characteristic.value();
  const size_t count = static_cast<size_t>(characteristic.valueLength()) / 4u;
  for (size_t index = 0u; index < kMaxShapeDimensions; ++index) {
    if (index < count) {
      out[index] = static_cast<uint32_t>(bytes[index * 4u]) |
                   (static_cast<uint32_t>(bytes[index * 4u + 1u]) << 8) |
                   (static_cast<uint32_t>(bytes[index * 4u + 2u]) << 16) |
                   (static_cast<uint32_t>(bytes[index * 4u + 3u]) << 24);
    } else {
      out[index] = 0u;
    }
  }
}

/**
 * @brief Number of output values a shape describes.
 *
 * @param shape  Up to three dimensions, zero padded.
 * @return The product of the non-zero dimensions.
 */
uint32_t shapeVolume(const uint32_t* shape) {
  uint32_t volume = 1u;
  for (size_t index = 0u; index < kMaxShapeDimensions; ++index) {
    if (shape[index] != 0u) {
      volume *= shape[index];
    }
  }
  return volume;
}

/** @brief Address of the record that opens the keyword slot. */
uint32_t recordAddress() {
  return AkidaNicla::externalModelAddressFromOffset(kKeywordSlotOffset);
}

/** @brief Address the model data is written to and loaded from. */
uint32_t modelDataAddress() {
  return AkidaNicla::externalModelAddressFromOffset(kKeywordSlotOffset +
                                                    kRecordBytes);
}

/** @brief Start of the program info inside the staging buffer. */
uint8_t* stagingInfo() { return g_staging.get(); }

/** @brief Start of the record inside the staging buffer. */
uint8_t* stagingRecord() { return g_staging.get() + g_incoming.infoLength; }

/** @brief Start of the model data inside the staging buffer. */
uint8_t* stagingData() {
  return g_staging.get() + g_incoming.infoLength + kRecordBytes;
}

/**
 * @brief Recompute the CRC the phone sent with the info file.
 *
 * The phone runs it over a header built from the fields it wrote to the
 * metadata characteristics, followed by the info bytes, so the same header has
 * to be rebuilt here to check it. The model name is the last segment of the
 * filesystem path the phone wrote, which is how it derives the name too.
 *
 * @return The CRC of the header and the received info bytes.
 */
uint32_t computeCombinedCrc32() {
  uint8_t header[kCrcHeaderBytes];
  memset(header, 0, sizeof(header));

  const uint32_t fields[kCrcHeaderFields] = {
      g_incoming.totalLength,    g_incoming.inputShape[0],
      g_incoming.inputShape[1],  g_incoming.inputShape[2],
      g_incoming.outputShape[0], g_incoming.outputShape[1],
      g_incoming.outputShape[2], g_incoming.flashAddress,
      g_incoming.isEdgeLearned,  g_incoming.numEdgeClasses,
      g_incoming.infoLength,     g_incoming.mfccFsBits,
      g_incoming.silenceClass,   g_incoming.unknownClass,
      g_incoming.inferenceMode};
  for (size_t index = 0u; index < kCrcHeaderFields; ++index) {
    header[index * 4u] = static_cast<uint8_t>(fields[index] & 0xFFu);
    header[index * 4u + 1u] =
        static_cast<uint8_t>((fields[index] >> 8) & 0xFFu);
    header[index * 4u + 2u] =
        static_cast<uint8_t>((fields[index] >> 16) & 0xFFu);
    header[index * 4u + 3u] =
        static_cast<uint8_t>((fields[index] >> 24) & 0xFFu);
  }
  memcpy(&header[kCrcHeaderFields * 4u], g_incoming.name,
         strnlen(g_incoming.name, kMaxNameLength));

  const uint32_t crc = crc32Update(0u, header, sizeof(header));
  return crc32Update(crc, stagingInfo(), g_incoming.infoLength);
}

/** @brief Abandon the transfer in flight and release its buffer. */
void abandonTransfer() {
  g_staging.reset();
  g_incoming = Incoming();
  if (g_on_activity != nullptr) {
    g_on_activity(false);
  }
}

/**
 * @brief Fill in the record that opens the slot.
 *
 * @param record  Receives the description of the model being installed.
 */
void fillRecord(Record* record) {
  memset(record, 0, sizeof(Record));
  memcpy(record->magic, "BB15", 4);
  record->version = kRecordVersion;
  record->infoLength = g_incoming.infoLength;
  record->dataLength = g_incoming.dataLength;
  record->dataCrc32 = g_incoming.dataCrc32;
  memcpy(record->inputShape, g_incoming.inputShape, sizeof(record->inputShape));
  memcpy(record->outputShape, g_incoming.outputShape,
         sizeof(record->outputShape));
  record->isEdgeLearned = g_incoming.isEdgeLearned;
  record->numEdgeClasses = g_incoming.numEdgeClasses;
  record->mfccFsBits = g_incoming.mfccFsBits;
  record->silenceClass = g_incoming.silenceClass;
  record->unknownClass = g_incoming.unknownClass;
  record->inferenceMode = g_incoming.inferenceMode;
  memcpy(record->name, g_incoming.name, kMaxNameLength);

  const uint8_t* covered =
      reinterpret_cast<const uint8_t*>(&record->infoLength);
  const size_t coveredBytes = sizeof(Record) - offsetof(Record, infoLength);
  record->recordCrc32 = crc32Update(0u, covered, coveredBytes);
}

/**
 * @brief Turn a record and its info blob into the loaded-model description.
 *
 * @param record       Record read from flash or just built.
 * @param program  The serialized program, which must outlive the loaded model.
 * @return The description of the model.
 */
Loaded describe(const Record& record, const uint8_t* program) {
  Loaded described;
  described.program = program;
  described.programBytes = record.infoLength + record.dataLength;
  described.dataBytes = record.dataLength;
  described.classCount = static_cast<uint8_t>(shapeVolume(record.outputShape));
  described.silenceClass = static_cast<uint8_t>(record.silenceClass);
  described.unknownClass = static_cast<uint8_t>(record.unknownClass);
  described.edgeLearning = record.isEdgeLearned != 0u;
  described.edgeClasses =
      static_cast<uint16_t>(record.numEdgeClasses & 0xFFFFu);
  described.neuronsPerClass =
      static_cast<uint16_t>((record.numEdgeClasses >> 16) & 0xFFFFu);
  memcpy(&described.mfccFullScale, &record.mfccFsBits, sizeof(float));
  memcpy(described.name, record.name, kMaxNameLength);
  described.name[kMaxNameLength - 1u] = '\0';
  described.valid = true;
  return described;
}

/**
 * @brief Read a little-endian unsigned 32-bit value out of a buffer.
 *
 * @param bytes  Buffer holding at least four bytes at `offset`.
 * @param offset Where to read from.
 * @return The value.
 */
uint32_t readU32At(const uint8_t* bytes, size_t offset) {
  return static_cast<uint32_t>(bytes[offset]) |
         (static_cast<uint32_t>(bytes[offset + 1u]) << 8) |
         (static_cast<uint32_t>(bytes[offset + 2u]) << 16) |
         (static_cast<uint32_t>(bytes[offset + 3u]) << 24);
}

/**
 * @brief Say whether a buffer is a serialized program this engine can load.
 *
 * Everything here arrives over Bluetooth, and the engine does not reject a
 * malformed or foreign program: it calls panic(), which halts the core and
 * takes USB down with it, leaving the board recoverable only with the reset
 * button. So the same things the engine would panic over are checked here
 * first, where a refusal can be reported instead.
 *
 * A serialized program is a size-prefixed program info followed by a
 * size-prefixed program, and the two lengths must account for the whole
 * buffer. The engine also refuses a program built for another Akida version,
 * so the version string it expects has to appear in the info.
 *
 * @param program  Whole serialized program.
 * @param bytes    Its length.
 * @return True when it is safe to hand to the engine.
 */
bool programAcceptable(const uint8_t* program, size_t bytes) {
  if (program == nullptr || bytes < kMinProgramBytes) {
    return false;
  }
  const size_t infoBytes = readU32At(program, 0u) + kSizePrefixBytes;
  if (infoBytes < kSizePrefixBytes || infoBytes > bytes - kSizePrefixBytes) {
    return false;
  }
  const size_t dataBytes = readU32At(program, infoBytes) + kSizePrefixBytes;
  if (infoBytes + dataBytes != bytes) {
    return false;
  }

  // The version is a null-terminated string inside the info flatbuffer, so it
  // is looked for rather than parsed: a wrong one must not reach the engine.
  const char* expected = akida::version();
  const size_t expectedBytes = strlen(expected) + 1u;
  if (expectedBytes > infoBytes) {
    return false;
  }
  for (size_t start = 0u; start + expectedBytes <= infoBytes; ++start) {
    if (memcmp(program + start, expected, expectedBytes) == 0) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Load a serialized program whose data half is in BrainBoard flash.
 *
 * @param program  Whole serialized program, which the runtime keeps a pointer
 *                 to for as long as the model stays loaded.
 * @param bytes    Its length.
 * @return True when the runner accepted it.
 */
bool loadFromFlash(const uint8_t* program, size_t bytes) {
  BB15Model model(program, bytes);
  model.setStorage(BB15ModelStorage::ExternalFlash)
      .setExternalAddress(modelDataAddress());
  return g_runner->loadModel(model) == BB15Status::Ok;
}

/**
 * @brief Rebuild the installed model out of BrainBoard flash and load it.
 *
 * The slot holds a record describing the model followed by the data half of
 * the serialized program; the info half lives inside the record. The whole
 * program is reassembled here because the engine needs both halves to parse
 * it, and it is validated before the engine is allowed near it.
 *
 * @return True when a model was found, accepted and loaded.
 */
/**
 * @brief Read the installed model out of the slot, through the SPI bridge.
 *
 * The bridge has to be taken over for the read: the memory mapped path the
 * board uses otherwise is not up on a cold boot, and the read simply fails.
 *
 * @param record   Receives the record that opens the slot.
 * @param program  Receives the reassembled serialized program.
 * @return True when a plausible record and its program were read.
 */
bool readInstalledProgram(Record* record, std::unique_ptr<uint8_t[]>* program) {
  if (!g_board->detectFlash() || !g_board->s2mEnter()) {
    return false;
  }

  std::unique_ptr<uint8_t[]> sector(new uint8_t[kRecordBytes]);
  bool ok =
      g_board->readExternalData(recordAddress(), sector.get(), kRecordBytes);
  if (ok) {
    memcpy(record, sector.get(), sizeof(Record));
    ok = memcmp(record->magic, "BB15", 4) == 0 &&
         record->version == kRecordVersion &&
         record->infoLength >= kMinProgramBytes &&
         record->infoLength <= kMaxInfoBytes && record->dataLength > 0u &&
         record->dataLength <= kModelSlotBytes - kRecordBytes;
  }

  if (ok) {
    const uint8_t* covered =
        reinterpret_cast<const uint8_t*>(&record->infoLength);
    const size_t coveredBytes = sizeof(Record) - offsetof(Record, infoLength);
    ok = crc32Update(0u, covered, coveredBytes) == record->recordCrc32;
  }

  if (ok) {
    program->reset(new uint8_t[record->infoLength + record->dataLength]);
    memcpy(program->get(), sector.get() + sizeof(Record), record->infoLength);
    ok = g_board->readExternalData(modelDataAddress(),
                                   program->get() + record->infoLength,
                                   record->dataLength);
  }

  g_board->s2mExit();
  return ok;
}

/**
 * @brief Load whatever model the slot holds.
 *
 * The program is validated before the engine is allowed near it, because the
 * engine answers a malformed or foreign one by halting the core.
 *
 * @return True when a model was found, accepted and loaded.
 */
bool loadInstalledModel() {
  Record record;
  std::unique_ptr<uint8_t[]> program;
  if (!readInstalledProgram(&record, &program)) {
    return false;
  }

  const size_t programBytes = record.infoLength + record.dataLength;
  if (!programAcceptable(program.get(), programBytes)) {
    Serial.print(kLogPrefix);
    Serial.println(" model refused: not a program this engine can load");
    return false;
  }
  if (!loadFromFlash(program.get(), programBytes)) {
    return false;
  }

  g_program = std::move(program);
  g_loaded = describe(record, g_program.get());
  return true;
}

/**
 * @brief Write the staged model to flash and load it.
 *
 * The staging buffer is laid out as program info, record, data so that one
 * write installs the slot: the library writes everything past the program
 * info, which is exactly the record followed by the data.
 *
 * @return True when the model was written and loaded.
 */
bool installStagedModel() {
  Record record;
  fillRecord(&record);
  memcpy(stagingRecord(), &record, sizeof(Record));
  memset(stagingRecord() + sizeof(Record), 0xFF, kRecordBytes - sizeof(Record));
  memcpy(stagingRecord() + sizeof(Record), stagingInfo(),
         g_incoming.infoLength);

  const size_t staged =
      g_incoming.infoLength + kRecordBytes + g_incoming.dataLength;
  const bool written =
      g_board->programExternalData(g_staging.get(), staged, recordAddress());
  // Released before the model is rebuilt, so the staging copy and the retained
  // program are never both held. The vision model makes that difference large.
  g_staging.reset();
  return written && loadInstalledModel();
}

/** @brief Check what the phone sent for the phase that has just finished. */
void completePhase() {
  const bool info = g_incoming.phase == kTransferInfo;
  const uint32_t expected =
      info ? g_incoming.combinedCrc32 : g_incoming.dataCrc32;
  const uint32_t actual =
      info ? computeCombinedCrc32()
           : crc32Update(0u, stagingData(), g_incoming.dataLength);

  Serial.print(kLogPrefix);
  Serial.print(" transfer phase=");
  Serial.print(info ? "info" : "data");
  Serial.print(" received=");
  Serial.print(static_cast<unsigned long>(g_incoming.received));
  Serial.print(" crc=0x");
  Serial.print(actual, HEX);
  Serial.print(" expected=0x");
  Serial.println(expected, HEX);

  if (actual != expected) {
    g_pending_ack = kAckCrcFail;
    return;
  }
  g_pending_ack = kAckWriteDone;
  g_install_pending = !info;
}

/**
 * @brief Take one chunk of file bytes into the staging buffer.
 *
 * @param bytes  Chunk the phone wrote.
 * @param count  Number of bytes in it.
 */
void receiveChunk(const uint8_t* bytes, size_t count) {
  if (g_staging == nullptr) {
    return;
  }
  const uint32_t expected = g_incoming.phase == kTransferInfo
                                ? g_incoming.infoLength
                                : g_incoming.dataLength;
  uint8_t* destination =
      g_incoming.phase == kTransferInfo ? stagingInfo() : stagingData();
  if (g_incoming.received >= expected) {
    return;
  }
  if (g_incoming.received + count > expected) {
    count = expected - g_incoming.received;
  }
  memcpy(destination + g_incoming.received, bytes, count);
  g_incoming.received += count;

  if (g_incoming.received >= expected) {
    completePhase();
  }
}

/**
 * @brief Allocate the staging buffer once the phone announces the total size.
 *
 * @return True when the buffer was allocated.
 */
bool allocateStaging() {
  // Both lengths are the phone's word for it, and everything downstream is
  // sized from them, so they are bounded here rather than trusted. The info
  // half has to fit in the record that is written at the head of the slot.
  if (g_incoming.infoLength < kMinProgramBytes ||
      g_incoming.infoLength > kMaxInfoBytes) {
    return false;
  }
  if (g_incoming.totalLength <= g_incoming.infoLength) {
    return false;
  }
  g_incoming.dataLength = g_incoming.totalLength - g_incoming.infoLength;
  if (g_incoming.dataLength + kRecordBytes > kModelSlotBytes) {
    return false;
  }
  g_staging.reset(new uint8_t[g_incoming.infoLength + kRecordBytes +
                              g_incoming.dataLength]);
  return g_staging != nullptr;
}

/**
 * @brief Take the model name from the filesystem path the phone wrote.
 *
 * @param characteristic  The path characteristic the phone has just written.
 */
void readModelName(const BLECharacteristic& characteristic) {
  char path[kMaxNameLength] = {0};
  size_t length = static_cast<size_t>(characteristic.valueLength());
  if (length > kMaxNameLength - 1u) {
    length = kMaxNameLength - 1u;
  }
  memcpy(path, characteristic.value(), length);

  const char* last = strrchr(path, '/');
  const char* name = last != nullptr ? last + 1 : path;
  memset(g_incoming.name, 0, kMaxNameLength);
  strncpy(g_incoming.name, name, kMaxNameLength - 1u);
}

/** @brief Note the start of a phase and tell the phone the board is ready. */
void beginPhase(const BLECharacteristic& characteristic) {
  const uint32_t size = readU32(characteristic);
  g_incoming.received = 0u;
  if (g_incoming.phase == kTransferInfo) {
    g_incoming.infoLength = size;
    g_incoming.active = true;
    if (g_on_activity != nullptr) {
      g_on_activity(true);
    }
  }
  // Nothing is erased here: the flash write happens once, after the data
  // phase. The ack tells the phone the board is ready for the bytes.
  g_pending_ack = kAckEraseDone;

  Serial.print(kLogPrefix);
  Serial.print(" transfer phase=");
  Serial.print(g_incoming.phase == kTransferInfo ? "info" : "data");
  Serial.print(" size=");
  Serial.println(static_cast<unsigned long>(size));
}

/** @brief Handle every write the phone makes to the transfer service. */
void onCharacteristicWritten(BLEDevice, BLECharacteristic characteristic) {
  switch (characteristicCode(characteristic)) {
    case kCodeTransferType:
      g_incoming.phase = characteristic.value()[0];
      g_incoming.received = 0u;
      break;
    case kCodeFileSize:
      beginPhase(characteristic);
      break;
    case kCodeTotalLength:
      g_incoming.totalLength = readU32(characteristic);
      if (!allocateStaging()) {
        Serial.print(kLogPrefix);
        Serial.println(" transfer staging_failed");
        g_pending_ack = kAckCrcFail;
        abandonTransfer();
      } else {
        Serial.print(kLogPrefix);
        Serial.print(" transfer total=");
        Serial.print(static_cast<unsigned long>(g_incoming.totalLength));
        Serial.print(" data=");
        Serial.println(static_cast<unsigned long>(g_incoming.dataLength));
      }
      break;
    case kCodeFileCrc:
      if (g_incoming.phase == kTransferInfo) {
        g_incoming.combinedCrc32 = readU32(characteristic);
      } else {
        g_incoming.dataCrc32 = readU32(characteristic);
      }
      break;
    case kCodeFileTransfer:
      receiveChunk(characteristic.value(),
                   static_cast<size_t>(characteristic.valueLength()));
      break;
    case kCodeFsName:
      readModelName(characteristic);
      break;
    case kCodeInputShape:
      readShape(characteristic, g_incoming.inputShape);
      break;
    case kCodeOutputShape:
      readShape(characteristic, g_incoming.outputShape);
      break;
    case kCodeFlashAddress:
      g_incoming.flashAddress = readU32(characteristic);
      break;
    case kCodeIsEdgeLearned:
      g_incoming.isEdgeLearned = readU32(characteristic);
      break;
    case kCodeNumEdgeClasses:
      g_incoming.numEdgeClasses = readU32(characteristic);
      break;
    case kCodeMfccFs:
      g_incoming.mfccFsBits = readU32(characteristic);
      break;
    case kCodeSilenceClass:
      g_incoming.silenceClass = readU32(characteristic);
      break;
    case kCodeUnknownClass:
      g_incoming.unknownClass = readU32(characteristic);
      break;
    case kCodeInferenceMode:
      g_incoming.inferenceMode = readU32(characteristic);
      break;
    default:
      break;
  }
}

}  // namespace

void begin(BB15& board, BB15Runner& runner) {
  g_board = &board;
  g_runner = &runner;

  BLECharacteristic* const written[] = {
      &g_file_transfer,   &g_file_size,        &g_app_index,
      &g_file_crc,        &g_transfer_type,    &g_input_shape,
      &g_output_shape,    &g_flash_address,    &g_total_length,
      &g_is_edge_learned, &g_num_edge_classes, &g_fs_name,
      &g_mfcc_fs,         &g_silence_class,    &g_unknown_class,
      &g_inference_mode};

  g_service.addCharacteristic(g_ack);
  for (BLECharacteristic* characteristic : written) {
    g_service.addCharacteristic(*characteristic);
    characteristic->setEventHandler(BLEWritten, onCharacteristicWritten);
  }
  BLE.addService(g_service);
}

void setHandlers(LoadedHandler onLoaded, ActivityHandler onActivity) {
  g_on_loaded = onLoaded;
  g_on_activity = onActivity;
}

bool restoreFromFlash() {
  if (!loadInstalledModel()) {
    return false;
  }
  if (g_on_loaded != nullptr) {
    g_on_loaded(g_loaded);
  }
  return true;
}

void poll() {
  if (g_pending_ack != 0u) {
    const uint8_t ack = g_pending_ack;
    g_pending_ack = 0u;
    g_ack.writeValue(&ack, 1);
    if (ack == kAckCrcFail) {
      abandonTransfer();
    }
  }

  if (!g_install_pending) {
    return;
  }
  g_install_pending = false;

  const uint32_t started = millis();
  const bool installed = installStagedModel();
  Serial.print(kLogPrefix);
  Serial.print(" install ok=");
  Serial.print(installed ? 1 : 0);
  Serial.print(" ms=");
  Serial.println(static_cast<unsigned long>(millis() - started));
  if (!installed) {
    Serial.print(kLogPrefix);
    Serial.print(" install detail=");
    g_board->printLastError(Serial);
  }
  abandonTransfer();
  if (installed && g_on_loaded != nullptr) {
    g_on_loaded(g_loaded);
  }
}

bool transferInProgress() { return g_incoming.active; }

const Loaded& loaded() { return g_loaded; }

}  // namespace model
