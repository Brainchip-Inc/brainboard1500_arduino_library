#include "ble_model_transfer.h"

#include <ArduinoBLE.h>
#include <akida/version.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <memory>
#include <utility>

namespace model {
namespace {

// One slot per application in the BrainBoard's external model window, so
// installing one application's model leaves the other's untouched.
constexpr uint32_t kModelSlotBytes = 0x80000u;

// The slot opens with one flash sector describing what follows, so the record
// sits at a fixed address and the model data stays sector aligned behind it.
constexpr uint32_t kRecordBytes = kBB15ExternalFlashSectorBytes;
constexpr uint8_t kRecordVersion = 2u;

// How much of a model the board takes before it commits it, which is what it
// reports to the phone. The flash erases a sector at a time, so a sector is
// both the least the board can commit and all of a model it has to hold.
constexpr uint32_t kBlockBytes = kBB15ExternalFlashSectorBytes;

// A FlatBuffers buffer opens with a four byte size prefix, and a program info
// blob is one such buffer and nothing else.
constexpr size_t kSizePrefixBytes = 4u;

// Fields the phone folds into its combined CRC, in its order: total length,
// three input dimensions, three output dimensions, flash address, edge flag,
// packed edge classes, info length, MFCC scale, silence class, unknown class
// and inference mode, then the model name padded out.
constexpr size_t kCrcHeaderFields = 15u;
constexpr size_t kCrcHeaderBytes = kCrcHeaderFields * 4u + kMaxNameLength;

constexpr uint8_t kControlStart = 0x01u;
constexpr uint8_t kControlAbort = 0x02u;
constexpr size_t kStartFrameBytes = 6u;

constexpr uint8_t kTransferInfo = 0x00u;
constexpr uint8_t kTransferData = 0x01u;

constexpr uint8_t kResultOk = 0x00u;
constexpr uint8_t kResultDone = 0x01u;
constexpr uint8_t kResultErrOffset = 0x02u;
constexpr uint8_t kResultErrIntegrity = 0x03u;
constexpr uint8_t kResultErrFlash = 0x04u;
constexpr uint8_t kResultErrState = 0x05u;
constexpr uint8_t kResultErrParam = 0x06u;
constexpr uint8_t kResultAborted = 0x07u;
constexpr uint8_t kResultReady = 0x08u;
constexpr uint8_t kResultErrProgram = 0x09u;

// Not a result code, and never sent: it marks that no status is waiting.
constexpr uint8_t kResultNone = 0xFFu;

// A status notification is always this long, and every write of model bytes
// opens with the absolute offset in the file of the bytes behind it.
constexpr size_t kStatusBytes = 14u;
constexpr size_t kOffsetBytes = 4u;

constexpr size_t kMaxDataWriteBytes = 244u;
constexpr size_t kMaxShapeDimensions = 3u;

// The input shape the phone sends before the bytes is what says which
// application a model belongs to, because it is the property that decides
// which pipeline can feed it.
constexpr uint32_t kKeywordInputShape[kMaxShapeDimensions] = {49u, 10u, 1u};
constexpr uint32_t kVisionInputShape[kMaxShapeDimensions] = {96u, 96u, 3u};

constexpr const char* kLogPrefix = "[bb15_nicla_vision_connect]";

// The low byte of each characteristic UUID, which is how a write is routed.
constexpr uint8_t kCodeData = 0x01u;
constexpr uint8_t kCodeControl = 0x03u;
constexpr uint8_t kCodeFileCrc = 0x06u;
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
BLECharacteristic g_data("f000aa01-0451-4000-b000-000000000000",
                         BLEWrite | BLEWriteWithoutResponse,
                         kMaxDataWriteBytes);
BLECharacteristic g_status("f000aa02-0451-4000-b000-000000000000", BLENotify,
                           kStatusBytes);
BLECharacteristic g_control("f000aa03-0451-4000-b000-000000000000", BLEWrite,
                            kStartFrameBytes);
BLECharacteristic g_app_index("f000aa05-0451-4000-b000-000000000000", BLEWrite,
                              1);
BLECharacteristic g_file_crc("f000aa06-0451-4000-b000-000000000000", BLEWrite,
                             4);
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
  uint32_t dataAddress;
  uint8_t dataFirstBytes[4];
  uint32_t outputShape[kMaxShapeDimensions];
  uint32_t mfccFsBits;
  uint32_t silenceClass;
  uint32_t unknownClass;
  char name[kMaxNameLength];
};

/** @brief Room left in the record sector for the program info half. */
constexpr size_t kMaxInfoBytes = kRecordBytes - sizeof(Record);

/** @brief What the phone has told us about the model it is sending. */
struct Session {
  App app = App::Keyword;
  uint32_t totalLength = 0u;
  uint32_t infoLength = 0u;
  uint32_t dataLength = 0u;
  uint32_t infoCrc32 = 0u;
  uint32_t dataCrc32 = 0u;
  uint32_t flashOffset = 0u;
  uint32_t inputShape[kMaxShapeDimensions] = {0u, 0u, 0u};
  uint32_t outputShape[kMaxShapeDimensions] = {0u, 0u, 0u};
  uint32_t isEdgeLearned = 0u;
  uint32_t numEdgeClasses = 0u;
  uint32_t mfccFsBits = 0u;
  uint32_t silenceClass = 0u;
  uint32_t unknownClass = 0u;
  uint32_t inferenceMode = 0u;
  char name[kMaxNameLength] = {0};
  bool flashOffsetSet = false;
  bool active = false;
};

/** @brief Where one half of the model has got to. */
struct Transfer {
  bool active = false;
  uint8_t type = kTransferInfo;
  uint32_t total = 0u;
  uint32_t position = 0u;
  uint32_t staged = 0u;
  uint32_t crc32 = 0u;
};

/** @brief Work a Bluetooth write has queued for poll() to carry out. */
enum class Work {
  None,
  ArmDataTransfer,
  CommitBlock,
  InstallModel,
};

BB15* g_board = nullptr;
BB15Runner* g_runner = nullptr;
LoadedHandler g_on_loaded = nullptr;
ActivityHandler g_on_activity = nullptr;

Session g_session;
Transfer g_transfer;
Record g_record = {};
Installed g_installed[kAppCount];

// One block of the model in flight, held only until it reaches flash.
std::unique_ptr<uint8_t[]> g_block;
// The program info the phone has sent but whose model is not installed yet.
std::unique_ptr<uint8_t[]> g_incoming_info;
// The program info of the model the engine is running. The engine keeps a
// pointer to it, so it outlives every later transfer until one replaces it.
std::unique_ptr<uint8_t[]> g_loaded_info;
Loaded g_loaded;

// The CRC the phone last wrote, which belongs to whichever half starts next.
uint32_t g_pending_crc32 = 0u;

Work g_work = Work::None;
uint8_t g_pending_result = kResultNone;
uint32_t g_pending_position = 0u;

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
 * @brief Holds the BrainBoard's flash bridge for as long as it is in scope.
 *
 * Writing the model flash goes through the AKD1500's SPI feedthrough, which
 * has to be taken over and given back, and the engine cannot run until it is
 * back. Pairing the two here keeps a failure part way through a block from
 * leaving the bridge held.
 */
class BridgeHold {
 public:
  explicit BridgeHold(BB15& board) : board_(board), held_(board.s2mEnter()) {}
  ~BridgeHold() {
    if (held_) {
      board_.s2mExit();
    }
  }

  BridgeHold(const BridgeHold&) = delete;
  BridgeHold& operator=(const BridgeHold&) = delete;

  bool held() const { return held_; }

 private:
  BB15& board_;
  bool held_;
};

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
 * @brief Read a little-endian unsigned 32-bit value out of a buffer.
 *
 * @param bytes   Buffer holding at least four bytes at `offset`.
 * @param offset  Where to read from.
 * @return The value.
 */
uint32_t readU32At(const uint8_t* bytes, size_t offset) {
  return static_cast<uint32_t>(bytes[offset]) |
         (static_cast<uint32_t>(bytes[offset + 1u]) << 8) |
         (static_cast<uint32_t>(bytes[offset + 2u]) << 16) |
         (static_cast<uint32_t>(bytes[offset + 3u]) << 24);
}

/**
 * @brief Write a little-endian unsigned 32-bit value into a buffer.
 *
 * @param bytes   Buffer with room for four bytes at `offset`.
 * @param offset  Where to write.
 * @param value   Value to write.
 */
void writeU32At(uint8_t* bytes, size_t offset, uint32_t value) {
  bytes[offset] = static_cast<uint8_t>(value & 0xFFu);
  bytes[offset + 1u] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  bytes[offset + 2u] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  bytes[offset + 3u] = static_cast<uint8_t>((value >> 24) & 0xFFu);
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
  return readU32At(characteristic.value(), 0u);
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
    out[index] = index < count ? readU32At(bytes, index * 4u) : 0u;
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

/** @brief Offset of one application's slot in the model window. */
uint32_t slotOffset(App app) {
  return static_cast<uint32_t>(app) * kModelSlotBytes;
}

/**
 * @brief Address of the record that opens one application's slot.
 *
 * @param app  Application whose slot is wanted.
 * @return The address the record is written to and read from.
 */
uint32_t recordAddress(App app) {
  return AkidaNicla::externalModelAddressFromOffset(slotOffset(app));
}

/** @brief Address the model data of the transfer in flight is written to. */
uint32_t transferDataAddress() {
  return AkidaNicla::externalModelAddressFromOffset(slotOffset(g_session.app) +
                                                    g_session.flashOffset);
}

/**
 * @brief Decide which application an incoming model belongs to.
 *
 * The phone writes the model's input shape before it sends any bytes, and
 * that is what decides: nothing else it sends names the application, and the
 * shape is the property that settles which pipeline can feed the model. A
 * shape neither pipeline produces is refused rather than guessed at, because
 * a model the engine cannot use halts the core.
 *
 * @param shape  Three input dimensions as the phone sent them.
 * @param app    Receives the application, untouched when there is no match.
 * @return True when the shape names an application this firmware runs.
 */
bool appForInputShape(const uint32_t* shape, App* app) {
  if (memcmp(shape, kKeywordInputShape, sizeof(kKeywordInputShape)) == 0) {
    *app = App::Keyword;
    return true;
  }
  if (memcmp(shape, kVisionInputShape, sizeof(kVisionInputShape)) == 0) {
    *app = App::Vision;
    return true;
  }
  return false;
}

/**
 * @brief Name of one application, for the log.
 *
 * @param app  Application to name.
 * @return A short lower-case name.
 */
const char* appName(App app) {
  return app == App::Keyword ? "keyword" : "vision";
}

/** @brief Take the model name from the filesystem path the phone wrote. */
void readModelName(const BLECharacteristic& characteristic) {
  char path[kMaxNameLength] = {0};
  size_t length = static_cast<size_t>(characteristic.valueLength());
  if (length > kMaxNameLength - 1u) {
    length = kMaxNameLength - 1u;
  }
  memcpy(path, characteristic.value(), length);

  const char* last = strrchr(path, '/');
  const char* name = last != nullptr ? last + 1 : path;
  memset(g_session.name, 0, kMaxNameLength);
  strncpy(g_session.name, name, kMaxNameLength - 1u);
}

/**
 * @brief Recompute the CRC the phone sent with the info file.
 *
 * The phone runs it over a header built from the fields it wrote to the
 * metadata characteristics, followed by the info bytes, so the same header has
 * to be rebuilt here to check it.
 *
 * @param info       The received program info bytes.
 * @param infoBytes  How many of them there are.
 * @return The CRC of the header and those bytes.
 */
uint32_t computeCombinedCrc32(const uint8_t* info, size_t infoBytes) {
  uint8_t header[kCrcHeaderBytes];
  memset(header, 0, sizeof(header));

  const uint32_t fields[kCrcHeaderFields] = {
      g_session.totalLength,    g_session.inputShape[0],
      g_session.inputShape[1],  g_session.inputShape[2],
      g_session.outputShape[0], g_session.outputShape[1],
      g_session.outputShape[2], g_session.flashOffset,
      g_session.isEdgeLearned,  g_session.numEdgeClasses,
      g_session.infoLength,     g_session.mfccFsBits,
      g_session.silenceClass,   g_session.unknownClass,
      g_session.inferenceMode};
  for (size_t index = 0u; index < kCrcHeaderFields; ++index) {
    writeU32At(header, index * 4u, fields[index]);
  }
  memcpy(&header[kCrcHeaderFields * 4u], g_session.name,
         strnlen(g_session.name, kMaxNameLength));

  return crc32Update(crc32Update(0u, header, sizeof(header)), info, infoBytes);
}

/**
 * @brief Say whether a buffer is a program info blob this engine can load.
 *
 * Everything here arrives over Bluetooth, and the engine does not reject a
 * malformed or foreign program info: it calls panic(), which halts the core
 * and takes USB down with it, leaving the board recoverable only with the
 * reset button. So the same things the engine would panic over are checked
 * here first, where a refusal can be reported instead.
 *
 * The blob is one size-prefixed FlatBuffers buffer and nothing else, so its
 * prefix has to account for exactly the bytes received. The engine also
 * refuses a program built for another Akida version, so the version string it
 * expects has to appear in the blob.
 *
 * @param programInfo   The program info half on its own.
 * @param programBytes  Its length.
 * @return True when it is safe to hand to the engine.
 */
bool programInfoAcceptable(const uint8_t* programInfo, size_t programBytes) {
  if (programInfo == nullptr || programBytes <= kSizePrefixBytes) {
    return false;
  }
  if (readU32At(programInfo, 0u) + kSizePrefixBytes != programBytes) {
    return false;
  }

  // The version is a null-terminated string inside the info flatbuffer, so it
  // is looked for rather than parsed: a wrong one must not reach the engine.
  const char* expected = akida::version();
  const size_t expectedBytes = strlen(expected) + 1u;
  if (expectedBytes > programBytes) {
    return false;
  }
  for (size_t start = 0u; start + expectedBytes <= programBytes; ++start) {
    if (memcmp(programInfo + start, expected, expectedBytes) == 0) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Turn the record describing the installed model into its description.
 *
 * @param programInfo  The program info the engine was loaded from, which must
 *                     outlive the loaded model.
 * @return The description of the model.
 */
Loaded describe(const uint8_t* programInfo) {
  Loaded described;
  described.programInfo = programInfo;
  described.programInfoBytes = g_record.infoLength;
  described.dataAddress = g_record.dataAddress;
  described.programBytes = g_record.infoLength + g_record.dataLength;
  described.classCount =
      static_cast<uint8_t>(shapeVolume(g_record.outputShape));
  described.silenceClass = static_cast<uint8_t>(g_record.silenceClass);
  described.unknownClass = static_cast<uint8_t>(g_record.unknownClass);
  memcpy(&described.mfccFullScale, &g_record.mfccFsBits, sizeof(float));
  memcpy(described.name, g_record.name, kMaxNameLength);
  described.name[kMaxNameLength - 1u] = '\0';
  described.valid = true;
  return described;
}

/**
 * @brief Summarize a record for the application list.
 *
 * @param record  Record read out of a slot.
 * @return What the phone needs to be told about that slot.
 */
Installed summarize(const Record& record) {
  Installed summary;
  summary.programBytes = record.infoLength + record.dataLength;
  summary.classCount = static_cast<uint8_t>(shapeVolume(record.outputShape));
  summary.silenceClass = static_cast<uint8_t>(record.silenceClass);
  summary.unknownClass = static_cast<uint8_t>(record.unknownClass);
  memcpy(summary.name, record.name, kMaxNameLength);
  summary.name[kMaxNameLength - 1u] = '\0';
  summary.present = true;
  return summary;
}

/** @brief The CRC covering everything in the record past its own CRC field. */
uint32_t recordCrc32(const Record& record) {
  const uint8_t* covered = reinterpret_cast<const uint8_t*>(&record.infoLength);
  return crc32Update(0u, covered,
                     sizeof(Record) - offsetof(Record, infoLength));
}

/**
 * @brief Fill in the record that opens the slot from the session just sent.
 *
 * @param dataFirstBytes  The first four bytes of the model data, as they read
 *                        back out of flash.
 */
void fillRecord(const uint8_t* dataFirstBytes) {
  memset(&g_record, 0, sizeof(Record));
  memcpy(g_record.magic, "BB15", 4);
  g_record.version = kRecordVersion;
  g_record.infoLength = g_session.infoLength;
  g_record.dataLength = g_session.dataLength;
  g_record.dataCrc32 = g_session.dataCrc32;
  g_record.dataAddress = transferDataAddress();
  memcpy(g_record.dataFirstBytes, dataFirstBytes,
         sizeof(g_record.dataFirstBytes));
  memcpy(g_record.outputShape, g_session.outputShape,
         sizeof(g_record.outputShape));
  g_record.mfccFsBits = g_session.mfccFsBits;
  g_record.silenceClass = g_session.silenceClass;
  g_record.unknownClass = g_session.unknownClass;
  memcpy(g_record.name, g_session.name, kMaxNameLength);
  g_record.recordCrc32 = recordCrc32(g_record);
}

/**
 * @brief Say whether a record read out of flash describes a usable model.
 *
 * @param record  Record as it was read.
 * @return True when it is one this build wrote and its fields are in range.
 */
bool recordUsable(const Record& record) {
  return memcmp(record.magic, "BB15", 4) == 0 &&
         record.version == kRecordVersion &&
         record.infoLength > kSizePrefixBytes &&
         record.infoLength <= kMaxInfoBytes && record.dataLength > 0u &&
         record.dataLength <= kModelSlotBytes - kRecordBytes &&
         recordCrc32(record) == record.recordCrc32;
}

/**
 * @brief Send the phone one status notification.
 *
 * @param result    One of the result codes.
 * @param position  The byte the board expects next, or where it gave up.
 */
void notifyStatus(uint8_t result, uint32_t position) {
  uint8_t frame[kStatusBytes];
  frame[0] = result;
  frame[1] = g_transfer.type;
  writeU32At(frame, 2u, kBlockBytes);
  writeU32At(frame, 6u, position);
  writeU32At(frame, 10u, g_transfer.total);
  g_status.writeValue(frame, static_cast<int>(kStatusBytes));
}

/** @brief Queue the status poll() will send next. */
void queueStatus(uint8_t result, uint32_t position) {
  g_pending_result = result;
  g_pending_position = position;
}

/** @brief Give up the half in flight and let go of the block it was using. */
void endTransfer() {
  g_transfer.active = false;
  g_transfer.staged = 0u;
  g_block.reset();
}

/** @brief Give up the whole session, so the next thing accepted is a START. */
void endSession() {
  endTransfer();
  g_incoming_info.reset();
  if (g_session.active && g_on_activity != nullptr) {
    g_on_activity(false);
  }
  g_session.active = false;
}

/**
 * @brief Refuse the transfer, telling the phone why.
 *
 * Every code but OK, DONE and READY ends the transfer, so the phone is told
 * and the board goes back to waiting for a START.
 *
 * @param result  The code to report.
 */
void refuse(uint8_t result) {
  Serial.print(kLogPrefix);
  Serial.print(" transfer refused type=");
  Serial.print(g_transfer.type == kTransferInfo ? "info" : "data");
  Serial.print(" result=0x");
  Serial.print(result, HEX);
  Serial.print(" position=");
  Serial.println(static_cast<unsigned long>(g_transfer.position));

  queueStatus(result, g_transfer.position);
  endSession();
}

/**
 * @brief Forget the model the engine is running, because it is being replaced.
 *
 * The first committed block of a new model overwrites the flash the old one is
 * read from, so from that moment the board has nothing to score with and says
 * so rather than running a model that is no longer there.
 */
void forgetLoadedModel() {
  if (!g_loaded.valid || g_loaded.app != g_session.app) {
    return;
  }

  // Name the slot, and say what is not being touched. A reader watching a
  // transfer needs to be able to tell "this application's model is being
  // replaced" from "the board has lost its models", and only the slot being
  // written is affected.
  Serial.print(kLogPrefix);
  Serial.print(" replacing the ");
  Serial.print(appName(g_session.app));
  Serial.print(" model; the ");
  Serial.print(
      appName(g_session.app == App::Keyword ? App::Vision : App::Keyword));
  Serial.println(" slot is untouched");

  g_loaded = Loaded();
  if (g_on_loaded != nullptr) {
    g_on_loaded(g_loaded);
  }
}

/**
 * @brief Begin the info half, which is small enough to be one block.
 *
 * @param total  Bytes of program info the phone is about to send.
 */
void beginInfoTransfer(uint32_t total) {
  endSession();
  g_transfer = Transfer();
  g_transfer.type = kTransferInfo;
  g_transfer.total = total;

  if (total <= kSizePrefixBytes || total > kMaxInfoBytes ||
      g_session.name[0] == '\0') {
    refuse(kResultErrParam);
    return;
  }

  g_block.reset(new uint8_t[kBlockBytes]);
  if (g_block == nullptr) {
    refuse(kResultErrParam);
    return;
  }

  g_session.infoLength = total;
  g_session.infoCrc32 = g_pending_crc32;
  g_session.active = true;
  g_transfer.active = true;

  if (g_on_activity != nullptr) {
    g_on_activity(true);
  }
  queueStatus(kResultOk, 0u);
}

/**
 * @brief Begin the data half, which inherits the session's metadata.
 *
 * @param total  Bytes of model data the phone is about to send.
 */
void beginDataTransfer(uint32_t total) {
  endTransfer();
  g_transfer = Transfer();
  g_transfer.type = kTransferData;
  g_transfer.total = total;

  if (g_incoming_info == nullptr || !g_session.flashOffsetSet) {
    refuse(kResultErrState);
    return;
  }
  if (total == 0u ||
      (g_session.flashOffset & (kBB15ExternalFlashSectorBytes - 1u)) != 0u ||
      g_session.flashOffset < kRecordBytes ||
      g_session.flashOffset + total > kModelSlotBytes) {
    refuse(kResultErrParam);
    return;
  }

  g_block.reset(new uint8_t[kBlockBytes]);
  if (g_block == nullptr) {
    refuse(kResultErrParam);
    return;
  }

  g_session.dataLength = total;
  g_session.dataCrc32 = g_pending_crc32;
  g_transfer.active = true;
  g_work = Work::ArmDataTransfer;
}

/**
 * @brief Handle a write to the control characteristic.
 *
 * @param frame   The bytes the phone wrote.
 * @param length  How many of them there are.
 */
void onControlWritten(const uint8_t* frame, size_t length) {
  if (length == 1u && frame[0] == kControlAbort) {
    Serial.print(kLogPrefix);
    Serial.println(" transfer aborted by phone");
    queueStatus(kResultAborted, 0u);
    endSession();
    return;
  }
  if (length != kStartFrameBytes || frame[0] != kControlStart) {
    refuse(kResultErrParam);
    return;
  }

  // A transfer the board has no way to report on is not worth beginning.
  if (!g_status.subscribed()) {
    Serial.print(kLogPrefix);
    Serial.println(" transfer refused: status notifications are not enabled");
    endSession();
    return;
  }

  const uint8_t type = frame[1];
  const uint32_t total = readU32At(frame, 2u);
  Serial.print(kLogPrefix);
  Serial.print(" transfer start type=");
  Serial.print(type == kTransferInfo ? "info" : "data");
  Serial.print(" total=");
  Serial.print(static_cast<unsigned long>(total));
  Serial.print(" block=");
  Serial.println(static_cast<unsigned long>(kBlockBytes));

  if (type == kTransferInfo) {
    beginInfoTransfer(total);
  } else if (type == kTransferData) {
    beginDataTransfer(total);
  } else {
    refuse(kResultErrParam);
  }
}

/** @brief Bytes of the block in flight, which is short only at the end. */
uint32_t blockLength() {
  const uint32_t remaining = g_transfer.total - g_transfer.position;
  return remaining < kBlockBytes ? remaining : kBlockBytes;
}

/**
 * @brief Take one write of model bytes into the block being staged.
 *
 * Nothing here answers at ATT level: every outcome is a status notification,
 * so a write with a response and one without behave the same.
 *
 * @param frame   The bytes the phone wrote, opening with the file offset.
 * @param length  How many of them there are.
 */
void onDataWritten(const uint8_t* frame, size_t length) {
  if (!g_transfer.active) {
    refuse(kResultErrState);
    return;
  }
  if (length <= kOffsetBytes) {
    refuse(kResultErrParam);
    return;
  }

  const uint32_t offset = readU32At(frame, 0u);
  const uint32_t payload = static_cast<uint32_t>(length - kOffsetBytes);
  if (offset != g_transfer.position + g_transfer.staged) {
    Serial.print(kLogPrefix);
    Serial.print(" transfer offset expected=");
    Serial.print(
        static_cast<unsigned long>(g_transfer.position + g_transfer.staged));
    Serial.print(" got=");
    Serial.println(static_cast<unsigned long>(offset));
    queueStatus(kResultErrOffset, g_transfer.position + g_transfer.staged);
    endSession();
    return;
  }
  if (payload > blockLength() - g_transfer.staged) {
    refuse(kResultErrParam);
    return;
  }

  memcpy(g_block.get() + g_transfer.staged, frame + kOffsetBytes, payload);
  g_transfer.staged += payload;
  g_transfer.crc32 =
      crc32Update(g_transfer.crc32, frame + kOffsetBytes, payload);

  if (g_transfer.staged == blockLength()) {
    g_work = Work::CommitBlock;
  }
}

/** @brief Handle every write the phone makes to the transfer service. */
void onCharacteristicWritten(BLEDevice, BLECharacteristic characteristic) {
  const uint8_t* value = characteristic.value();
  const size_t length = static_cast<size_t>(characteristic.valueLength());

  switch (characteristicCode(characteristic)) {
    case kCodeControl:
      onControlWritten(value, length);
      break;
    case kCodeData:
      onDataWritten(value, length);
      break;
    case kCodeFileCrc:
      g_pending_crc32 = readU32(characteristic);
      break;
    case kCodeTotalLength:
      g_session.totalLength = readU32(characteristic);
      break;
    case kCodeFsName:
      readModelName(characteristic);
      break;
    case kCodeInputShape:
      readShape(characteristic, g_session.inputShape);
      break;
    case kCodeOutputShape:
      readShape(characteristic, g_session.outputShape);
      break;
    case kCodeFlashAddress:
      g_session.flashOffset = readU32(characteristic);
      g_session.flashOffsetSet = true;
      break;
    case kCodeIsEdgeLearned:
      g_session.isEdgeLearned = readU32(characteristic);
      break;
    case kCodeNumEdgeClasses:
      g_session.numEdgeClasses = readU32(characteristic);
      break;
    case kCodeMfccFs:
      g_session.mfccFsBits = readU32(characteristic);
      break;
    case kCodeSilenceClass:
      g_session.silenceClass = readU32(characteristic);
      break;
    case kCodeUnknownClass:
      g_session.unknownClass = readU32(characteristic);
      break;
    case kCodeInferenceMode:
      g_session.inferenceMode = readU32(characteristic);
      break;
    default:
      break;
  }
}

/**
 * @brief Take the record naming the stored model away, then answer the START.
 *
 * The moment the first block is written, any record describing that flash is a
 * lie: it names a length and a CRC for bytes that are no longer there. Taking
 * it away first means a transfer that stops part way boots into "there is no
 * model", which is the truth and a state this sketch already handles.
 */
void armDataTransfer() {
  forgetLoadedModel();

  BridgeHold bridge(*g_board);
  if (!bridge.held() ||
      !g_board->eraseExternalData(recordAddress(g_session.app), kRecordBytes)) {
    refuse(kResultErrFlash);
    return;
  }
  queueStatus(kResultOk, 0u);
}

/**
 * @brief Check the info half the phone has just finished sending.
 *
 * Nothing is written here. The info is kept until the data half completes,
 * because the record that carries it into flash also carries the length and
 * the CRC of the data, and those are not known yet.
 */
void completeInfoTransfer() {
  const uint32_t actual = computeCombinedCrc32(g_block.get(), g_transfer.total);
  if (actual != g_session.infoCrc32) {
    Serial.print(kLogPrefix);
    Serial.print(" transfer info crc=0x");
    Serial.print(actual, HEX);
    Serial.print(" expected=0x");
    Serial.println(g_session.infoCrc32, HEX);
    refuse(kResultErrIntegrity);
    return;
  }
  if (!programInfoAcceptable(g_block.get(), g_transfer.total)) {
    Serial.print(kLogPrefix);
    Serial.println(" model refused: not a program this engine can load");
    refuse(kResultErrParam);
    return;
  }
  // The slot is settled here, before any of the data half arrives, so a model
  // no pipeline can feed is refused while it is still cheap to refuse.
  if (!appForInputShape(g_session.inputShape, &g_session.app)) {
    Serial.print(kLogPrefix);
    Serial.print(" model refused: no application takes input shape ");
    Serial.print(static_cast<unsigned long>(g_session.inputShape[0]));
    Serial.print("x");
    Serial.print(static_cast<unsigned long>(g_session.inputShape[1]));
    Serial.print("x");
    Serial.println(static_cast<unsigned long>(g_session.inputShape[2]));
    refuse(kResultErrParam);
    return;
  }
  Serial.print(kLogPrefix);
  Serial.print(" transfer app=");
  Serial.println(appName(g_session.app));

  g_incoming_info.reset(new uint8_t[g_transfer.total]);
  if (g_incoming_info == nullptr) {
    refuse(kResultErrParam);
    return;
  }
  memcpy(g_incoming_info.get(), g_block.get(), g_transfer.total);

  g_transfer.position = g_transfer.total;
  endTransfer();
  queueStatus(kResultDone, g_transfer.total);
}

/**
 * @brief Write the block just staged into the sector it belongs in.
 *
 * @return True when the sector erased, programmed and read back as written.
 */
bool writeStagedBlock() {
  BridgeHold bridge(*g_board);
  const uint32_t address = transferDataAddress() + g_transfer.position;
  return bridge.held() &&
         g_board->eraseExternalData(address, g_transfer.staged) &&
         g_board->writeExternalData(address, g_block.get(), g_transfer.staged);
}

/**
 * @brief Read the whole stored model back and check it against what arrived.
 *
 * The block by block readback has already proved each sector, so this is the
 * second, independent check the standard asks for: that the file as a whole is
 * in flash and reads as the phone's own CRC says it should.
 *
 * @param firstBytes  Receives the first four bytes of the stored model.
 * @return True when the stored model matches the CRC the phone sent.
 */
bool storedModelMatches(uint8_t* firstBytes) {
  const uint32_t address = transferDataAddress();
  uint32_t crc = 0u;
  for (uint32_t offset = 0u; offset < g_session.dataLength;
       offset += kBlockBytes) {
    const uint32_t remaining = g_session.dataLength - offset;
    const uint32_t chunk = remaining < kBlockBytes ? remaining : kBlockBytes;
    if (!g_board->readExternalData(address + offset, g_block.get(), chunk)) {
      return false;
    }
    if (offset == 0u) {
      memcpy(firstBytes, g_block.get(), 4u);
    }
    crc = crc32Update(crc, g_block.get(), chunk);
  }

  if (crc != g_session.dataCrc32) {
    Serial.print(kLogPrefix);
    Serial.print(" stored model crc=0x");
    Serial.print(crc, HEX);
    Serial.print(" expected=0x");
    Serial.println(g_session.dataCrc32, HEX);
    return false;
  }
  return true;
}

/**
 * @brief Write the record and the program info into the sector that opens the
 *        slot.
 *
 * @return True when the sector erased, programmed and read back as written.
 */
bool storeRecord() {
  memcpy(g_block.get(), &g_record, sizeof(Record));
  memcpy(g_block.get() + sizeof(Record), g_incoming_info.get(),
         g_session.infoLength);
  const size_t used = sizeof(Record) + g_session.infoLength;
  const uint32_t address = recordAddress(g_session.app);
  return g_board->eraseExternalData(address, kRecordBytes) &&
         g_board->writeExternalData(address, g_block.get(), used);
}

/**
 * @brief Check and store the model whose last block has just been committed.
 *
 * The record is written only once the bytes have been proved to be in flash,
 * so nothing claims a model is present until one is.
 */
void completeDataTransfer() {
  if (g_transfer.crc32 != g_session.dataCrc32) {
    Serial.print(kLogPrefix);
    Serial.print(" transfer data crc=0x");
    Serial.print(g_transfer.crc32, HEX);
    Serial.print(" expected=0x");
    Serial.println(g_session.dataCrc32, HEX);
    refuse(kResultErrIntegrity);
    return;
  }

  const uint32_t started = millis();
  uint8_t firstBytes[4] = {0};
  BridgeHold bridge(*g_board);
  if (!bridge.held() || !storedModelMatches(firstBytes)) {
    refuse(kResultErrIntegrity);
    return;
  }

  fillRecord(firstBytes);
  if (!storeRecord()) {
    refuse(kResultErrFlash);
    return;
  }

  Serial.print(kLogPrefix);
  Serial.print(" model stored bytes=");
  Serial.print(
      static_cast<unsigned long>(g_session.infoLength + g_session.dataLength));
  Serial.print(" checked_and_recorded_ms=");
  Serial.println(static_cast<unsigned long>(millis() - started));

  endTransfer();
  queueStatus(kResultDone, g_transfer.total);
  g_work = Work::InstallModel;
}

/** @brief Write the block just staged, and say what to do next. */
void commitStagedBlock() {
  const uint32_t started = millis();
  if (!writeStagedBlock()) {
    Serial.print(kLogPrefix);
    Serial.print(" block write failed detail=");
    g_board->printLastError(Serial);
    refuse(kResultErrFlash);
    return;
  }

  g_transfer.position += g_transfer.staged;
  g_transfer.staged = 0u;
  Serial.print(kLogPrefix);
  Serial.print(" block committed position=");
  Serial.print(static_cast<unsigned long>(g_transfer.position));
  Serial.print(" of ");
  Serial.print(static_cast<unsigned long>(g_transfer.total));
  Serial.print(" ms=");
  Serial.println(static_cast<unsigned long>(millis() - started));

  if (g_transfer.position < g_transfer.total) {
    queueStatus(kResultOk, g_transfer.position);
    return;
  }
  completeDataTransfer();
}

/**
 * @brief Load the model the record describes, reading its data from flash.
 *
 * @param programInfo   The program info half, which the engine keeps a pointer
 *                      to for as long as the model stays loaded.
 * @param programBytes  Its length.
 * @return True when the runner accepted it.
 */
bool loadModelFromFlash(const uint8_t* programInfo, size_t programBytes) {
  const BB15Model model = BB15Model::fromExternalFlash(
      programInfo, programBytes, g_record.dataAddress);
  return g_runner->loadModel(model) == BB15Status::Ok;
}

/**
 * @brief Run one inference on a blank input, to prove the model really runs.
 *
 * A model can be stored, verified and programmed and still not run, so the
 * board says it is ready only after it has scored something with it.
 *
 * @return True when the engine returned a result.
 */
bool modelInfers() {
  const akida::Shape dimensions = g_runner->modelInfo().input.dimensions;
  const size_t elements = akida::shape_size(dimensions);
  if (elements == 0u) {
    return false;
  }
  std::unique_ptr<uint8_t[]> blank(new uint8_t[elements]());
  return g_runner->infer(blank.get(), dimensions).ok();
}

/**
 * @brief Program the BrainBoard with the model just stored and prove it runs.
 *
 * The info blob is kept whatever happens, because the engine holds a pointer
 * to whatever it was last programmed from.
 */
void installStoredModel() {
  const uint32_t started = millis();
  std::unique_ptr<uint8_t[]> info = std::move(g_incoming_info);
  const bool ready =
      loadModelFromFlash(info.get(), g_record.infoLength) && modelInfers();
  g_loaded_info = std::move(info);

  Serial.print(kLogPrefix);
  Serial.print(" install ok=");
  Serial.print(ready ? 1 : 0);
  Serial.print(" ms=");
  Serial.println(static_cast<unsigned long>(millis() - started));
  if (!ready) {
    Serial.print(kLogPrefix);
    Serial.print(" install detail=");
    g_board->printLastError(Serial);
    queueStatus(kResultErrProgram, g_transfer.total);
    endSession();
    return;
  }

  g_loaded = describe(g_loaded_info.get());
  g_loaded.app = g_session.app;
  g_installed[static_cast<size_t>(g_session.app)] = summarize(g_record);
  queueStatus(kResultReady, g_transfer.total);
  endSession();
  if (g_on_loaded != nullptr) {
    g_on_loaded(g_loaded);
  }
}

/**
 * @brief Say why a slot is being treated as empty, when it is not erased.
 *
 * An erased slot is the ordinary case and says nothing. A slot that carries a
 * record this firmware cannot read is not ordinary: it means a model is in
 * flash that the board is about to ignore, and the commonest reason is a
 * record an older firmware wrote. Saying which it is turns a silently missing
 * application into something a reader can act on.
 *
 * @param app     Slot that was read.
 * @param record  What was read from the head of it.
 */
void reportUnusableRecord(App app, const Record& record) {
  if (memcmp(record.magic, "BB15", 4) != 0) {
    return;
  }

  Serial.print(kLogPrefix);
  Serial.print(" slot app=");
  Serial.print(appName(app));
  if (record.version != kRecordVersion) {
    Serial.print(" ignored: record version ");
    Serial.print(static_cast<unsigned long>(record.version));
    Serial.print(", this firmware writes ");
    Serial.print(static_cast<unsigned long>(kRecordVersion));
    Serial.println(". Send the model again to replace it.");
    return;
  }
  Serial.println(" ignored: the record is damaged. Send the model again.");
}

/**
 * @brief Read the installed model's record and program info out of the slot.
 *
 * The bridge has to be taken over for the read: the memory mapped path the
 * board uses otherwise is not up on a cold boot, and the read simply fails.
 *
 * @param app          Slot to read.
 * @param programInfo  Receives the program info half.
 * @return True when a usable record and its program info were read.
 */
bool readInstalledModel(App app, std::unique_ptr<uint8_t[]>* programInfo) {
  BridgeHold bridge(*g_board);
  if (!bridge.held()) {
    return false;
  }

  std::unique_ptr<uint8_t[]> sector(new uint8_t[kRecordBytes]);
  if (!g_board->readExternalData(recordAddress(app), sector.get(),
                                 kRecordBytes)) {
    return false;
  }
  memcpy(&g_record, sector.get(), sizeof(Record));
  if (!recordUsable(g_record)) {
    reportUnusableRecord(app, g_record);
    return false;
  }

  uint8_t firstBytes[4] = {0};
  if (!g_board->readExternalData(g_record.dataAddress, firstBytes,
                                 sizeof(firstBytes)) ||
      memcmp(firstBytes, g_record.dataFirstBytes, sizeof(firstBytes)) != 0) {
    Serial.print(kLogPrefix);
    Serial.println(
        " model refused: flash does not hold the model the record "
        "describes");
    return false;
  }

  programInfo->reset(new uint8_t[g_record.infoLength]);
  memcpy(programInfo->get(), sector.get() + sizeof(Record),
         g_record.infoLength);
  return true;
}

/** @brief Abandon a transfer whose phone has gone away. */
void dropTransferIfDisconnected() {
  if (g_session.active && !BLE.connected()) {
    Serial.print(kLogPrefix);
    Serial.println(" transfer abandoned: the phone disconnected");
    endSession();
  }
}

/** @brief Carry out whatever the last Bluetooth write asked poll() to do. */
void runQueuedWork() {
  const Work work = g_work;
  g_work = Work::None;
  switch (work) {
    case Work::ArmDataTransfer:
      armDataTransfer();
      break;
    case Work::CommitBlock:
      if (g_transfer.type == kTransferInfo) {
        completeInfoTransfer();
      } else {
        commitStagedBlock();
      }
      break;
    case Work::InstallModel:
      installStoredModel();
      break;
    case Work::None:
      break;
  }
}

/** @brief Send the status the work just done left waiting. */
void sendQueuedStatus() {
  if (g_pending_result == kResultNone) {
    return;
  }
  const uint8_t result = g_pending_result;
  g_pending_result = kResultNone;
  notifyStatus(result, g_pending_position);
}

}  // namespace

void begin(BB15& board, BB15Runner& runner) {
  g_board = &board;
  g_runner = &runner;

  BLECharacteristic* const written[] = {&g_data,
                                        &g_control,
                                        &g_app_index,
                                        &g_file_crc,
                                        &g_input_shape,
                                        &g_output_shape,
                                        &g_flash_address,
                                        &g_total_length,
                                        &g_is_edge_learned,
                                        &g_num_edge_classes,
                                        &g_fs_name,
                                        &g_mfcc_fs,
                                        &g_silence_class,
                                        &g_unknown_class,
                                        &g_inference_mode};

  g_service.addCharacteristic(g_status);
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

size_t readInstalled() {
  size_t found = 0u;
  for (size_t index = 0u; index < kAppCount; ++index) {
    const App app = static_cast<App>(index);
    std::unique_ptr<uint8_t[]> programInfo;
    g_installed[index] = Installed();
    if (!readInstalledModel(app, &programInfo)) {
      continue;
    }
    g_installed[index] = summarize(g_record);
    ++found;

    Serial.print(kLogPrefix);
    Serial.print(" slot app=");
    Serial.print(appName(app));
    Serial.print(" model=");
    Serial.print(g_installed[index].name);
    Serial.print(" classes=");
    Serial.print(g_installed[index].classCount);
    Serial.print(" program_bytes=");
    Serial.println(static_cast<unsigned long>(g_installed[index].programBytes));
  }
  return found;
}

const Installed& installed(App app) {
  return g_installed[static_cast<size_t>(app)];
}

bool load(App app) {
  if (g_loaded.valid && g_loaded.app == app) {
    return true;
  }
  if (!g_installed[static_cast<size_t>(app)].present) {
    return false;
  }

  const uint32_t started = millis();
  std::unique_ptr<uint8_t[]> programInfo;
  if (!readInstalledModel(app, &programInfo)) {
    return false;
  }
  if (!programInfoAcceptable(programInfo.get(), g_record.infoLength)) {
    Serial.print(kLogPrefix);
    Serial.println(" model refused: not a program this engine can load");
    return false;
  }

  // The outgoing model goes first: the engine holds a pointer into the info
  // blob it was last programmed from, so that has to outlive the swap.
  g_loaded = Loaded();
  const bool ready =
      loadModelFromFlash(programInfo.get(), g_record.infoLength) &&
      modelInfers();
  g_loaded_info = std::move(programInfo);
  if (!ready) {
    Serial.print(kLogPrefix);
    Serial.print(" load failed app=");
    Serial.print(appName(app));
    Serial.print(" detail=");
    g_board->printLastError(Serial);
    return false;
  }

  g_loaded = describe(g_loaded_info.get());
  g_loaded.app = app;

  Serial.print(kLogPrefix);
  Serial.print(" loaded app=");
  Serial.print(appName(app));
  Serial.print(" ms=");
  Serial.println(static_cast<unsigned long>(millis() - started));

  if (g_on_loaded != nullptr) {
    g_on_loaded(g_loaded);
  }
  return true;
}

void poll() {
  dropTransferIfDisconnected();
  runQueuedWork();
  sendQueuedStatus();
}

bool transferInProgress() { return g_session.active; }

const Loaded& loaded() { return g_loaded; }

}  // namespace model
