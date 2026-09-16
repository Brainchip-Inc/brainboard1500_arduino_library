#include "vision_pipeline.h"

#include <math.h>
#include <string.h>

#include <memory>

#include "akida/program_info.h"
#include "camera.h"
#include "gc2145.h"

namespace vision {
namespace {

constexpr int32_t kCameraResolution = CAMERA_R320x240;
constexpr int32_t kCameraImageMode = CAMERA_RGB565;
constexpr int32_t kCameraFrameRate = 30;
constexpr uint16_t kCameraWidth = 320u;
constexpr uint16_t kCameraHeight = 240u;
constexpr size_t kCaptureBytes =
    static_cast<size_t>(kCameraWidth) * kCameraHeight * 2u;
constexpr uint32_t kGrabTimeoutMs = 3000u;

// The camera DMA writes straight into this buffer, and the driver refuses one
// that is not on a cache line.
constexpr size_t kCaptureAlignment = 32u;

constexpr uint16_t kModelChannels = 3u;
constexpr size_t kModelInputBytes =
    static_cast<size_t>(kFrameWidth) * kFrameHeight * kModelChannels;

// Longest an enqueued inference is waited for before the frame is abandoned.
constexpr uint32_t kResultTimeoutMs = 250u;

// The model scores two classes, and this is the one that means a person.
constexpr uint8_t kPersonClass = 1u;
constexpr const char* kClassLabels[] = {"no_person", "person"};
constexpr uint8_t kClassLabelCount =
    sizeof(kClassLabels) / sizeof(kClassLabels[0]);
constexpr uint8_t kMaxClasses = 8u;

/** @brief Where the pipeline is in the grab, score, report cycle. */
enum class Stage : uint8_t {
  Grab,
  AwaitResult,
};

ModelParameters g_model;
BB15Runner* g_runner = nullptr;
const int32_t* g_output_shifts = nullptr;
const float* g_output_scales = nullptr;

DetectionHandler g_on_detection = nullptr;
PreviewHandler g_on_preview = nullptr;

// Held only while scoring, because the model transfer needs the same memory.
std::unique_ptr<uint8_t[]> g_capture;

uint8_t g_model_input[kModelInputBytes];
uint8_t g_preview[kPreviewBytes];
uint16_t g_crop_x[kFrameWidth];
uint16_t g_crop_y[kFrameHeight];
bool g_crop_maps_ready = false;

Stage g_stage = Stage::Grab;
uint32_t g_result_deadline_ms = 0u;
bool g_running = false;
bool g_streaming = false;
bool g_camera_ready = false;

/** @brief The camera driver's sensor, made on first use. */
GC2145& sensor() {
  static GC2145 instance;
  return instance;
}

/** @brief The camera, made on first use. */
Camera& camera() {
  static Camera instance(sensor());
  return instance;
}

/** @brief The frame the camera writes into, pointed at the owned buffer. */
FrameBuffer& framebuffer() {
  static FrameBuffer instance;
  return instance;
}

/**
 * @brief Work out which source pixel each model pixel comes from.
 *
 * The model takes a square image and the sensor gives a 4:3 one, so the
 * widest centred square is taken and sampled down. The maps only depend on
 * the two resolutions, so they are built once.
 */
void prepare_crop_maps() {
  if (g_crop_maps_ready) {
    return;
  }
  constexpr uint16_t kCropSize = kCameraHeight;
  constexpr uint16_t kCropOriginX = (kCameraWidth - kCropSize) / 2u;
  for (uint16_t x = 0u; x < kFrameWidth; ++x) {
    g_crop_x[x] = static_cast<uint16_t>(
        kCropOriginX + (static_cast<uint32_t>(x) * kCropSize) / kFrameWidth);
  }
  for (uint16_t y = 0u; y < kFrameHeight; ++y) {
    g_crop_y[y] = static_cast<uint16_t>((static_cast<uint32_t>(y) * kCropSize) /
                                        kFrameHeight);
  }
  g_crop_maps_ready = true;
}

/**
 * @brief Take the camera's frame buffer, or give it back.
 *
 * @param wanted  True to hold the buffer, false to release it.
 * @return True when the buffer is in the state asked for.
 */
bool hold_capture_buffer(bool wanted) {
  if (wanted == (g_capture != nullptr)) {
    return true;
  }
  if (!wanted) {
    g_capture.reset();
    framebuffer().setBuffer(nullptr);
    return true;
  }

  g_capture.reset(new (std::nothrow)
                      uint8_t[kCaptureBytes + kCaptureAlignment]);
  if (g_capture == nullptr) {
    return false;
  }
  uintptr_t start = reinterpret_cast<uintptr_t>(g_capture.get());
  start = (start + kCaptureAlignment - 1u) & ~(kCaptureAlignment - 1u);
  framebuffer().setBuffer(reinterpret_cast<uint8_t*>(start));
  return true;
}

/**
 * @brief Turn one RGB565 capture into the model's input and its preview.
 *
 * The model was trained with the camera mounted USB-side down, so the crop is
 * read out rotated by half a turn. The preview is the luminance of the same
 * pixels, so what the phone shows is what the network was given.
 *
 * @param frame  Whole RGB565 capture from the sensor.
 */
void build_model_input(const uint8_t* frame) {
  prepare_crop_maps();
  for (uint16_t y = 0u; y < kFrameHeight; ++y) {
    for (uint16_t x = 0u; x < kFrameWidth; ++x) {
      const size_t source =
          static_cast<size_t>(g_crop_y[y]) * kCameraWidth + g_crop_x[x];
      const uint16_t packed = (static_cast<uint16_t>(frame[source * 2u]) << 8) |
                              frame[source * 2u + 1u];
      const uint8_t red =
          static_cast<uint8_t>(((packed >> 11) & 0x1Fu) * 255u / 31u);
      const uint8_t green =
          static_cast<uint8_t>(((packed >> 5) & 0x3Fu) * 255u / 63u);
      const uint8_t blue = static_cast<uint8_t>((packed & 0x1Fu) * 255u / 31u);

      const size_t pixel =
          static_cast<size_t>(kFrameHeight - 1u - y) * kFrameWidth +
          (kFrameWidth - 1u - x);
      g_model_input[pixel * kModelChannels] = red;
      g_model_input[pixel * kModelChannels + 1u] = green;
      g_model_input[pixel * kModelChannels + 2u] = blue;
      g_preview[pixel] =
          static_cast<uint8_t>((77u * red + 150u * green + 29u * blue) >> 8);
    }
  }
}

/**
 * @brief Dequantize the raw Akida potentials, as the engine does.
 *
 * @param potentials  One raw potential per class.
 * @param out         Receives the dequantized values.
 */
void dequantize_potentials(const int32_t* potentials, float* out) {
  for (uint8_t index = 0u; index < g_model.classCount; ++index) {
    out[index] =
        static_cast<float>(potentials[index] - g_output_shifts[index]) /
        g_output_scales[index];
  }
}

/**
 * @brief Numerically stable softmax in place.
 *
 * @param values  One value per class, replaced by the normalized result.
 */
void softmax_in_place(float* values) {
  float highest = values[0];
  for (uint8_t index = 1u; index < g_model.classCount; ++index) {
    if (values[index] > highest) {
      highest = values[index];
    }
  }
  float total = 0.0f;
  for (uint8_t index = 0u; index < g_model.classCount; ++index) {
    values[index] = expf(values[index] - highest);
    total += values[index];
  }
  for (uint8_t index = 0u; index < g_model.classCount; ++index) {
    values[index] /= total;
  }
}

/**
 * @brief Grab a frame and hand it to the model.
 *
 * @return True when an inference is now in flight.
 */
bool start_frame() {
  if (camera().grabFrame(framebuffer(), kGrabTimeoutMs) != 0) {
    return false;
  }
  const uint8_t* frame = framebuffer().getBuffer();
  if (frame == nullptr) {
    return false;
  }
  build_model_input(frame);

  BB15Input input;
  input.data = g_model_input;
  input.type = akida::TensorType::uint8;
  input.dimensions = {1u, kFrameHeight, kFrameWidth, kModelChannels};
  return g_runner->enqueue(input) == BB15Status::Ok;
}

/**
 * @brief Collect the result of the frame in flight and report it.
 *
 * @return True when a result was collected.
 */
bool finish_frame() {
  const BB15RunResult result = g_runner->fetch();
  if (result.status == BB15Status::OutputNotReady) {
    return false;
  }
  if (!result.ok() || result.type != akida::TensorType::int32 ||
      result.elementCount() < g_model.classCount) {
    return true;
  }

  float scores[kMaxClasses];
  dequantize_potentials(result.data<int32_t>(), scores);
  softmax_in_place(scores);

  Detection detection;
  detection.classIndex = 0u;
  detection.confidence = scores[0];
  for (uint8_t index = 1u; index < g_model.classCount; ++index) {
    if (scores[index] > detection.confidence) {
      detection.classIndex = index;
      detection.confidence = scores[index];
    }
  }
  detection.person = detection.classIndex == kPersonClass;

  // The result goes first and on its own, so the phone's indicator keeps up
  // with the board even while a preview frame is still going out.
  if (g_on_detection != nullptr) {
    g_on_detection(detection);
  }
  if (g_streaming && g_on_preview != nullptr) {
    g_on_preview(g_preview);
  }
  return true;
}

}  // namespace

bool begin() {
  g_camera_ready =
      camera().begin(kCameraResolution, kCameraImageMode, kCameraFrameRate);
  return g_camera_ready;
}

bool setModel(BB15Runner* runner, const ModelParameters& parameters) {
  g_runner = runner;
  g_output_shifts = nullptr;
  g_output_scales = nullptr;
  g_stage = Stage::Grab;

  if (runner == nullptr) {
    g_model = ModelParameters();
    return false;
  }

  g_model = parameters;
  if (g_model.programInfo == nullptr || g_model.classCount == 0u ||
      g_model.classCount > kMaxClasses) {
    g_runner = nullptr;
    return false;
  }

  const akida::ProgramInfo info(g_model.programInfo, g_model.programInfoBytes,
                                g_model.dataAddress);
  if (!info.is_valid() || info.shifts().size < g_model.classCount ||
      info.scales().size < g_model.classCount) {
    g_runner = nullptr;
    return false;
  }
  g_output_shifts = info.shifts().data;
  g_output_scales = info.scales().data;
  return true;
}

bool modelReady() {
  return g_runner != nullptr && g_output_shifts != nullptr &&
         g_output_scales != nullptr;
}

void setInferenceRunning(bool running) {
  if (running == g_running) {
    return;
  }
  g_running =
      running && modelReady() && g_camera_ready && hold_capture_buffer(true);
  g_stage = Stage::Grab;
  if (!g_running) {
    hold_capture_buffer(false);
  }
}

bool inferenceRunning() { return g_running; }

void setPreviewStreaming(bool streaming) { g_streaming = streaming; }

void setHandlers(DetectionHandler onDetection, PreviewHandler onPreview) {
  g_on_detection = onDetection;
  g_on_preview = onPreview;
}

void standDown() {
  g_running = false;
  g_streaming = false;
  g_stage = Stage::Grab;
  hold_capture_buffer(false);
}

bool poll() {
  if (!g_running) {
    return false;
  }

  if (g_stage == Stage::Grab) {
    if (!start_frame()) {
      return false;
    }
    g_stage = Stage::AwaitResult;
    g_result_deadline_ms = millis() + kResultTimeoutMs;
    return false;
  }

  if (finish_frame()) {
    g_stage = Stage::Grab;
    return true;
  }
  if (static_cast<int32_t>(millis() - g_result_deadline_ms) >= 0) {
    g_stage = Stage::Grab;
  }
  return false;
}

const char* classLabel(uint8_t classIndex) {
  static char fallback[8];
  if (classIndex < kClassLabelCount) {
    return kClassLabels[classIndex];
  }
  snprintf(fallback, sizeof(fallback), "class%u",
           static_cast<unsigned>(classIndex));
  return fallback;
}

uint8_t classCount() { return g_model.classCount; }

}  // namespace vision
