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

// How long an enqueued inference may go uncollected before the pipeline says
// so. It only reports: an enqueued job is never abandoned, because the engine
// answers the next enqueue while one is outstanding by halting the core.
constexpr uint32_t kSlowResultMs = 1000u;

// The model scores two classes, and this is the one that means a person.
constexpr uint8_t kPersonClass = 1u;
constexpr const char* kClassLabels[] = {"no_person", "person"};
constexpr uint8_t kClassLabelCount =
    sizeof(kClassLabels) / sizeof(kClassLabels[0]);
constexpr uint8_t kMaxClasses = 8u;

// The person score has to reach this and hold it for this many frames in a row
// before a detection is reported. Both come from the keyword detector, which
// decides the same way.
constexpr float kScoreThreshold = 0.50f;
constexpr uint8_t kChimingThreshold = 3u;

ModelParameters g_model;
BB15Runner* g_runner = nullptr;
const int32_t* g_output_shifts = nullptr;
const float* g_output_scales = nullptr;

DetectionHandler g_on_detection = nullptr;
PreviewHandler g_on_preview = nullptr;

uint8_t g_chiming = 0u;
uint8_t g_lapsed = 0u;
bool g_armed = true;

// Held only while scoring, because the model transfer needs the same memory.
std::unique_ptr<uint8_t[]> g_capture;

uint8_t g_model_input[kModelInputBytes];
uint8_t g_preview[kPreviewBytes];
uint16_t g_crop_x[kFrameWidth];
uint16_t g_crop_y[kFrameHeight];
bool g_crop_maps_ready = false;

// True from the moment the engine is given a frame until it gives the result
// back. The engine refuses a second enqueue while the first is outstanding by
// halting the core, so this is never cleared without collecting the result.
bool g_inflight = false;
bool g_slow_reported = false;
uint32_t g_enqueued_ms = 0u;
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
 * @brief Collect the outstanding result, so the engine is left ready.
 *
 * Stopping the pipeline with a frame still in the engine is what the phone
 * does every time it switches application. The job has to be collected rather
 * than forgotten: the engine answers the next enqueue while one is
 * outstanding by halting the core, and nothing in a sketch recovers from that.
 *
 * If the result never arrives the frame stays outstanding on purpose, which
 * leaves the pipeline quiet instead of taking the board down.
 */
void drain_inflight() {
  if (!g_inflight) {
    return;
  }
  const uint32_t deadline = millis() + kSlowResultMs;
  while (g_inflight && static_cast<int32_t>(millis() - deadline) < 0) {
    if (g_runner->fetch().status != BB15Status::OutputNotReady) {
      g_inflight = false;
    }
  }
  if (g_inflight) {
    Serial.println(
        "[vision] the engine still holds a frame; no more will be sent");
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
  if (g_runner->enqueue(input) != BB15Status::Ok) {
    return false;
  }
  g_inflight = true;
  g_slow_reported = false;
  g_enqueued_ms = millis();
  return true;
}

/** @brief Clear the decision counters and let the detector fire again. */
void reset_decision_state() {
  g_chiming = 0u;
  g_lapsed = 0u;
  g_armed = true;
}

/**
 * @brief Say whether this frame is the one to report a person on.
 *
 * The score has to hold above the threshold for kChimingThreshold frames,
 * which is how the keyword detector decides. What arms that one again is the
 * word ending; a person does not end, they stay in view, so this one arms
 * again only once the score has fallen back for as many frames as it took to
 * fire. A person is therefore reported when they arrive and not again until
 * they have gone.
 *
 * @param personScore  This frame's score for the person class, or zero when
 *                     the frame went the other way.
 * @return True when this frame should be reported to the phone.
 */
bool apply_decision(float personScore) {
  if (personScore >= kScoreThreshold) {
    g_lapsed = 0u;
    if (g_chiming < kChimingThreshold) {
      ++g_chiming;
    }
    if (g_armed && g_chiming >= kChimingThreshold) {
      g_armed = false;
      return true;
    }
    return false;
  }

  g_chiming = 0u;
  if (g_lapsed < kChimingThreshold) {
    ++g_lapsed;
  }
  if (g_lapsed >= kChimingThreshold) {
    g_armed = true;
  }
  return false;
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

  // The engine has given the frame back, whatever it thinks of it, so the next
  // one may be handed over.
  g_inflight = false;
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
  detection.triggered =
      apply_decision(detection.person ? detection.confidence : 0.0f);

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
  reset_decision_state();
  if (!g_running) {
    drain_inflight();
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
  reset_decision_state();
  drain_inflight();
  hold_capture_buffer(false);
}

bool poll() {
  if (!g_running) {
    return false;
  }
  if (!g_inflight) {
    start_frame();
    return false;
  }

  if (finish_frame()) {
    return true;
  }
  if (!g_slow_reported && static_cast<int32_t>(millis() - g_enqueued_ms) >
                              static_cast<int32_t>(kSlowResultMs)) {
    g_slow_reported = true;
    Serial.println("[vision] the engine is slow to return a frame");
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
