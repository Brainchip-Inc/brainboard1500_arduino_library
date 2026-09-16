#ifndef BB15_NICLA_VISION_CONNECT_KWS_PIPELINE_H_
#define BB15_NICLA_VISION_CONNECT_KWS_PIPELINE_H_

#include <Arduino.h>
#include <BB15.h>

namespace kws {

/** @brief Largest output width this demo will score. */
constexpr uint8_t kMaxClasses = 16u;

/** @brief Reported as the predicted index when nothing has been detected. */
constexpr uint8_t kNoPrediction = 0xFFu;

/** @brief Values in one streamed waveform frame, as 32 min/max pairs. */
constexpr uint16_t kWaveformValues = 64u;

/** @brief Runtime parameters the phone reads and writes over CMD_CONFIG. */
struct Config {
  uint16_t rmsThreshold = 1100u;
  uint16_t debounceMs = 300u;
  float smoothingAlpha = 0.70f;
  float scoreThreshold = 0.50f;
  uint8_t chimingThreshold = 3u;
  uint16_t speechTimeoutMs = 1300u;
};

/**
 * @brief What the pipeline needs to know about the model now loaded.
 *
 * Every field comes from the model's own metadata, which arrives with the
 * model over Bluetooth, so nothing here is hard-coded to one export.
 */
struct ModelParameters {
  const uint8_t* program = nullptr;
  size_t programBytes = 0u;
  uint8_t classCount = 0u;
  uint8_t silenceClass = 0u;
  uint8_t unknownClass = 0u;
  float mfccFullScale = 0.0f;
};

/** @brief One keyword detection, reported as it happens. */
struct Detection {
  uint8_t classIndex = kNoPrediction;
  float confidence = 0.0f;
};

/**
 * @brief Called with the envelope of each completed audio block.
 *
 * @param values  kWaveformValues interleaved minimum and maximum samples.
 */
using WaveformHandler = void (*)(const int16_t* values);

/** @brief Called once per keyword detection. */
using DetectionHandler = void (*)(const Detection& detection);

/**
 * @brief Start the microphone and the MFCC front end.
 *
 * @return True when the microphone is streaming and features can be computed.
 */
bool begin();

/**
 * @brief Adopt a model that the caller has already loaded on the BrainBoard.
 *
 * The runner stays the caller's: the model arrives over Bluetooth, so loading
 * it is the transfer's job and this pipeline only scores with it.
 *
 * @param runner      Runner holding the loaded model, or nullptr to stop.
 * @param parameters  The loaded model's own metadata. Ignored when runner is
 *                    nullptr. `program` must outlive the model.
 * @return True when the model's dequantization could be read and scoring can
 *         start.
 */
bool setModel(BB15Runner* runner, const ModelParameters& parameters);

/** @brief Say whether a model has been adopted and can be scored. */
bool modelReady();

/** @brief Read the parameters the phone can change. */
const Config& config();

/**
 * @brief Adopt a complete set of parameters and clear the detector state.
 *
 * @param updated  Parameters to adopt.
 */
void setConfig(const Config& updated);

/** @brief Restore the parameters this demo ships with. */
void resetConfig();

/**
 * @brief Start or stop turning audio into inferences.
 *
 * @param running  True to score audio, false to discard it.
 */
void setInferenceRunning(bool running);

/** @brief Say whether the detector is scoring audio. */
bool inferenceRunning();

/**
 * @brief Start or stop streaming the waveform envelope.
 *
 * @param streaming  True to call the waveform handler once per block.
 */
void setWaveformStreaming(bool streaming);

/**
 * @brief Register the callbacks the pipeline reports through.
 *
 * @param onWaveform   Called once per block while streaming is on.
 * @param onDetection  Called once per keyword detection.
 */
void setHandlers(WaveformHandler onWaveform, DetectionHandler onDetection);

/**
 * @brief Consume one microphone buffer if the driver has filled one.
 *
 * The only place the pipeline does work, so it never runs inside a Bluetooth
 * callback. Call it often.
 *
 * @return True when a buffer was processed.
 */
bool poll();

/**
 * @brief Label for one class of the loaded model.
 *
 * @param classIndex  Index into the model's output.
 * @return The label, or a numbered placeholder past the known keywords.
 */
const char* classLabel(uint8_t classIndex);

/** @brief Number of classes the loaded model scores. */
uint8_t classCount();

/** @brief Drop any part-built utterance and clear the smoothed scores. */
void reset();

}  // namespace kws

#endif  // BB15_NICLA_VISION_CONNECT_KWS_PIPELINE_H_
