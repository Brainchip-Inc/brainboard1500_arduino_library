#ifndef BB15_NICLA_VISION_CONNECT_BLE_MODEL_TRANSFER_H_
#define BB15_NICLA_VISION_CONNECT_BLE_MODEL_TRANSFER_H_

#include <Arduino.h>
#include <BB15.h>

namespace model {

/** @brief Longest model name the phone app's metadata carries. */
constexpr size_t kMaxNameLength = 64u;

/** @brief A model that is present in BrainBoard flash and loaded. */
struct Loaded {
  bool valid = false;
  const uint8_t* programInfo = nullptr;
  size_t programInfoBytes = 0u;
  uint32_t dataAddress = 0u;
  size_t programBytes = 0u;
  uint8_t classCount = 0u;
  uint8_t silenceClass = 0u;
  uint8_t unknownClass = 0u;
  float mfccFullScale = 0.0f;
  char name[kMaxNameLength] = {0};
};

/**
 * @brief Called whenever the model the board runs changes.
 *
 * It carries a valid model once one is loaded and ready to score with, and an
 * invalid one when the board has stopped having a model to run, which happens
 * as soon as a new transfer starts overwriting the one in flash.
 */
using LoadedHandler = void (*)(const Loaded& loaded);

/** @brief Called when a transfer starts and when it ends, for LED feedback. */
using ActivityHandler = void (*)(bool busy);

/**
 * @brief Publish the model transfer service and take the board to write with.
 *
 * Must be called before BLE.advertise(), because ArduinoBLE builds its
 * attribute table as services are added.
 *
 * @param board   Board whose external flash the model is written to.
 * @param runner  Runner the model is loaded into.
 */
void begin(BB15& board, BB15Runner& runner);

/**
 * @brief Register the callbacks this module reports through.
 *
 * @param onLoaded    Called when the loaded model changes, from poll().
 * @param onActivity  Called with true while a transfer is running.
 */
void setHandlers(LoadedHandler onLoaded, ActivityHandler onActivity);

/**
 * @brief Load the model left in BrainBoard flash by an earlier session.
 *
 * Reads the record written beside the model data and loads what it describes,
 * so a power cycle does not cost the user another transfer.
 *
 * @return True when a valid record was found and its model loaded.
 */
bool restoreFromFlash();

/**
 * @brief Do the work a transfer has queued: write flash, answer, install.
 *
 * Committing a block takes the best part of a second, so it happens here
 * rather than inside the Bluetooth write callback that asked for it.
 */
void poll();

/** @brief Say whether a transfer is under way. */
bool transferInProgress();

/** @brief The model currently loaded, whose `valid` says whether there is one.
 */
const Loaded& loaded();

}  // namespace model

#endif  // BB15_NICLA_VISION_CONNECT_BLE_MODEL_TRANSFER_H_
