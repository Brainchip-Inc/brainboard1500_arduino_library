#ifndef BB15_NICLA_VISION_CONNECT_DEVICE_STATUS_H_
#define BB15_NICLA_VISION_CONNECT_DEVICE_STATUS_H_

#include <Arduino.h>

namespace device {

/** @brief Battery state as the phone app's battery frame encodes it. */
enum class BatteryState : uint8_t {
  NotCharging = 0u,
  Charging = 1u,
  Warning = 2u,
  Fault = 3u,
};

/** @brief What the board can say about its power right now. */
struct Power {
  uint8_t percent = 0u;
  BatteryState state = BatteryState::NotCharging;
  // True while the board is running from USB, from the PMIC's VBUS sense.
  bool usbPowered = false;
};

/** @brief What the RGB LED is saying about the demo. */
enum class LedMode : uint8_t {
  Advertising,
  Connected,
  ModelTransfer,
  Failed,
};

/**
 * @brief Start the fuel gauge, the charger and the LED pins.
 *
 * A failure here is not fatal: the demo runs without power reporting, and
 * readPower() then reports no cell.
 *
 * @return True when the fuel gauge answered and was configured.
 */
bool begin();

/**
 * @brief Read the current power state.
 *
 * The percentage comes from the fuel gauge and the charging state from the
 * PMIC. Whether a cell is fitted at all cannot be read on this board while it
 * runs from USB: the gauge sits on the system rail, so with no cell it reports
 * that rail as a full, healthy battery and its battery-present bit agrees.
 *
 * @return The power state to report to the phone.
 */
Power readPower();

/**
 * @brief The board's permanent serial, as the phone app requires it.
 *
 * @return 16 lowercase hex characters, from the microcontroller's unique id.
 */
const char* serialNumber();

/**
 * @brief Set what the LED is saying.
 *
 * @param mode  State to show until the next call.
 */
void setLedMode(LedMode mode);

/**
 * @brief Flash the LED to mark a keyword detection.
 *
 * The flash overrides the current mode briefly, then the mode returns.
 */
void flashDetection();

/**
 * @brief Drive the LED. Call often; it never blocks.
 */
void updateLed();

}  // namespace device

#endif  // BB15_NICLA_VISION_CONNECT_DEVICE_STATUS_H_
