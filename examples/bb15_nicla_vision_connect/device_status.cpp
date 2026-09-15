#include "device_status.h"

#include <Arduino_PowerManagement.h>
#include <stdio.h>

namespace device {
namespace {

// Capacity of the cell fitted to the board. The fuel gauge needs it to turn
// its voltage and current readings into a state of charge, so a board running
// a different cell must have this changed to match.
constexpr int kBatteryCapacityMah = 200;

// The microcontroller's 96-bit unique id. The app requires 16 lowercase hex
// characters, so the lower 64 bits are reported.
constexpr uint32_t kUniqueIdAddress = 0x1FF1E800u;

constexpr uint32_t kDetectionFlashMs = 180u;
constexpr uint32_t kAdvertisingPeriodMs = 2000u;
constexpr uint32_t kAdvertisingOnMs = 120u;
constexpr uint32_t kTransferPeriodMs = 300u;
constexpr uint32_t kFailedPeriodMs = 600u;

Battery g_battery;
Charger g_charger;
Board g_board;
bool g_gauge_ready = false;
char g_serial[17] = {0};

LedMode g_led_mode = LedMode::Advertising;
uint32_t g_flash_until_ms = 0u;

/** @brief Drive the three LED pins, which are active low. */
void writeLed(bool red, bool green, bool blue) {
  digitalWrite(LEDR, red ? LOW : HIGH);
  digitalWrite(LEDG, green ? LOW : HIGH);
  digitalWrite(LEDB, blue ? LOW : HIGH);
}

/**
 * @brief Map the PMIC's charger state onto the four the phone app knows.
 *
 * @param state  Charger state the PMIC reports.
 * @return The state to put in the battery frame.
 */
BatteryState mapChargerState(ChargingState state) {
  switch (state) {
    case ChargingState::preCharge:
    case ChargingState::fastChargeConstantCurrent:
    case ChargingState::fastChargeConstantVoltage:
      return BatteryState::Charging;
    case ChargingState::timerFaultError:
    case ChargingState::thermistorSuspendError:
      return BatteryState::Warning;
    case ChargingState::batteryOvervoltageError:
      return BatteryState::Fault;
    default:
      return BatteryState::NotCharging;
  }
}

}  // namespace

bool begin() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  writeLed(false, false, false);

  const uint32_t* unique_id =
      reinterpret_cast<const uint32_t*>(kUniqueIdAddress);
  snprintf(g_serial, sizeof(g_serial), "%08lx%08lx",
           static_cast<unsigned long>(unique_id[1]),
           static_cast<unsigned long>(unique_id[0]));

  BatteryCharacteristics characteristics;
  characteristics.capacity = kBatteryCapacityMah;
  g_battery = Battery(characteristics);

  g_gauge_ready = g_battery.begin();
  g_charger.begin();
  g_board.begin();
  return g_gauge_ready;
}

Power readPower() {
  Power power;
  power.usbPowered = g_board.isUSBPowered();
  if (!g_gauge_ready) {
    return power;
  }
  power.percent = g_battery.percentage();
  power.state = mapChargerState(g_charger.getState());
  return power;
}

const char* serialNumber() { return g_serial; }

void setLedMode(LedMode mode) { g_led_mode = mode; }

void flashDetection() { g_flash_until_ms = millis() + kDetectionFlashMs; }

void updateLed() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - g_flash_until_ms) < 0) {
    writeLed(true, false, false);
    return;
  }

  switch (g_led_mode) {
    case LedMode::Advertising:
      writeLed(false, false, (now % kAdvertisingPeriodMs) < kAdvertisingOnMs);
      return;
    case LedMode::Connected:
      writeLed(false, true, false);
      return;
    case LedMode::ModelTransfer:
      writeLed(false, false,
               (now % kTransferPeriodMs) < (kTransferPeriodMs / 2u));
      return;
    case LedMode::Failed:
      writeLed((now % kFailedPeriodMs) < (kFailedPeriodMs / 2u), false, false);
      return;
  }
}

}  // namespace device
