#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace bb15_demo {

// Minimal helper for the BB15 I2C GPIO expander at 0x43.
//
// Typical use:
//   bb15_demo::PioExpander6408 expander(Wire);
//   expander.probe();
//   expander.pinMode(0, bb15_demo::PioExpander6408::PinMode::Output);
//   expander.digitalWrite(0, false);
class PioExpander6408 {
 public:
  enum class PinMode : uint8_t {
    Input = 0,
    Output = 1,
  };

  static constexpr uint8_t kDefaultAddress = 0x43u;

  explicit PioExpander6408(TwoWire& bus, uint8_t address = kDefaultAddress)
      : bus_(bus), address_(address) {}

  bool probe() {
    bus_.beginTransmission(address_);
    return bus_.endTransmission() == 0u;
  }

  bool configureBb15DefaultOutputs() {
    return writeRegister(kRegOutputState, 0x00u) &&
           writeRegister(kRegDirection, 0xFFu) &&
           writeRegister(kRegOutputHighZ, 0x00u) &&
           writeRegister(kRegOutputState, 0x02u) &&
           writeRegister(0x11u, 0xFFu);
  }

  bool pinMode(uint8_t pin, PinMode mode) {
    const uint8_t mask = pinMask(pin);
    if (mask == 0u) {
      return false;
    }

    uint8_t direction = 0u;
    uint8_t output_high_z = 0u;
    if (!readRegister(kRegDirection, direction) ||
        !readRegister(kRegOutputHighZ, output_high_z)) {
      return false;
    }

    if (mode == PinMode::Output) {
      direction |= mask;
      output_high_z &= static_cast<uint8_t>(~mask);
    } else {
      direction &= static_cast<uint8_t>(~mask);
    }

    return writeRegister(kRegDirection, direction) &&
           writeRegister(kRegOutputHighZ, output_high_z);
  }

  bool digitalWrite(uint8_t pin, bool high) {
    const uint8_t mask = pinMask(pin);
    if (mask == 0u) {
      return false;
    }

    uint8_t output_state = 0u;
    if (!readRegister(kRegOutputState, output_state)) {
      return false;
    }

    if (high) {
      output_state |= mask;
    } else {
      output_state &= static_cast<uint8_t>(~mask);
    }
    return writeRegister(kRegOutputState, output_state);
  }

  bool readDeviceIdControl(uint8_t& value) {
    return readRegister(kRegDeviceIdControl, value);
  }

 private:
  static constexpr uint8_t kRegDeviceIdControl = 0x01u;
  static constexpr uint8_t kRegDirection = 0x03u;
  static constexpr uint8_t kRegOutputState = 0x05u;
  static constexpr uint8_t kRegOutputHighZ = 0x07u;

  static uint8_t pinMask(uint8_t pin) {
    return (pin < 8u) ? static_cast<uint8_t>(1u << pin) : 0u;
  }

  bool writeRegister(uint8_t reg, uint8_t value) {
    bus_.beginTransmission(address_);
    bus_.write(reg);
    bus_.write(value);
    return bus_.endTransmission() == 0u;
  }

  bool readRegister(uint8_t reg, uint8_t& value) {
    bus_.beginTransmission(address_);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0u) {
      return false;
    }
    if (bus_.requestFrom(static_cast<int>(address_), 1) != 1) {
      return false;
    }
    value = static_cast<uint8_t>(bus_.read());
    return true;
  }

  TwoWire& bus_;
  uint8_t address_;
};

// Board bring-up helper for the Nicla Vision + BB15 demo wiring.
//
// Typical use in a sketch:
//   bb15_demo::Bb15NiclaVisionBoard board;
//   board.printSummary(Serial);
//   if (!board.begin()) {
//     Serial.println(board.lastError());
//   }
//
// `begin()` handles the minimum required demo boot sequence:
// - release bridge power-down on PA_10/D2
// - configure the BB15 expander defaults
// - strap AKD1500 external-flash mode
// - release AKD1500 reset
class Bb15NiclaVisionBoard {
 public:
  explicit Bb15NiclaVisionBoard(TwoWire& bus = Wire)
      : bus_(bus), expander_(bus) {}

  bool begin() {
    // Release the companion board's power-down line before reconfiguring any
    // of the shared pins.
    releasePowerDown();

    // Free the bridge-chip UART pins so they can be reused for AKD1500/flash.
    detachExternalUartPins();
    delay(100);
    releasePowerDown();

    // Bring up the BB15 control bus used for the external GPIO expander.
    bus_.begin();
    bus_.setClock(kI2cClockHz);

    // The demo depends on the 0x43 expander for power/mode strapping.
    if (!expander_.probe()) {
      setError("expander_missing");
      return false;
    }

    // Drive the board defaults that keep the BB15 peripherals in a known state.
    if (!expander_.configureBb15DefaultOutputs()) {
      setError("expander_config_failed");
      return false;
    }

    // Force the AKD1500 boot strap for external-flash execution, then release
    // reset so the library can link to the chip.
    if (!resetAkidaIntoExternalFlashMode()) {
      return false;
    }

    return true;
  }

  bool coldBootAkidaIntoExternalFlashMode(uint32_t hold_reset_ms = 250u) {
    // Re-assert the known board wiring before forcing a cold reset cycle.
    releasePowerDown();
    detachExternalUartPins();

    // Keep the external-flash boot strap active while AKD1500 is held in
    // reset so the next release behaves like a fresh cold boot.
    setAkidaReset(true);
    if (!expander_.pinMode(0u, PioExpander6408::PinMode::Output) ||
        !expander_.digitalWrite(0u, false)) {
      setError("akida_mode_strap_failed");
      return false;
    }

    delay(hold_reset_ms);
    setAkidaReset(false);
    delay(kAkidaResetReleaseSettleMs);
    return true;
  }

  bool holdAkidaInReset() {
    assertPowerDown();
    detachExternalUartPins();
    setAkidaReset(true);
    if (!expander_.pinMode(0u, PioExpander6408::PinMode::Output) ||
        !expander_.digitalWrite(0u, false) || !setAkidaSleepGate(false)) {
      setError("akida_mode_strap_failed");
      return false;
    }
    return true;
  }

  bool setAkidaSleepGate(bool enabled) {
    if (!expander_.pinMode(kAkidaSleepGatePin,
                           PioExpander6408::PinMode::Output) ||
        !expander_.digitalWrite(kAkidaSleepGatePin, enabled)) {
      setError("akida_sleep_gate_failed");
      return false;
    }
    return true;
  }

  const char* lastError() const { return last_error_; }

// Zamijeni u bb15_demo_board.h da bude prazna
  void printSummary(Print& out) const {
    // Prazno radi uštede memorije
  }

private:
  static constexpr uint32_t kI2cClockHz = 100000u;
  static constexpr uint32_t kAkidaResetAssertMs = 5u;
  static constexpr uint32_t kAkidaResetReleaseSettleMs = 30u;
  
  // NOVE KONSTANTE ZA MKR:
  static constexpr uint8_t kPowerDownPin = 14u;     // D14 na MKRu
  static constexpr uint8_t kBridgeCsPin = 13u;      // D13 na MKRu (bivši D1)
  static constexpr uint8_t kAkidaResetPin = 255u;    // 255 znači "zanemari" prema tvojim uputama
  static constexpr uint8_t kAkidaSleepGatePin = 2u;  // Ostaje 2 jer je ovo pin na I2C expanderu, ne na MCU

  void setError(const char* error) { last_error_ = error; }

  void releasePowerDown() {
    pinMode(kPowerDownPin, OUTPUT);
    digitalWrite(kPowerDownPin, HIGH);
  }

  void assertPowerDown() {
    pinMode(kPowerDownPin, OUTPUT);
    digitalWrite(kPowerDownPin, LOW);
  }

  void detachExternalUartPins() {
    // Na MKR-u samo osiguravamo da je Bridge CS (D13) u HIGH stanju radi SPI flasha
    pinMode(kBridgeCsPin, OUTPUT);
    digitalWrite(kBridgeCsPin, HIGH);
  }

  void setAkidaReset(bool asserted) {
    // Ako je pin postavljen na 255, potpuno ignoriramo reset logiku
    if (kAkidaResetPin != 255u) {
      pinMode(kAkidaResetPin, OUTPUT);
      digitalWrite(kAkidaResetPin, asserted ? LOW : HIGH);
    }
  }

  bool resetAkidaIntoExternalFlashMode() {
    // Hold the device in reset before changing its external boot strap.
    releasePowerDown();
    setAkidaReset(true);
    delay(kAkidaResetAssertMs);

    // P0 low is the mode strap needed for the external-flash execution path.
    if (!expander_.pinMode(0u, PioExpander6408::PinMode::Output) ||
        !expander_.digitalWrite(0u, false) || !setAkidaSleepGate(false)) {
      setError("akida_mode_strap_failed");
      return false;
    }

    // Release reset and give the device a short settle time before use.
    setAkidaReset(false);
    delay(kAkidaResetReleaseSettleMs);
    return true;
  }

  TwoWire& bus_;
  PioExpander6408 expander_;
  const char* last_error_ = "ok";
};

}  // namespace bb15_demo
