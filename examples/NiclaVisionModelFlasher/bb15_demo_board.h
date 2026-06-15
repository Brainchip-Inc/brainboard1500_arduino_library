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
// - hold the proceed GPIO high
// - configure the BB15 expander defaults
// - strap AKD1500 external-flash mode
// - release AKD1500 reset
class Bb15NiclaVisionBoard {
 public:
  explicit Bb15NiclaVisionBoard(TwoWire& bus = Wire)
      : bus_(bus), expander_(bus) {}

  bool begin() {
    // Keep the companion board's proceed line asserted before reconfiguring
    // any of the shared pins.
    assertProceedHigh();

    // Free the bridge-chip UART pins so they can be reused for AKD1500/flash.
    detachExternalUartPins();
    delay(100);
    assertProceedHigh();

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

  const char* lastError() const { return last_error_; }

  void printSummary(Print& out) const {
    out.println("[camera_flash_demo] board=Nicla Vision + BB15");
    out.println("[camera_flash_demo] bus=Wire SDA=D11 SCL=D12");
    out.println("[camera_flash_demo] wiring=proceed=D2/PA_10 HIGH akida_cs=D7 bridge_cs=D1 akida_reset_n=D3 expander=0x43");
  }

 private:
  static constexpr uint32_t kI2cClockHz = 100000u;
  static constexpr uint32_t kAkidaResetAssertMs = 5u;
  static constexpr uint32_t kAkidaResetReleaseSettleMs = 10u;
  static constexpr PinName kProceedGpioPin = PA_10;
  static constexpr uint8_t kAkidaResetPin = 3u;

  void setError(const char* error) { last_error_ = error; }

  void assertProceedHigh() {
    // The demo board expects PA_10 and D2 high during bring-up.
    pinMode(kProceedGpioPin, OUTPUT);
    digitalWrite(kProceedGpioPin, HIGH);
    pinMode(D2, OUTPUT);
    digitalWrite(D2, HIGH);
  }

  void detachExternalUartPins() {
    // Disconnect serial peripherals from the shared bridge pins so the demo
    // can safely repurpose them for flash access.
    Serial1.end();
    Serial2.end();
    pinMode(PA_9, OUTPUT);
    digitalWrite(PA_9, HIGH);
    pinMode(D1, OUTPUT);
    digitalWrite(D1, HIGH);
    assertProceedHigh();
  }

  void setAkidaReset(bool asserted) {
    // The AKD1500 reset line is active-low on this board stack.
    pinMode(kAkidaResetPin, OUTPUT);
    digitalWrite(kAkidaResetPin, asserted ? LOW : HIGH);
  }

  bool resetAkidaIntoExternalFlashMode() {
    // Hold the device in reset before changing its external boot strap.
    setAkidaReset(true);
    delay(kAkidaResetAssertMs);

    // P0 low is the mode strap needed for the external-flash execution path.
    if (!expander_.pinMode(0u, PioExpander6408::PinMode::Output) ||
        !expander_.digitalWrite(0u, false)) {
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
