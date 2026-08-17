#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace bb15 {

class PioExpander6408 {
 public:
  enum class PinMode : uint8_t {
    Input = 0,
    Output = 1,
  };

  explicit PioExpander6408(TwoWire& bus, uint8_t address)
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
           writeRegister(kRegInterruptMask, 0xFFu);
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

 private:
  static constexpr uint8_t kRegDirection = 0x03u;
  static constexpr uint8_t kRegOutputState = 0x05u;
  static constexpr uint8_t kRegOutputHighZ = 0x07u;
  static constexpr uint8_t kRegInterruptMask = 0x11u;

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

}  // namespace bb15
