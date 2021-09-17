// Copyright 2020 Mei Akizuru (mayth)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __SHT3X_HPP__
#define __SHT3X_HPP__

#include <cstdint>
#include <stdexcept>
#include <string>

#include "i2c_wrapper.hpp"

namespace SHT3x {
/// Represents an I2C slave address for SHT3x.
enum class Address : I2C::Address {
  /// Use address A, 0x44.
  /// Note: ADDR pin is low.
  Addr44 = 0x44,
  /// Use address B, 0x45.
  /// Note: ADDR pin is high.
  Addr45 = 0x45
};

/// Represents a repeatability.
enum class Repeatability : uint8_t { High, Medium, Low };

/// Represents a verification error.
class VerificationError : public std::runtime_error {
 public:
  /// Initializes a new VerificationError.
  VerificationError()
      : std::runtime_error::runtime_error("verification failed") {}
  /// Initializes a new VerificationError with message.
  VerificationError(const std::string& what)
      : std::runtime_error::runtime_error("verification failed: " + what) {}
};

/// Represents a measurement result.
class Result {
 public:
  /// Initializes a new measurement result with specified values.
  /// \param raw_temperature A sensor value of temperature.
  /// \param raw_humidity    A sensor value of humidity.
  Result(uint16_t raw_temperature, uint16_t raw_humidity) {
    temp_ = raw_temperature;
    humi_ = raw_humidity;
  }

  /// Gets a raw value of temperature.
  uint16_t raw_temperature() const noexcept { return temp_; }

  /// Gets a raw value of humidity.
  uint16_t raw_humidity() const noexcept { return humi_; }

  /// Gets a calculated value of temperature.
  float temperature() const noexcept {
    return -45.0 + 175.0 * ((float)temp_ / 65535.0);
  }

  /// Gets a calculated value of humidity.
  float humidity() const noexcept { return 100.0 * ((float)humi_ / 65535.0); }

 private:
  /// A sensor value of temperature.
  uint16_t temp_;
  /// A sensor value of humidity.
  uint16_t humi_;

  /// Initializes a new empty result.
  Result() {
    temp_ = 0;
    humi_ = 0;
  }
};

/// Communicates with SHT3x sensor.
class SHT3x {
 public:
  /// Initializes a new SHT3x instance.
  SHT3x(I2C::I2C& i2c, Address addr) : i2c_(i2c), addr_(addr) {}

  /// Measures temperature and humidity.
  /// \param rep A repeatability precision.
  /// \throw VerificationError if CRC checksum failed.
  Result Measure(Repeatability rep);

 private:
  I2C::I2C& i2c_;
  Address addr_;
};

};  // namespace SHT3x

#endif