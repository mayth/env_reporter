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

#include "sht3x.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <tuple>

#include "i2c_wrapper.hpp"

namespace {
/// A polynomial used for calculating CRC-8.
const static uint8_t kCRCPolynomial = 0x31;

/// Command table for single shot mode.
const std::map<std::tuple<SHT3x::Repeatability, bool>, std::vector<uint8_t>>
    kSingleShotCommandTable = {
        {std::make_pair(SHT3x::Repeatability::High, true), {0x2C, 0x06}},
        {std::make_pair(SHT3x::Repeatability::Medium, true), {0x2C, 0x0D}},
        {std::make_pair(SHT3x::Repeatability::Low, true), {0x2C, 0x10}},
        {std::make_pair(SHT3x::Repeatability::High, false), {0x24, 0x00}},
        {std::make_pair(SHT3x::Repeatability::Medium, false), {0x24, 0x0B}},
        {std::make_pair(SHT3x::Repeatability::Low, false), {0x24, 0x16}}};

/// A table of maximum wait for measurement.
const std::map<SHT3x::Repeatability, int> kMeasurementWait = {
    {SHT3x::Repeatability::High, 15},
    {SHT3x::Repeatability::Medium, 6},
    {SHT3x::Repeatability::Low, 4}};

/// Calculates a CRC-8 for the given data.
/// \param[in] data A byte array to calculate CRC.
/// \tparam N  A length of data.
/// \return A calculated CRC value.
template <std::size_t N>
uint8_t CalculateCRC(const std::array<uint8_t, N> &data) {
  uint8_t crc = 0xFF;
  for (auto b : data) {
    crc ^= b;
    for (int i = 0; i < 8; ++i) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ kCRCPolynomial;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

/// Verifies the data using CRC.
///
/// \param[in] data     A byte vector to be verified.
/// \param[in] read_crc A CRC checksum read from the device.
/// \return true if successfully verified; otherwise, false.
template <std::size_t N>
bool VerifyData(const std::array<uint8_t, N> &data, uint8_t read_crc) {
  return read_crc == CalculateCRC(data);
}

/// Converts a byte array into a uint16_t.
///
/// \param[in] bytes A byte array to be converted.
constexpr uint16_t ToUint16(const std::array<uint8_t, 2> &bytes) {
  return ((uint16_t)bytes[0] << 8) | bytes[1];
}
};  // namespace

namespace SHT3x {
Result SHT3x::Measure(Repeatability rep) {
  auto cmd = kSingleShotCommandTable.at(std::make_pair(rep, false));
  i2c_.Send(static_cast<I2C::Address>(addr_), cmd);

  vTaskDelay(kMeasurementWait.at(rep) / portTICK_PERIOD_MS);

  auto res = i2c_.Read(static_cast<I2C::Address>(addr_), 6);
  std::array<uint8_t, 2> raw_temp = {res[0], res[1]};
  uint8_t crc_temp = res[2];
  std::array<uint8_t, 2> raw_humi = {res[3], res[4]};
  uint8_t crc_humi = res[5];

  if (!VerifyData(raw_temp, crc_temp)) {
    throw VerificationError("temperature");
  }
  if (!VerifyData(raw_humi, crc_humi)) {
    throw VerificationError("humidity");
  }
  return Result(ToUint16(raw_temp), ToUint16(raw_humi));
}
};  // namespace SHT3x