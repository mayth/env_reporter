// Copyright 2021 Mei Akizuru (mayth)
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

#ifndef __I2C_WRAPPER_HPP__
#define __I2C_WRAPPER_HPP__

#include <driver/i2c.h>

#include <vector>

namespace I2C {

typedef uint8_t Address;

/// Wraps i2c_cmd_handle_t for resource management.
class CommandHandle {
  i2c_cmd_handle_t handle_;

 public:
  /// Establishes a new I2C command link and obtain its handle.
  CommandHandle();
  /// Releases the I2C command link.
  ~CommandHandle();

  /// Gets the undelying handle.
  i2c_cmd_handle_t handle() { return handle_; };

  /// Gets the undelying handle.
  operator i2c_cmd_handle_t() { return handle(); }
};

/// Wraps I2C operations.
/// An instance of this class should be reused while using the same
/// port/sda/scl.
class I2C {
 public:
  /// Prepares for a new I2C communication.
  /// \param[in] port       I2C port number. You can use I2C_NUM_*.
  /// \param[in] sda        GPIO number for SDA. You can use GPIO_NUM_*.
  /// \param[in] scl        GPIO number for SCL. You can use GPIO_NUM_*.
  /// \param[in] clk_speed  I2C clock speed.
  I2C(i2c_port_t port, int sda, int scl, int clk_speed);

  /// Closes a I2C communication.
  ~I2C();

  /// Sends data to addr.
  /// \param[in] addr An I2C slave address.
  void Send(Address addr, const std::vector<uint8_t> data);

  /// Reads data from addr.
  /// \param[in] addr An I2C slave address.
  /// \param[in] len  A maximum number of bytes to read.
  std::vector<uint8_t> Read(Address addr, size_t len);

 private:
  /// Sets a slave address. This should be called before read/write.
  void SetAddress(CommandHandle &cmd, Address addr, i2c_rw_t rw);

  i2c_port_t port_;
};
};  // namespace I2C

#endif