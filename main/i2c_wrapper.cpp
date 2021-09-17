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

#include "i2c_wrapper.hpp"

#include <vector>

namespace I2C {
CommandHandle::CommandHandle() { handle_ = i2c_cmd_link_create(); }

CommandHandle::~CommandHandle() { i2c_cmd_link_delete(handle_); }

I2C::I2C(i2c_port_t port, int sda, int scl, int clk_speed) {
  port_ = port;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = sda;
  conf.scl_io_num = scl;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = clk_speed;
  conf.clk_flags = 0;
  ESP_ERROR_CHECK(i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0));
  ESP_ERROR_CHECK(i2c_param_config(port, &conf));
}

I2C::~I2C() { i2c_driver_delete(port_); }

void I2C::Send(Address addr, const std::vector<uint8_t> data) {
  CommandHandle cmd;
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  SetAddress(cmd, addr, I2C_MASTER_WRITE);
  ESP_ERROR_CHECK(i2c_master_write(cmd, data.data(), data.size(), true));
  ESP_ERROR_CHECK(i2c_master_stop(cmd));
  ESP_ERROR_CHECK(i2c_master_cmd_begin(port_, cmd, 1 / portTICK_RATE_MS));
}

std::vector<uint8_t> I2C::Read(Address addr, size_t len) {
  CommandHandle cmd;
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  SetAddress(cmd, addr, I2C_MASTER_READ);
  std::vector<uint8_t> buf(len);
  ESP_ERROR_CHECK(i2c_master_read(cmd, buf.data(), len - 1, I2C_MASTER_ACK));
  ESP_ERROR_CHECK(
      i2c_master_read_byte(cmd, buf.data() + len - 1, I2C_MASTER_NACK));
  ESP_ERROR_CHECK(i2c_master_stop(cmd));
  ESP_ERROR_CHECK(i2c_master_cmd_begin(port_, cmd, 1 / portTICK_RATE_MS));
  return buf;
}

void I2C::SetAddress(CommandHandle &cmd, Address addr, i2c_rw_t rw) {
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (addr << 1) | rw, true));
}

};  // namespace I2C
