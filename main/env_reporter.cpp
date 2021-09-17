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

#include <NimBLEDevice.h>
#include <driver/i2c.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_sntp.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <nvs_flash.h>

#include <array>
#include <ctime>

#include "sdkconfig.h"
#include "sht3x.hpp"

namespace {
/***** from sdkconfig *****/
/// SSID
const char *kWiFiSSID = CONFIG_ER_WIFI_SSID;
/// WiFi Password
const char *kWiFiPassword = CONFIG_ER_WIFI_PWD;
/// A maximum number of retries to connect to Wi-Fi AP.
const int kWiFiMaxRetries = CONFIG_ER_WIFI_MAX_RETRY;
/// Country code
const char *kWiFiCountryCode = CONFIG_ER_WIFI_COUNTRY;
/// NTP server host name.
const char *kSNTPServer = CONFIG_ER_SNTP_SERVER;
/// A maximum number of retries while waiting for the time synchronization.
const int kSNTPMaxRetries = CONFIG_ER_SNTP_MAX_RETRY;

/// Multiplier to convert seconds to microseconds;
const long kSecondsUS = 1000000L;
/// Multiplier to convert seconds to milliseconds;
const long kSecondsMS = 1000L;

/***** Constants *****/
/// tag for logging.
const char *TAG = "env_reporter";
/// Full name used in BLE advertising.
const std::string kDeviceName = "env_reporter_1";
/// Short name used in BLE advertising.
const std::string kShortDeviceName = "envrp1";

/// Working interval in seconds.
const uint64_t kInterval = 600;
/// Advertising duration in seconds.
const uint64_t kAdvertisingDuration = 120;
/// Deep sleep duration in seconds.
const uint64_t kSleepDuration = kInterval - kAdvertisingDuration;

const i2c_port_t kI2CPort = I2C_NUM_0;
/// GPIO pin number to use as I2C SDA.
const gpio_num_t kI2CSDANum = GPIO_NUM_21;
/// GPIO pin number to use as I2C SCL.
const gpio_num_t kI2CSCLNum = GPIO_NUM_22;
/// I2C address for SHT3x sensor.
const SHT3x::Address kI2CAddress =
#if CONFIG_ER_SHT3X_ADDR_44
    SHT3x::Address::Addr44;
#elif CONFIG_ER_SHT3X_ADDR_45
    SHT3x::Address::Addr45;
#endif

/// BLE UUID for environmental sensing service.
const BLEUUID kEnvironmentalSensingServiceUUID((uint16_t)0x181A);
/// BLE UUID for temperature characteristic.
const BLEUUID kTemperatureCharacteristicUUID((uint16_t)0x2A6E);
/// BLE UUID for date time characteristic.
const BLEUUID kDateTimeCharacteristicUUID((uint16_t)0x2A08);
/// BLE UUID for humidity characteristic.
const BLEUUID kHumidityCharacteristicUUID((uint16_t)0x2A6F);
/// BLE UUID for User Description characteristic.
const BLEUUID kUserDescriptionCharacteristicUUID((uint16_t)0x2901);
/// Service Data adv type.
const uint8_t kServiceDataAdvType = 0x16;

/// Flag that represents successfully connected to Wi-Fi AP.
const int kWiFiConnected = BIT0;
/// Flag that represents connection failure.
const int kWiFiFail = BIT1;

/***** Global Variables *****/
/// An event group handles Wi-Fi events.
EventGroupHandle_t g_wifi_event_group;
/// A number of retries for Wi-Fi connection.
int g_num_wifi_retries = 0;

/***** Functions *****/
/// Formats time_t into ISO8601 into string.
std::string TimeToString(const std::time_t t) {
  char s[20];
  std::tm *tm = std::gmtime(&t);
  std::strftime(s, 20, "%Y-%m-%d %H:%M:%S", tm);
  return std::string(s);
}

/// Handles WiFi event.
void HandleWiFiEvent(void *arg, esp_event_base_t event_base, int32_t event_id,
                     void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (g_num_wifi_retries < kWiFiMaxRetries) {
      esp_wifi_connect();
      ++g_num_wifi_retries;
      ESP_LOGI(TAG, "retrying to connect to the AP (attempt %d / %d)",
               g_num_wifi_retries, kWiFiMaxRetries);
    } else {
      xEventGroupSetBits(g_wifi_event_group, kWiFiFail);
    }
    ESP_LOGI(TAG, "connect to the AP failed");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
    g_num_wifi_retries = 0;
    xEventGroupSetBits(g_wifi_event_group, kWiFiConnected);
  }
}

void InitWiFi() {
  ESP_LOGI(TAG, "establishing WiFi connection to %s", kWiFiSSID);
  g_wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &HandleWiFiEvent, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &HandleWiFiEvent, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {};
  strncpy((char *)wifi_config.sta.ssid, kWiFiSSID, 32);
  strncpy((char *)wifi_config.sta.password, kWiFiPassword, 64);
  wifi_config.sta.bssid_set = 0;
  wifi_config.sta.sort_method = wifi_sort_method_t::WIFI_CONNECT_AP_BY_SECURITY;
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  wifi_config.sta.pmf_cfg = {/* capable */ true,
                             /* required */ false};
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  char cc[3];
  ESP_ERROR_CHECK(esp_wifi_get_country_code(cc));
  if (strcmp(cc, kWiFiCountryCode) != 0) {
    ESP_ERROR_CHECK(esp_wifi_set_country_code(kWiFiCountryCode, false));
  }
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(TAG, "wifi started");

  EventBits_t bits =
      xEventGroupWaitBits(g_wifi_event_group, kWiFiConnected | kWiFiFail,
                          pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & kWiFiConnected) {
    ESP_LOGI(TAG, "connected to AP %s", kWiFiSSID);
  } else if (bits & kWiFiFail) {
    ESP_LOGW(TAG, "failed to connect to AP %s", kWiFiSSID);
  } else {
    ESP_LOGE(TAG, "Unexpected event");
  }

  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
      IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
      WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
  vEventGroupDelete(g_wifi_event_group);
}

/// Handles time synced event.
void HandleSNTPSynced(struct timeval *tv) {
  auto dt_s = TimeToString(time(NULL));
  ESP_LOGI(TAG, "time synced: %s", dt_s.c_str());
}

/// Starts SNTP time synchronization.
void StartSNTP() {
  ESP_LOGI(TAG, "starting time synchronization with SNTP");
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, kSNTPServer);
  sntp_set_time_sync_notification_cb(HandleSNTPSynced);
  sntp_init();
}

/// Creates a new advertisement data.
/// \param[in] measure_result A result of measurement to be broadcasted.
/// \param[in] measured_at A timestamp of measurement.
BLEAdvertisementData MakeAdvertisementData(const SHT3x::Result &measure_result,
                                           std::time_t measured_at) {
  BLEAdvertisementData advData;
  uint16_t esUUID = kEnvironmentalSensingServiceUUID.getNative()->u16.value;
  uint64_t t_measure_64 = (uint64_t)measured_at;
  uint16_t raw_temp = measure_result.raw_temperature();
  uint16_t raw_humi = measure_result.raw_humidity();
  char payload[16] = {
      // 0: Length
      15,
      // 1: AdvType
      kServiceDataAdvType,
      // remaining elements are filled by following `memcpy`.
      // 2-3:   UUID
      // 4-11:  timestamp of measurement (8 bytes)
      // 12-13: temperature
      // 14-15: humidity
  };
  memcpy(payload + 2, &esUUID, 2);
  memcpy(payload + 4, &t_measure_64, 8);
  memcpy(payload + 12, &raw_temp, 2);
  memcpy(payload + 14, &raw_humi, 2);
  advData.addData(payload, 16);

  return advData;
}
}  // namespace

extern "C" void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  InitWiFi();
  StartSNTP();
  BLEDevice::init(kDeviceName);

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    // wait for syncing on power-on
    int sntp_retries = 0;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET &&
           ++sntp_retries < kSNTPMaxRetries) {
      ESP_LOGI(TAG, "waiting for time synchronization (%d / %d)", sntp_retries,
               kSNTPMaxRetries);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
  }

  I2C::I2C i2c(kI2CPort, kI2CSDANum, kI2CSCLNum, 100000L);

  ESP_LOGD(TAG, "measuring environment");
  SHT3x::SHT3x sht3x(i2c, kI2CAddress);
  auto res = sht3x.Measure(SHT3x::Repeatability::Medium);
  std::time_t t_measure = std::time(NULL);
  std::string t_measure_str = TimeToString(t_measure);
  ESP_LOGI(TAG, "%s: Temp=%.2f deg, Humi=%.2f %%", t_measure_str.c_str(),
           res.temperature(), res.humidity());

  auto advData = MakeAdvertisementData(res, t_measure);
  advData.setShortName(kShortDeviceName);

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->setAdvertisementData(advData);
  adv->setScanResponse(false);

  std::time_t t_adv_end = std::time(NULL) + kAdvertisingDuration;
  std::string t_adv_end_str = TimeToString(t_adv_end);

  ESP_LOGI(TAG, "advertise for %lld seconds (until %s)", kAdvertisingDuration,
           t_adv_end_str.c_str());
  adv->start();

  vTaskDelay((kAdvertisingDuration * kSecondsMS) / portTICK_PERIOD_MS);

  // stop WiFi/BT before entering deep sleep
  ESP_LOGD(TAG, "stop advertising");
  adv->stop();

  ESP_LOGD(TAG, "stop Wi-Fi");
  ESP_ERROR_CHECK(esp_wifi_stop());
  ESP_ERROR_CHECK(esp_wifi_deinit());

  std::time_t t_sleep_end = std::time(NULL) + kSleepDuration;
  std::string t_sleep_end_str = TimeToString(t_sleep_end);
  ESP_LOGI(TAG, "entering deep sleep for %lld seconds (until %s)",
           kSleepDuration, t_sleep_end_str.c_str());
  esp_deep_sleep(kSleepDuration * kSecondsUS);
}
