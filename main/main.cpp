#include "sdkconfig.h"

#include <chrono>

#include "driver/i2c.h"
#include "nvs_flash.h"

#include "continuous_adc.hpp"
#include "format.hpp"
#include "task.hpp"
#include "wifi_sta.hpp"

#include "battery.hpp"
#include "bm8563.hpp"
#include "fs_init.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Camera Streamer", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");
  // initialize NVS, needed for WiFi
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    logger.warn("Erasing NVS flash...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  // initialize file system
  logger.info("Initializing littlefs");
  fs_init();
  // TODO: initialize LED
  logger.info("Initializing LED");
  // initialize WiFi
  logger.info("Initializing WiFi");
  espp::WifiSta wifi_sta({
      .ssid = CONFIG_ESP_WIFI_SSID,
        .password = CONFIG_ESP_WIFI_PASSWORD,
        .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
        .on_connected = nullptr,
        .on_disconnected = nullptr,
        .on_got_ip = [&logger](ip_event_got_ip_t* eventdata) {
          logger.info("got IP: {}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
        }
        });
  // TODO: initialize camera
  logger.info("Initializing camera");
  // initialize the i2c bus (for RTC)
  logger.info("Initializing I2C");
  i2c_config_t i2c_cfg;
  memset(&i2c_cfg, 0, sizeof(i2c_cfg));
  i2c_cfg.sda_io_num = GPIO_NUM_12;
  i2c_cfg.scl_io_num = GPIO_NUM_14;
  i2c_cfg.mode = I2C_MODE_MASTER;
  i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.master.clk_speed = (400*1000);
  auto err = i2c_param_config(I2C_NUM_0, &i2c_cfg);
  if (err != ESP_OK)
    logger.error("config i2c failed {} '{}'", err, esp_err_to_name(err));
  err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER,  0, 0, 0);
  if (err != ESP_OK)
    logger.error("install i2c driver failed {} '{}'", err, esp_err_to_name(err));
  static const int I2C_TIMEOUT_MS = 10;
  auto bm8563_write = [](uint8_t *write_data, size_t write_len) {
    i2c_master_write_to_device(I2C_NUM_0,
                               Bm8563::ADDRESS,
                               write_data,
                               write_len,
                               I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  };
  auto bm8563_read = [](uint8_t reg_addr, uint8_t *read_data, size_t read_len) {
    i2c_master_write_read_device(I2C_NUM_0,
                                 Bm8563::ADDRESS,
                                 &reg_addr,
                                 1, // size of addr
                                 read_data,
                                 read_len,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);

  };
  // initialize RTC
  logger.info("Initializing RTC");
  Bm8563 bm8563(Bm8563::Config{
      .write = bm8563_write,
      .read = bm8563_read,
      .log_level = espp::Logger::Verbosity::WARN
    });
  // initialize ADC
  logger.info("Initializing Continuous ADC");
  std::vector<espp::AdcConfig> channels{
    {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_2, // this is the ADC for GPIO 38
      .attenuation = ADC_ATTEN_DB_11
    }
  };
  // we use continuous adc here because 1) it already creates a task for
  // sampling / filtering, and 2) it means that if we wanted to add other ADCs
  // (for other components / uses) we can just update it's config.
  espp::ContinuousAdc adc({
      .sample_rate_hz = 20*1000, // 20KHz is minimum sample rate for ESP32 continuous ADC
      .channels = channels,
      .convert_mode = ADC_CONV_SINGLE_UNIT_1, // ESP32 only supports SINGLE_UNIT_1
      .window_size_bytes = 1024,
      .log_level = espp::Logger::Verbosity::WARN
    });
  auto read_battery_voltage = [&adc, &channels]() -> float {
    auto channel = channels[0].channel;
    auto maybe_mv = adc.get_mv(channel);
    float measurement = 0;
    if (maybe_mv.has_value()) {
      auto mv = maybe_mv.value();
      measurement = mv;
    }
    return measurement;
  };
  // initialize battery
  logger.info("Initializing battery measurement");
  Battery battery(Battery::Config{
      .read = read_battery_voltage,
      // NOTE: cannot initialize battery hold pin right now, if I do then the
      // board ... stops running; therefore I've configured the battery to not
      // use the hold gpio
      .hold_gpio = -1, // 33,
      .log_level = espp::Logger::Verbosity::WARN
    });
  // create the camera and transmit tasks, and the cv/m they'll use to
  // communicate
  logger.info("Creating camera and transmit tasks");
  std::mutex image_ready_cv_m;
  std::condition_variable image_ready_cv;
  auto camera_task_fn = [&image_ready_cv](auto& m, auto& cv) {
    auto start = std::chrono::high_resolution_clock::now();
    // TODO: take image
    // notify that image is ready
    image_ready_cv.notify_all();
    // try to get ~10 FPS
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, start + 100ms);
    }
  };
  auto transmit_task_fn = [&image_ready_cv, &image_ready_cv_m, &wifi_sta, &logger](auto& m, auto& cv) {
    {
      // check the image_ready_cv to see if there's an image to transmit
      std::unique_lock<std::mutex> lk(image_ready_cv_m);
      auto cv_retval = image_ready_cv.wait_for(lk, 10ms);
      if (cv_retval == std::cv_status::no_timeout) {
        // we got an image, now transmit it over the network! (but let's break
        // out of this scope to release the mutex for the cv)
      } else {
        // we timed out, there is no image ready, so simply return from this
        // function.
        return;
      }
    }
    // first make sure we're actually still connected to WiFi
    if (!wifi_sta.is_connected()) {
      logger.error("Not connected to WiFi, cannot send image!\n"
                   "Waiting 1 second for WiFi to reconnect before trying again.");
      // sleep here since wifi can take some time to reconnect and we don't want
      // a _ton_ of logs...
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 1s);
      }
      // now return to try again.
      return;
    }
    // TODO: here we'll actually get the image data, serialize it, and send it over
    // WiFi.
  };
  auto camera_task = espp::Task::make_unique({
      .name = "Camera Task",
      .callback = camera_task_fn,
      .priority = 10
    });
  auto transmit_task = espp::Task::make_unique({
      .name = "Transmit Task",
      .callback = transmit_task_fn,
      .priority = 5
    });
  logger.info("Starting camera and transmit tasks");
  camera_task->start();
  transmit_task->start();
  while (true) {
    // monitor inputs or some other (lower priority) thing here...
    std::this_thread::sleep_for(100ms);
  }
}
