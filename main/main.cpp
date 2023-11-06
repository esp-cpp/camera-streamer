#include "sdkconfig.h"

#include <chrono>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_heap_caps.h>
#include <mdns.h>
#include <nvs_flash.h>

#include "format.hpp"
#include "gaussian.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"
#include "wifi_sta.hpp"

#include "esp_camera.h"
#include "battery.hpp"
#include "bm8563.hpp"

#include "rtsp_server.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  esp_err_t err;
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

  // initialize LED
  logger.info("Initializing LED");
  std::vector<espp::Led::ChannelConfig> led_channels{
    {
      .gpio = 2,
      .channel = LEDC_CHANNEL_5,
      .timer = LEDC_TIMER_2,
    }
  };
  espp::Led led(espp::Led::Config{
      .timer = LEDC_TIMER_2,
      .frequency_hz = 5000,
      .channels = led_channels,
      .duty_resolution = LEDC_TIMER_10_BIT,
    });

  // initialize WiFi
  logger.info("Initializing WiFi");
  std::string server_address;
  espp::WifiSta wifi_sta({
      .ssid = CONFIG_ESP_WIFI_SSID,
        .password = CONFIG_ESP_WIFI_PASSWORD,
        .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
        .on_connected = nullptr,
        .on_disconnected = nullptr,
        .on_got_ip = [&logger, &server_address](ip_event_got_ip_t* eventdata) {
          server_address = fmt::format("{}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
          logger.info("got IP: {}", server_address);
        }
        });
  // wait for network
  float duty = 0.0f;
  while (!wifi_sta.is_connected()) {
    logger.info("waiting for wifi connection...");
    led.set_duty(led_channels[0].channel, duty);
    duty = duty == 0.0f ? 50.0f : 0.0f;
    std::this_thread::sleep_for(1s);
  }

  // initialize camera
  /**
   * @note display sizes supported:
   * *  QVGA:  320x240
   * *  WQVGA: 400x240
   * *  HVGA:  480x320
   * *  VGA:   640x480
   * *  WVGA:  768x480
   * *  FWVGA: 854x480
   * *  SVGA:  800x600
   * *  DVGA:  960x640
   * *  WSVGA: 1024x600
   * *  XGA:   1024x768
   * *  WXGA:  1280x800
   * *  WSXGA: 1440x900
   * *  SXGA:  1280x1024
   * *  UXGA:  1600x1200
   */

  logger.info("Initializing camera");
  static camera_config_t camera_config = {
    .pin_pwdn  = -1,
    .pin_reset = 15,
    .pin_xclk = 27,
    .pin_sccb_sda = 25,
    .pin_sccb_scl = 23,

    .pin_d7 = 19,
    .pin_d6 = 36,
    .pin_d5 = 18,
    .pin_d4 = 39,
    .pin_d3 = 5,
    .pin_d2 = 34,
    .pin_d1 = 35,
    .pin_d0 = 32,
    .pin_vsync = 22,
    .pin_href = 26,
    .pin_pclk = 21,

    .xclk_freq_hz = 10000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,// QVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 15, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_LATEST // CAMERA_GRAB_WHEN_EMPTY // . Sets when buffers should be filled
  };
  err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    logger.error("Could not initialize camera: {} '{}'", err, esp_err_to_name(err));
  }

  float breathing_period = 3.5f; // seconds
  espp::Gaussian gaussian({.gamma = 0.1f, .alpha = 1.0f, .beta = 0.5f});
  auto breathe = [&gaussian, &breathing_period]() -> float {
    static auto breathing_start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<float>(now - breathing_start).count();
    float t = std::fmod(elapsed, breathing_period) / breathing_period;
    return gaussian(t);
  };
  auto led_callback = [&breathe, &led, &led_channels](auto &m, auto &cv) -> bool {
    led.set_duty(led_channels[0].channel, 100.0f * breathe());
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 10ms);
    return false;
  };
  auto led_task = espp::Task::make_unique({.name = "breathe", .callback = led_callback});
  led_task->start();

  // initialize the i2c bus (for RTC)
  logger.info("Initializing I2C");
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_12,
        .scl_io_num = GPIO_NUM_14,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    });

  // initialize RTC
  logger.info("Initializing RTC");
  espp::Bm8563 bm8563({
      .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      .read = std::bind(&espp::I2c::read_at_register, &i2c, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .log_level = espp::Logger::Verbosity::WARN
    });

  // initialize ADC
  logger.info("Initializing Oneshot ADC");
  std::vector<espp::AdcConfig> channels{
    {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_2, // this is the ADC for GPIO 38
      .attenuation = ADC_ATTEN_DB_11
    }
  };

  // we use oneshot adc here to that we could add other channels if need be for
  // other components, but it has no in-built filtering. NOTE: for some reason,
  // I cannot use Continuous ADC in combination with esp32-camera...
  espp::OneshotAdc adc({
      .unit = ADC_UNIT_1,
      .channels = channels,
      .log_level = espp::Logger::Verbosity::WARN
    });
  auto read_battery_voltage = [&adc, &channels]() -> float {
    auto maybe_mv = adc.read_mv(channels[0]);
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

  // create the camera and rtsp server, and the cv/m they'll use to
  // communicate
  int server_port = CONFIG_RTSP_SERVER_PORT;
  logger.info("Creating RTSP server at {}:{}", server_address, server_port);
  espp::RtspServer rtsp_server({
      .server_address = server_address,
      .port = server_port,
      .path = "mjpeg/1",
      .log_level = espp::Logger::Verbosity::WARN
    });
  rtsp_server.set_session_log_level(espp::Logger::Verbosity::WARN);
  rtsp_server.start();

  // initialize mDNS
  logger.info("Initializing mDNS");
  err = mdns_init();
  if (err != ESP_OK) {
    logger.error("Could not initialize mDNS: {}", err);
    return;
  }

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  std::string hostname = fmt::format("camera-streamer-{:x}{:x}{:x}", mac[3], mac[4], mac[5]);
  err = mdns_hostname_set(hostname.c_str());
  if (err != ESP_OK) {
    logger.error("Could not set mDNS hostname: {}", err);
    return;
  }
  logger.info("mDNS hostname set to '{}'", hostname);
  err = mdns_instance_name_set("Camera Streamer");
  if (err != ESP_OK) {
    logger.error("Could not set mDNS instance name: {}", err);
    return;
  }
  err = mdns_service_add("RTSP Server", "_rtsp", "_tcp", server_port, NULL, 0);
  if (err != ESP_OK) {
    logger.error("Could not add mDNS service: {}", err);
    return;
  }
  logger.info("mDNS initialized");

  // initialize the camera
  logger.info("Creating camera task");
  auto camera_task_fn = [&rtsp_server, &logger](auto& m, auto& cv) -> bool {
    // take image
    static camera_fb_t * fb = NULL;
    static size_t _jpg_buf_len;
    static uint8_t * _jpg_buf;

    fb = esp_camera_fb_get();
    if (!fb) {
      logger.error("Camera capture failed");
      return false;
    }

    _jpg_buf_len = fb->len;
    _jpg_buf = fb->buf;

    if (!(_jpg_buf[_jpg_buf_len - 1] != 0xd9 || _jpg_buf[_jpg_buf_len - 2] != 0xd9)) {
      esp_camera_fb_return(fb);
      return false;
    }

    espp::JpegFrame image(reinterpret_cast<const char*>(_jpg_buf), _jpg_buf_len);
    rtsp_server.send_frame(image);

    esp_camera_fb_return(fb);
    return false;
  };

  auto camera_task = espp::Task::make_unique({
      .name = "Camera Task",
      .callback = camera_task_fn,
      .priority = 10
    });
  camera_task->start();

  auto start = std::chrono::high_resolution_clock::now();
  while (true) {
    std::this_thread::sleep_for(1s);
    // print out some stats (battery, framerate)
    auto end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(end-start).count();
    fmt::print("\x1B[1A"); // go up a line
    fmt::print("\x1B[2K\r"); // erase the line
    logger.info("[{:.1f}] Minimum free memory: {}, Battery voltage: {:.2f}",
                elapsed, heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT), battery.get_voltage());
  }
}
