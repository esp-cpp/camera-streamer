#include "sdkconfig.h"

#include <chrono>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_heap_caps.h>
#include <mdns.h>

#include "cli.hpp"
#include "esp32-timer-cam.hpp"
#include "nvs.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"
#include "wifi_sta.hpp"
#include "wifi_sta_menu.hpp"

#include "esp_camera.h"

#include "rtsp_server.hpp"

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "Camera Streamer", .level = espp::Logger::Verbosity::INFO});

std::recursive_mutex server_mutex;
std::unique_ptr<espp::Task> camera_task;
std::shared_ptr<espp::RtspServer> rtsp_server;

esp_err_t initialize_camera(void);
void start_rtsp_server(std::string_view server_address, int server_port);
bool camera_task_fn(const std::mutex &m, const std::condition_variable &cv);

extern "C" void app_main(void) {
  esp_err_t err;
  logger.info("Bootup");

#if CONFIG_ESP32_WIFI_NVS_ENABLED
  std::error_code ec;
  // initialize NVS, needed for WiFi
  espp::Nvs nvs;
  nvs.init(ec);
#endif

  auto &timer_cam = espp::EspTimerCam::get();

  // initialize RTC
  if (!timer_cam.initialize_rtc()) {
    logger.error("Could not initialize RTC");
    return;
  }

  // initialize LED
  static constexpr float disconnected_led_breathing_period = 1.0f;
  static constexpr float connected_led_breathing_period = 3.5f;
  if (!timer_cam.initialize_led(disconnected_led_breathing_period)) {
    logger.error("Could not initialize LED");
    return;
  }
  timer_cam.start_led_breathing();

  // initialize camera
  logger.info("Initializing camera");
  err = initialize_camera();
  if (err != ESP_OK) {
    logger.error("Could not initialize camera: {} '{}'", err, esp_err_to_name(err));
  }

  // initialize WiFi
  logger.info("Initializing WiFi");
  espp::WifiSta wifi_sta(
      {.ssid = CONFIG_ESP_WIFI_SSID,
       .password = CONFIG_ESP_WIFI_PASSWORD,
       .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
       .on_connected =
           []() {
             static auto &timer_cam = espp::EspTimerCam::get();
             timer_cam.set_led_breathing_period(connected_led_breathing_period);
           },
       .on_disconnected =
           []() {
             static auto &timer_cam = espp::EspTimerCam::get();
             timer_cam.set_led_breathing_period(disconnected_led_breathing_period);
             std::lock_guard<std::recursive_mutex> lock(server_mutex);
             logger.info("Stopping camera task");
             camera_task.reset();
             logger.info("Stopping RTSP server");
             rtsp_server.reset();
             logger.info("Deiniting MDNS");
             mdns_free();
           },
       .on_got_ip =
           [](ip_event_got_ip_t *eventdata) {
             auto server_address = fmt::format("{}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
             logger.info("got IP: {}", server_address);
             // create the camera and rtsp server, and the cv/m
             // they'll use to communicate
             std::lock_guard<std::recursive_mutex> lock(server_mutex);
             start_rtsp_server(server_address, CONFIG_RTSP_SERVER_PORT);
             // initialize the camera
             logger.info("Creating camera task");
             camera_task = espp::Task::make_unique(
                 espp::Task::Config{.callback = camera_task_fn,
                                    .task_config = {.name = "Camera Task", .priority = 10}});
             camera_task->start();
           }});

  espp::WifiStaMenu sta_menu(wifi_sta);
  auto root_menu = sta_menu.get();
  root_menu->Insert(
      "memory",
      [](std::ostream &out) {
        out << "Minimum free memory: " << heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT)
            << std::endl;
      },
      "Display minimum free memory.");
  root_menu->Insert(
      "battery",
      [](std::ostream &out) {
        static auto &timer_cam = espp::EspTimerCam::get();
        out << fmt::format("Battery voltage: {:.2f}\n", timer_cam.get_battery_voltage());
      },
      "Display the current battery voltage.");

  cli::Cli cli(std::move(root_menu));
  cli::SetColor();
  cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

  espp::Cli input(cli);
  input.SetInputHistorySize(10);
  input.Start();
}

esp_err_t initialize_camera(void) {
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

  auto &timer_cam = espp::EspTimerCam::get();
  static camera_config_t camera_config = {
      .pin_pwdn = -1,
      .pin_reset = timer_cam.get_camera_reset_pin(),
      .pin_xclk = timer_cam.get_camera_xclk_pin(),
      .pin_sccb_sda = timer_cam.get_camera_sda_pin(),
      .pin_sccb_scl = timer_cam.get_camera_scl_pin(),

      .pin_d7 = timer_cam.get_camera_d7_pin(),
      .pin_d6 = timer_cam.get_camera_d6_pin(),
      .pin_d5 = timer_cam.get_camera_d5_pin(),
      .pin_d4 = timer_cam.get_camera_d4_pin(),
      .pin_d3 = timer_cam.get_camera_d3_pin(),
      .pin_d2 = timer_cam.get_camera_d2_pin(),
      .pin_d1 = timer_cam.get_camera_d1_pin(),
      .pin_d0 = timer_cam.get_camera_d0_pin(),
      .pin_vsync = timer_cam.get_camera_vsync_pin(),
      .pin_href = timer_cam.get_camera_href_pin(),
      .pin_pclk = timer_cam.get_camera_pclk_pin(),

      // EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
      .xclk_freq_hz = timer_cam.get_camera_xclk_freq_hz(),
      .ledc_timer = LEDC_TIMER_0,
      .ledc_channel = LEDC_CHANNEL_0,

      .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
      .frame_size = FRAMESIZE_QVGA,   // QVGA-UXGA, For ESP32, do not use sizes above QVGA when not
                                      // JPEG. The performance of the ESP32-S series has improved a
                                      // lot, but JPEG mode always gives better frame rates.

      .jpeg_quality = 15, // 0-63, for OV series camera sensors, lower number means higher quality
      .fb_count = 2, // When jpeg mode is used, if fb_count more than one, the driver will work in
                     // continuous mode.
      .grab_mode =
          CAMERA_GRAB_LATEST // CAMERA_GRAB_WHEN_EMPTY // . Sets when buffers should be filled
  };
  auto err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    logger.error("Could not initialize camera: {} '{}'", err, esp_err_to_name(err));
    return err;
  }
  // set the mirror and flip - specific to the ESP32-TimerCam!
  logger.info("Enabling camera vflip");
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, true);
  s->set_hmirror(s, false);
  return ESP_OK;
}

void start_rtsp_server(std::string_view server_address, int server_port) {
  logger.info("Creating RTSP server at {}:{}", server_address, server_port);
  std::lock_guard<std::recursive_mutex> lock(server_mutex);
  rtsp_server = std::make_shared<espp::RtspServer>(
      espp::RtspServer::Config{.server_address = std::string(server_address),
                               .port = server_port,
                               .path = "mjpeg/1",
                               .log_level = espp::Logger::Verbosity::WARN});
  rtsp_server->set_session_log_level(espp::Logger::Verbosity::WARN);
  rtsp_server->start();

  // initialize mDNS
  logger.info("Initializing mDNS");
  esp_err_t err = mdns_init();
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
}

bool camera_task_fn(const std::mutex &m, const std::condition_variable &cv) {
  // take image
  static camera_fb_t *fb = NULL;
  static size_t _jpg_buf_len;
  static uint8_t *_jpg_buf;

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

  std::span<const uint8_t> jpg_buf(_jpg_buf, _jpg_buf_len);
  espp::JpegFrame image(jpg_buf);
  std::lock_guard<std::recursive_mutex> lock(server_mutex);
  rtsp_server->send_frame(image);

  esp_camera_fb_return(fb);
  return false;
};
