#include "sdkconfig.h"

#include <chrono>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "nvs_flash.h"

#include "format.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"
#include "udp_socket.hpp"
#include "tcp_socket.hpp"
#include "wifi_sta.hpp"

#include "esp_camera.h"
#include "battery.hpp"
#include "bm8563.hpp"
#include "fs_init.hpp"

using namespace std::chrono_literals;

struct Image {
  uint8_t *data;
  uint32_t num_bytes;
};

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
  // initialize camera
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

    .xclk_freq_hz = 20000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,//QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 15, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 3, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY//CAMERA_GRAB_LATEST. Sets when buffers should be filled
  };
  err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    logger.error("Could not initialize camera: {} '{}'", err, esp_err_to_name(err));
  }
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
  err = i2c_param_config(I2C_NUM_0, &i2c_cfg);
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
    auto channel = channels[0].channel;
    auto maybe_mv = adc.read_mv(channel);
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
  QueueHandle_t transmit_queue = xQueueCreate(10, sizeof(Image));
  auto camera_task_fn = [&transmit_queue, &logger](auto& m, auto& cv) {
    auto start = std::chrono::high_resolution_clock::now();
    // take image
    static camera_fb_t * fb = NULL;
    static size_t _jpg_buf_len;
    static uint8_t * _jpg_buf;

    fb = esp_camera_fb_get();
    if (!fb) {
      logger.error("Camera capture failed");
      return;
    }
    uint32_t sig = *((uint32_t *)&fb->buf[fb->len - 4]);

    _jpg_buf_len = fb->len;
    _jpg_buf = fb->buf;

    if (!(_jpg_buf[_jpg_buf_len - 1] != 0xd9 || _jpg_buf[_jpg_buf_len - 2] != 0xd9)) {
      esp_camera_fb_return(fb);
      return;
    }

    // copy the image data into a buffer and pass that buffer to the sending
    // task notify that image is ready.
    Image image;
    image.num_bytes = _jpg_buf_len;
    image.data = (uint8_t*)malloc(image.num_bytes);
    memcpy(image.data, _jpg_buf, image.num_bytes);
    xQueueSend(transmit_queue, &image, portMAX_DELAY);

    esp_camera_fb_return(fb);

    auto end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(end-start).count();
    logger.info("MJPG: {}KB {}s ({:.1f}fps), 0x{:02x}",
             (uint32_t)(_jpg_buf_len/1024), elapsed, 1.0f / elapsed, sig);

    // try to get ~10 FPS
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, start + 100ms);
    }
  };
  // make the udp_client to multicast to the network
  espp::UdpSocket udp_client({});
  auto transmit_task_fn = [&udp_client, &transmit_queue, &wifi_sta, &logger](auto& m, auto& cv) {
    static Image image;
    // wait on the queue until we have an image ready to transmit
    xQueueReceive(transmit_queue, &image, portMAX_DELAY);
    // copy the image buffer into a std::shared_ptr so that it's freed when this
    // function returns...
    std::unique_ptr<uint8_t> buffer_ptr(image.data);
    // first make sure we're actually still connected to WiFi
    if (!wifi_sta.is_connected()) {
      logger.error("Not connected to WiFi, cannot send image!");
      // now return to try again.
      return;
    }
    // get the image data, serialize it, and send it over WiFi.
    std::vector<uint8_t> data(image.data, image.data + image.num_bytes);
    udp_client.send(data, {
        .ip_address = "239.1.1.1",
        .port = 5000,
        .is_multicast_endpoint = true,
        .wait_for_response = false
      });
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
    logger.info("Battery voltage: {:.2f}", battery.get_voltage());
    // monitor inputs or some other (lower priority) thing here...
    std::this_thread::sleep_for(1s);
  }
}
