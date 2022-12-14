#include "sdkconfig.h"

#include <chrono>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "driver/i2c.h"
#include "nvs_flash.h"

#include "format.hpp"
#include "led.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"
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
  // wait for network
  float duty = 0.0f;
  while (!wifi_sta.is_connected()) {
    logger.info("waiting for wifi connection...");
    led.set_duty(led_channels[0].channel, duty);
    duty = duty == 0.0f ? 50.0f : 0.0f;
    std::this_thread::sleep_for(1s);
  }
  // use UDP multicast to listen for where the server is
  std::string multicast_group = "239.1.1.1";
  size_t port = 5000;
  espp::UdpSocket server_socket({.log_level=espp::Logger::Verbosity::WARN});
  auto server_task_config = espp::Task::Config{
    .name = "UdpServer",
    .callback = nullptr,
    .stack_size_bytes = 6 * 1024,
  };
  std::atomic<bool> found_receiver = false;
  std::string receiver_ip;
  auto server_config = espp::UdpSocket::ReceiveConfig{
    .port = port,
    .buffer_size = 1024,
    .is_multicast_endpoint = true,
    .multicast_group = multicast_group,
    .on_receive_callback = [&found_receiver, &receiver_ip](auto& data, auto& source) -> auto {
      receiver_ip = source.address;
      if (!found_receiver) {
        fmt::print("Found receiver server!\n");
      }
      found_receiver = true;
      return std::nullopt;
    }
  };
  server_socket.start_receiving(server_task_config, server_config);
  duty = 50.0f;
  while (!found_receiver) {
    logger.info("waiting for receiver to multicast their info over 239.1.1.1...");
    led.set_duty(led_channels[0].channel, duty);
    duty = duty == 50.0f ? 100.0f : 50.0f;
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
  std::atomic<int> num_frames_captured{0};
  QueueHandle_t transmit_queue = xQueueCreate(2, sizeof(Image));
  auto camera_task_fn = [&transmit_queue, &num_frames_captured, &logger](auto& m, auto& cv) {
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

    num_frames_captured += 1;

    // only copy / allocate if there is space in the queue, otherwise just
    // discard this image.
    auto num_spots = uxQueueSpacesAvailable(transmit_queue);
    if (num_spots > 0) {
      // copy the image data into a buffer and pass that buffer to the sending
      // task notify that image is ready.
      Image image;
      image.num_bytes = _jpg_buf_len + 8;
      image.data = (uint8_t*)heap_caps_malloc(image.num_bytes, MALLOC_CAP_SPIRAM);
      if (image.data != nullptr) {
        // header
        image.data[0] = 0xAA;
        image.data[1] = 0xBB;
        image.data[2] = 0xCC;
        image.data[3] = 0xDD;
        // length
        image.data[4] = (_jpg_buf_len >> 24) & 0xFF;
        image.data[5] = (_jpg_buf_len >> 16) & 0xFF;
        image.data[6] = (_jpg_buf_len >> 8) & 0xFF;
        image.data[7] = (_jpg_buf_len >> 0) & 0xFF;
        // data
        memcpy(&image.data[8], _jpg_buf, _jpg_buf_len);
        if (xQueueSend(transmit_queue, &image, portMAX_DELAY) != pdPASS) {
          // couldn't transmit the image, so we should free the memory here
          free(image.data);
        }
      } else {
        logger.error("Could not allocate for camera image!");
      }
    }

    esp_camera_fb_return(fb);
  };
  // make the tcp_client to transmit to the network
  std::atomic<int> num_frames_transmitted{0};
  std::atomic<float> transmission_elapsed{0};
  auto transmit_task_fn = [&found_receiver, &receiver_ip, &transmit_queue, &num_frames_transmitted, &transmission_elapsed, &logger](auto& m, auto& cv) {
    static size_t port = 8888;
    static Image image;
    static auto start = std::chrono::high_resolution_clock::now();
    static auto tcp_client = std::make_shared<espp::TcpSocket>(espp::TcpSocket::Config{});
    // wait on the queue until we have an image ready to transmit
    if (xQueueReceive(transmit_queue, &image, portMAX_DELAY) == pdPASS) {
      if (!found_receiver || !tcp_client->is_connected()) {
        logger.warn("Client is not connected...");
        if (!tcp_client->connect({.ip_address = receiver_ip, .port = port})) {
          logger.error("Client could not connect");
          tcp_client->reinit();
          found_receiver = false;
        } else {
          // reset our metrics now that we have a new connection
          start = std::chrono::high_resolution_clock::now();
          num_frames_transmitted = 0;
        }
      } else {
        logger.debug("Transmitting image {}B", image.num_bytes);
        if (!tcp_client->transmit(std::string_view{(const char*)image.data, image.num_bytes},
                                  { .wait_for_response = false })) {
          logger.error("couldn't transmit the data!");
        } else {
          num_frames_transmitted = num_frames_transmitted + 1;
        }
      }
      // now free the memory we allocated
      free(image.data);
    }
    auto end = std::chrono::high_resolution_clock::now();
    transmission_elapsed = std::chrono::duration<float>(end-start).count();
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
  auto start = std::chrono::high_resolution_clock::now();
  auto led_channel = led_channels[0].channel;
  while (true) {
    std::this_thread::sleep_for(1s);
    // pulse the LED very slowly...
    if (led.can_change(led_channel)) {
      // if the fade has finished, then pulse it again...
      auto maybe_duty = led.get_duty(led_channel);
      if (maybe_duty.has_value()) {
        auto duty = maybe_duty.value();
        if (duty < 50.0f) {
          led.set_fade_with_time(led_channel, 100.0f, 5000);
        } else {
          led.set_fade_with_time(led_channel, 0.0f, 5000);
        }
      }
    }
    // print out some stats (battery, framerate)
    auto end = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(end-start).count();
    logger.info("[{:.1f}] Battery voltage: {:.2f}", elapsed, battery.get_voltage());
    logger.info("[{:.1f}] Framerate (capture): {:.1f} FPS (average)", elapsed, num_frames_captured / elapsed);
    // this is an atomic float / shared variable, so let's access it once here
    float tx_elapsed = transmission_elapsed;
    if (tx_elapsed > 0) {
      logger.info("[{:.1f}] Framerate (transmit): {:.1f} FPS (average)", tx_elapsed, num_frames_transmitted / tx_elapsed);
    }
  }
}
