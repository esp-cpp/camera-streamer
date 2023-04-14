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

#include "OV2640.h"
#include "OV2640Streamer.h"
#include "CRtspSession.h"
#include "Cstreamer.h"

class Streamer : public CStreamer {
public:
  Streamer() : CStreamer(320, 240) {
    // TODO: do more here?
  }

  virtual void streamImage(uint32_t currMsec) override {
    // TODO: get image data from camera here
    BufPtr bytes = nullptr;
    uint32_t len = 0;
    streamFrame(bytes, len, currMsec);
  }
};

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
  OV2640 cam;
  CStreamer *streamer;
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
  cam.init(camera_config);
  streamer = new OV2640Streamer(cam);

  // err = esp_camera_init(&camera_config);
  // if (err != ESP_OK) {
  //   logger.error("Could not initialize camera: {} '{}'", err, esp_err_to_name(err));
  // }

  SOCKET MasterSocket;                                      // our masterSocket(socket that listens for RTSP client connections)
  sockaddr_in ServerAddr;                                   // server address parameters

  ServerAddr.sin_family      = AF_INET;
  ServerAddr.sin_addr.s_addr = INADDR_ANY;
  ServerAddr.sin_port        = htons(8554);                 // listen on RTSP port 8554
  MasterSocket               = socket(AF_INET,SOCK_STREAM,0);

  int enable = 1;
  if (setsockopt(MasterSocket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
    printf("setsockopt(SO_REUSEADDR) failed");
  }

  // bind our master socket to the RTSP port and listen for a client connection
  if (bind(MasterSocket,(sockaddr*)&ServerAddr,sizeof(ServerAddr)) != 0) {
    printf("error can't bind port errno=%d\n", errno);
  }
  if (listen(MasterSocket,5) != 0) {
    printf("Couldn't listen!\n");
  }

  auto camera_task_fn = [&streamer](auto &m, auto& cv) -> bool {
    static float seconds_per_frame = 0.1f;
    static auto start = std::chrono::high_resolution_clock::now();
    static auto last_image_time = start;
    streamer->handleRequests(0); // don't use a timeout
    if (streamer->anySessions()) {
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds_since_last = std::chrono::duration<float>(now-last_image_time).count();
      auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count();
      if (seconds_since_last > seconds_per_frame) {
        last_image_time = now;
        fmt::print("Streaming image...\n");
        streamer->streamImage(millis);
      }
    }
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 1ms);
    return false;
  };

  auto server_task_fn = [&streamer, &MasterSocket](auto &m, auto& cv) -> bool {
    fmt::print("Server accepting client...\n");
    sockaddr_in ClientAddr;                                   // address parameters of a new RTSP client
    socklen_t ClientAddrLen = sizeof(ClientAddr);
    auto ClientSocket = accept(MasterSocket,(struct sockaddr*)&ClientAddr,&ClientAddrLen);
    printf("Client connected. Client address: %s\r\n",inet_ntoa(ClientAddr.sin_addr));
    // create a task for the client
    // CRtspSession rtsp(ClientSocket, streamer);
    streamer->addSession(ClientSocket);

    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 30ms);
    return false;
  };
  auto server_task = espp::Task::make_unique(espp::Task::Config{
      .name = "Server Task",
      .callback = server_task_fn,
      .stack_size_bytes = 10*1024,
      .priority = 10
    });

  auto camera_task = espp::Task::make_unique(espp::Task::Config{
      .name = "Camera Task",
      .callback = camera_task_fn,
      .stack_size_bytes = 10*1024,
      .priority = 10
    });

  logger.info("Starting camera and transmit tasks");
  // server_task->start();
  // camera_task->start();

  {
    sockaddr_in ClientAddr;                                   // address parameters of a new RTSP client
    socklen_t ClientAddrLen = sizeof(ClientAddr);
    SOCKET ClientSocket;                                      // RTSP socket to handle an client
    while (true) {
      ClientSocket = accept(MasterSocket,(struct sockaddr*)&ClientAddr,&ClientAddrLen);
      printf("Client connected. Client address: %s\r\n",inet_ntoa(ClientAddr.sin_addr));

      CRtspSession rtsp(ClientSocket, streamer);     // our threads RTSP session and state

      while (!rtsp.m_stopped) {
        uint32_t timeout_ms = 400;
        if(!rtsp.handleRequests(timeout_ms)) {
          struct timeval now;
          gettimeofday(&now, NULL); // crufty msecish timer
          uint32_t msec = now.tv_sec * 1000 + now.tv_usec / 1000;
          // rtsp.broadcastCurrentFrame(msec);
          streamer->streamImage(msec);
        }
      }
    }
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
  // logger.info("Creating camera task");
  // std::atomic<int> num_frames_captured{0};
  // QueueHandle_t transmit_queue = xQueueCreate(2, sizeof(Image));
  // auto camera_task_fn = [&transmit_queue, &num_frames_captured, &logger](auto& m, auto& cv) {
  //   // take image
  //   static camera_fb_t * fb = NULL;
  //   static size_t _jpg_buf_len;
  //   static uint8_t * _jpg_buf;

  //   fb = esp_camera_fb_get();
  //   if (!fb) {
  //     logger.error("Camera capture failed");
  //     return;
  //   }
  //   uint32_t sig = *((uint32_t *)&fb->buf[fb->len - 4]);

  //   _jpg_buf_len = fb->len;
  //   _jpg_buf = fb->buf;

  //   if (!(_jpg_buf[_jpg_buf_len - 1] != 0xd9 || _jpg_buf[_jpg_buf_len - 2] != 0xd9)) {
  //     esp_camera_fb_return(fb);
  //     return;
  //   }

  //   num_frames_captured += 1;

  //   // only copy / allocate if there is space in the queue, otherwise just
  //   // discard this image.
  //   auto num_spots = uxQueueSpacesAvailable(transmit_queue);
  //   if (num_spots > 0) {
  //     // copy the image data into a buffer and pass that buffer to the sending
  //     // task notify that image is ready.
  //     Image image;
  //     image.num_bytes = _jpg_buf_len + 8;
  //     image.data = (uint8_t*)heap_caps_malloc(image.num_bytes, MALLOC_CAP_SPIRAM);
  //     if (image.data != nullptr) {
  //       // header
  //       image.data[0] = 0xAA;
  //       image.data[1] = 0xBB;
  //       image.data[2] = 0xCC;
  //       image.data[3] = 0xDD;
  //       // length
  //       image.data[4] = (_jpg_buf_len >> 24) & 0xFF;
  //       image.data[5] = (_jpg_buf_len >> 16) & 0xFF;
  //       image.data[6] = (_jpg_buf_len >> 8) & 0xFF;
  //       image.data[7] = (_jpg_buf_len >> 0) & 0xFF;
  //       // data
  //       memcpy(&image.data[8], _jpg_buf, _jpg_buf_len);
  //       if (xQueueSend(transmit_queue, &image, portMAX_DELAY) != pdPASS) {
  //         // couldn't transmit the image, so we should free the memory here
  //         free(image.data);
  //       }
  //     } else {
  //       logger.error("Could not allocate for camera image!");
  //     }
  //   }

  //   esp_camera_fb_return(fb);
  // };
  // // make the tcp_client to transmit to the network
  // std::atomic<int> num_frames_transmitted{0};
  // std::atomic<float> transmission_elapsed{0};
  // auto camera_task = espp::Task::make_unique({
  //     .name = "Camera Task",
  //     .callback = camera_task_fn,
  //     .priority = 10
  //   });
  // logger.info("Starting camera and transmit tasks");
  // camera_task->start();

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
    // logger.info("[{:.1f}] Battery voltage: {:.2f}", elapsed, battery.get_voltage());
    // logger.info("[{:.1f}] Framerate (capture): {:.1f} FPS (average)", elapsed, num_frames_captured / elapsed);
    // // this is an atomic float / shared variable, so let's access it once here
    // float tx_elapsed = transmission_elapsed;
    // if (tx_elapsed > 0) {
    //   logger.info("[{:.1f}] Framerate (transmit): {:.1f} FPS (average)", tx_elapsed, num_frames_transmitted / tx_elapsed);
    // }
  }

  closesocket(MasterSocket);
}
