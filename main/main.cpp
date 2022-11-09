#include "sdkconfig.h"

#include <chrono>

#include "format.hpp"
#include "task.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Camera Streamer", .level = espp::Logger::Verbosity::WARN});
  logger.info("Bootup");
  // TODO: initialize file system
  logger.info("Initializing littlefs");
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
          logger.info("got IP: {}.{}.{}.{}\n", IP2STR(&eventdata->ip_info.ip));
        }
        });
  // TODO: initialize camera
  logger.info("Initializing camera");
  // TODO: initialize RTC
  logger.info("Initializing RTC");
  // TODO: initialize battery
  logger.info("Initializing battery measurement");
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
  auto transmit_task_fn = [&image_ready_cv, &image_ready_cv_m](auto& m, auto& cv) {
    {
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
