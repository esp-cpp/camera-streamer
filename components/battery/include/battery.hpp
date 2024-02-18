#pragma once

#include "driver/gpio.h"

#include "logger.hpp"

class Battery {
public:
  typedef std::function<float()> read_fn;

  static constexpr float SCALE = 0.661f;

  struct Config {
    read_fn read;
    int hold_gpio{-1};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  Battery(const Config &config)
      : read_(config.read)
      , hold_gpio_(config.hold_gpio)
      , logger_({.tag = "Battery", .level = config.log_level}) {
    init();
  }

  void hold_output() {
    if (hold_gpio_ == -1) {
      logger_.error("Hold GPIO not configured, cannot hold output.");
      return;
    }
    logger_.info("holding output");
    gpio_set_level((gpio_num_t)hold_gpio_, 1);
  }

  void disable_output() {
    if (hold_gpio_ == -1) {
      logger_.error("Hold GPIO not configured, cannot disable output.");
      return;
    }
    logger_.info("disabling output");
    gpio_set_level((gpio_num_t)hold_gpio_, 0);
  }

  float get_voltage() {
    auto voltage = read_() / SCALE;
    logger_.info("Got voltage {}", voltage);
    return voltage;
  }

protected:
  void init() {
    if (hold_gpio_ == -1) {
      logger_.warn("Hold GPIO not configured, will not be able to hold/disable output.");
      return;
    }
    logger_.info("Initializing hold GPIO {}", hold_gpio_);
    uint64_t pin_mask = (1 << hold_gpio_);
    gpio_config_t io_config = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));
    hold_output();
  }

  read_fn read_;
  int hold_gpio_;
  espp::Logger logger_;
};
