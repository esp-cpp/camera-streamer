#pragma once

#include <functional>

#include "logger.hpp"

class Bm8563 {
public:
  static constexpr uint8_t ADDRESS = (0x51);

  typedef std::function<void(uint8_t*, size_t)> write_fn;
  typedef std::function<void(uint8_t, uint8_t*, size_t)> read_fn;

  struct Date {
    uint16_t year;
    uint8_t month;
    uint8_t weekday;
    uint8_t day;
  };
  struct Time {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
  };
  struct DateTime {
    Date date;
    Time time;
  };

  struct Config {
    write_fn write;
    read_fn read;
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Log verbosity for the input driver.  */
  };

  Bm8563(const Config& config)
    : write_(config.write),
      read_(config.read),
      logger_({.tag = "Bm8563", .level = config.log_level}) {
    init();
  }

  static uint8_t bcd2byte(uint8_t value) {
    return (value >> 4) * 10 + (value & 0x0f);
  }

  static uint8_t byte2bcd(uint8_t value) {
    return ((value / 10) << 4) + value % 10;
  }

  DateTime get_date_time() {
    DateTime dt;
    dt.time = get_time();
    dt.date = get_date();
    return dt;
  }

  void set_date_time(DateTime &dt) {
    set_date(dt.date);
    set_time(dt.time);
  }

  Date get_date() {
    logger_.info("getting date");
    uint8_t data[4];
    read_((uint8_t)(Registers::DATE), data, 4);
    Date d;
    int base_year = (data[2] & CENTURY_BIT) ? 1900 : 2000;
    d.year = base_year + bcd2byte(data[3] & 0xff);
    d.month = bcd2byte(data[2] & 0x1f);
    d.weekday = bcd2byte(data[1] & 0x07);
    d.day = bcd2byte(data[0] & 0x3f);
    return d;
  }

  void set_date(const Date& d) {
    logger_.info("setting date");
    uint8_t data[] = {
      (uint8_t)(Registers::DATE),
      byte2bcd(d.day),
      byte2bcd(d.weekday),
      (uint8_t)(byte2bcd(d.month) | ((d.year < 2000) ? 0x80 : 0x00)),
      byte2bcd(d.year % 100)
    };
    write_(data, 5);
  }

  Time get_time() {
    logger_.info("getting time");
    uint8_t data[3];
    read_((uint8_t)(Registers::TIME), data, 3);
    Time t;
    t.hour = bcd2byte(data[2] & 0x3f);
    t.minute = bcd2byte(data[1] & 0x7f);
    t.second = bcd2byte(data[0] & 0x7f);
    return t;
  }

  void set_time(const Time& t) {
    logger_.info("Setting time");
    uint8_t data[] = {
      (uint8_t)(Registers::TIME),
      byte2bcd(t.second),
      byte2bcd(t.minute),
      byte2bcd(t.hour)
    };
    write_(data, 4);
  }

protected:
  void init() {
    uint8_t data[] = {
      (uint8_t)(Registers::CONTROL_STATUS1),
      0x00,
      0x00
    };
    write_(data, 3);
    logger_.info("initialized");
  }

  static constexpr int CENTURY_BIT = 0b10000000;
  enum class Registers : uint8_t {
    CONTROL_STATUS1  = 0x00,
    CONTROL_STATUS2  = 0x01,
    TIME             = 0x02, // seconds, minutes, hours
    SECONDS          = 0x02,
    MINUTES          = 0x03,
    HOURS            = 0x04,
    DATE             = 0x05, // day, weekday, month, year
    DAY              = 0x05,
    WEEKDAY          = 0x06,
    MONTH            = 0x07,
    YEAR             = 0x08,

    MINUTE_ALARM     = 0x09,
    HOUR_ALARM       = 0x0a,
    DAY_ALARM        = 0x0b,
    WEEKDAY_ALARM    = 0x0c,

    TIMER_CONTROL    = 0x0e,
    TIMER            = 0x0f,
  };

  write_fn write_;
  read_fn read_;
  espp::Logger logger_;
};
