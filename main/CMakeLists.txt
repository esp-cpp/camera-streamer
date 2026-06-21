idf_build_get_property(target IDF_TARGET)

if(target STREQUAL "esp32s3")
  set(board_requires xiao-esp32s3-sense esp_driver_i2s)
else()
  set(board_requires esp32-timer-cam)
endif()

idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       REQUIRES esp_psram esp32-camera mdns esp_wifi nvs_flash
                                cli monitor nvs rtsp socket task wifi
                                ${board_requires})
