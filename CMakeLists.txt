# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
)

add_compile_definitions(BOARD_HAS_PSRAM)

set(
  COMPONENTS
  "main esptool_py esp_psram esp32-camera esp32-timer-cam mdns monitor nvs rtsp socket task wifi "
  CACHE STRING
  "List of components to include"
  )

project(camera-streamer)

set(CMAKE_CXX_STANDARD 20)
