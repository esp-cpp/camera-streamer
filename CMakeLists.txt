# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
)

add_compile_definitions(BOARD_HAS_PSRAM)

# The set of board-specific components (esp32-timer-cam vs xiao-esp32s3-sense)
# depends on the target chip, so component selection is handled by the
# component manager (main/idf_component.yml) and main/CMakeLists.txt REQUIRES
# rather than a hard-coded COMPONENTS list here.

project(camera-streamer)

set(CMAKE_CXX_STANDARD 20)
