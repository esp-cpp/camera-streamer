idf_component_register(
  INCLUDE_DIRS "../../thirdparty/micro-rtsp/src"
  SRC_DIRS "../../thirdparty/micro-rtsp/src"
  REQUIRES "esp32-camera"
  )

target_compile_options(${COMPONENT_LIB} INTERFACE "-Wno-format-truncation")
