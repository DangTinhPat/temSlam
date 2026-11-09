cmake_minimum_required(VERSION 3.20)

add_library(rb1_hal INTERFACE)
target_include_directories(rb1_hal INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rb1_hal INTERFACE cxx_std_17)

include(CTest)
if(BUILD_TESTING)
  add_executable(motor_frame_codec_tests tests/motor_frame_codec_tests.cpp)
  target_link_libraries(motor_frame_codec_tests PRIVATE rb1_hal)
  target_compile_options(motor_frame_codec_tests PRIVATE $<IF:$<CXX_COMPILER_ID:MSVC>,/UNDEBUG,-UNDEBUG>)
  add_test(NAME motor_frame_codec_tests COMMAND motor_frame_codec_tests)
endif()
