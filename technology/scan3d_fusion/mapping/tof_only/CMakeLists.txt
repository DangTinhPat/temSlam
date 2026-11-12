cmake_minimum_required(VERSION 3.20)
project(rb_tof_only LANGUAGES CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(TOF_CALIBRATION "${CMAKE_CURRENT_BINARY_DIR}/tof_calibration.hpp")
set(TOF_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/../../config/tof.yaml")
execute_process(
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_tof_calibration.py ${TOF_CALIBRATION}
  COMMAND_ERROR_IS_FATAL ANY)

add_executable(rb_tof_mapper mapper.cpp)
target_compile_features(rb_tof_mapper PRIVATE cxx_std_17)
target_compile_options(rb_tof_mapper PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(rb_tof_mapper PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${TOF_CONFIG}" "${CMAKE_CURRENT_SOURCE_DIR}/generate_tof_calibration.py")
