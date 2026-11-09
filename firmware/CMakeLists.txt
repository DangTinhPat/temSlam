cmake_minimum_required(VERSION 3.20)
project(rb1_firmware LANGUAGES CXX)

# Build native (máy host): thư viện điều khiển, test và trình mô phỏng MuJoCo.
# Firmware ESP32 build riêng bằng PlatformIO (xem platformio.ini).
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
include(CTest)

add_subdirectory(hal)
add_subdirectory(core)
add_subdirectory(control)

option(RB1_BUILD_SIMULATOR "Build MuJoCo simulator" ON)
if(RB1_BUILD_SIMULATOR)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../simulation/mujoco sim)
endif()
