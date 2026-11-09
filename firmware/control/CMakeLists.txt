cmake_minimum_required(VERSION 3.20)

add_library(rb1_control STATIC
  src/RobotController.cpp
  src/comm/BusTransport.cpp
  src/comm/MotorFeedback.cpp
  src/legs/LegHeight.cpp
  src/legs/LegLock.cpp)
target_include_directories(rb1_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rb1_control PUBLIC cxx_std_17)
# Prototype tách CoM/gimbal chỉ chạy trên host, không có trong firmware ESP32.
target_compile_definitions(rb1_control PUBLIC RB1_SPLIT_HOST_PROTOTYPE=1)
target_link_libraries(rb1_control PUBLIC rb1_core rb1_hal)
set_target_properties(rb1_control PROPERTIES CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rb1_control PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
  foreach(test
      yaw_hold_tests addwifi_balance_tests motor_feedback_tests imu_mount_tests controller_tests
      standup_tests motion_profile_tests plan_scurve_tests recovery_transfer_tests
      oscillation_guard_tests physics_tests tier_tests coordination_tests leg_height_tests split_tests)
    add_executable(${test} tests/${test}.cpp)
    target_link_libraries(${test} PRIVATE rb1_control)
    target_compile_options(${test} PRIVATE $<IF:$<CXX_COMPILER_ID:MSVC>,/UNDEBUG,-UNDEBUG>)
    add_test(NAME ${test} COMMAND ${test})
  endforeach()

  add_executable(split_replay tests/split_replay.cpp)
  target_compile_definitions(split_replay PRIVATE RB1_SPLIT_REPLAY_CURRENT=1)
  target_link_libraries(split_replay PRIVATE rb1_control)

  # Công cụ thiết kế LQR toàn thân (chỉ host): ghi balance/WholeBodyGains.hpp.
  find_package(Eigen3 3.3 NO_MODULE QUIET)
  if(Eigen3_FOUND)
    add_executable(wb_lqr_design tools/wb_lqr_design.cpp)
    target_include_directories(wb_lqr_design PRIVATE tests)
    target_link_libraries(wb_lqr_design PRIVATE rb1_control Eigen3::Eigen)
    # tier_tests tự thiết kế hệ số toàn thân trong mô phỏng.
    target_link_libraries(tier_tests PRIVATE Eigen3::Eigen)
    target_compile_definitions(tier_tests PRIVATE RB1_HAVE_EIGEN=1)
  endif()
endif()
