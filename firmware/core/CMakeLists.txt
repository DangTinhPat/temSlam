cmake_minimum_required(VERSION 3.20)

add_library(rb1_core STATIC
  src/ImuStateEstimator.cpp
  src/LqrBalancer.cpp
  src/LqrTuning.cpp
  src/WholeBodyModel.cpp)
target_include_directories(rb1_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rb1_core PUBLIC cxx_std_17)
set_target_properties(rb1_core PROPERTIES CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rb1_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
  foreach(test imu_state_estimator_tests wheel_frame_tests)
    add_executable(${test} tests/${test}.cpp)
    target_link_libraries(${test} PRIVATE rb1_core)
    target_compile_options(${test} PRIVATE $<IF:$<CXX_COMPILER_ID:MSVC>,/UNDEBUG,-UNDEBUG>)
    add_test(NAME ${test} COMMAND ${test})
  endforeach()
  add_executable(lqr_tests tests/lqr_tests.cpp)
  target_link_libraries(lqr_tests PRIVATE rb1_core)
  add_test(NAME lqr_tests COMMAND lqr_tests)
endif()
