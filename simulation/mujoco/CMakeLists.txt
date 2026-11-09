cmake_minimum_required(VERSION 3.20)
project(rb1_sim LANGUAGES CXX)

find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# MuJoCo comes from MUJOCO_ROOT, or from the wheel installed for the current Python (it ships the headers).
if(NOT MUJOCO_ROOT)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c
    "import pathlib, mujoco; print(pathlib.Path(mujoco.__file__).resolve().parent)"
    OUTPUT_VARIABLE MUJOCO_ROOT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
endif()
if(NOT MUJOCO_ROOT OR NOT EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
  message(FATAL_ERROR "MuJoCo not found. Set MUJOCO_ROOT to a MuJoCo installation or install the mujoco Python wheel.")
endif()
file(GLOB MUJOCO_LIBRARY_CANDIDATES "${MUJOCO_ROOT}/libmujoco.so.*")
list(SORT MUJOCO_LIBRARY_CANDIDATES)
list(GET MUJOCO_LIBRARY_CANDIDATES -1 MUJOCO_LIBRARY)
if(NOT MUJOCO_LIBRARY)
  find_library(MUJOCO_LIBRARY NAMES mujoco PATHS "${MUJOCO_ROOT}" "${MUJOCO_ROOT}/lib" NO_DEFAULT_PATH)
endif()
if(NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo shared library not found below ${MUJOCO_ROOT}")
endif()

# Thư viện của firmware chạy trong mô phỏng.
foreach(lib hal core control)
  if(NOT TARGET rb1_${lib})
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/${lib} rb1_${lib}_build)
  endif()
endforeach()

# Runner MuJoCo là ứng dụng host độc lập về vị trí thư mục; nó liên kết thư viện controller hiện có.
add_executable(rb1_sim sim_runner.cpp)
target_compile_features(rb1_sim PRIVATE cxx_std_17)
target_compile_options(rb1_sim PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(rb1_sim PRIVATE "${MUJOCO_ROOT}/include" "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(rb1_sim PRIVATE rb1_control rb1_core rb1_hal
  "${MUJOCO_LIBRARY}" Threads::Threads dl)
set_target_properties(rb1_sim PROPERTIES BUILD_RPATH "${MUJOCO_ROOT}")
