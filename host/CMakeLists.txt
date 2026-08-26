cmake_minimum_required(VERSION 3.20)
project(imu_host CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

include_directories(include)

# SerialPort has two mutually-exclusive platform backends -- see
# include/network/serial_port.hpp's header comment for why. Only one is
# ever compiled, chosen here, not by an #ifdef inside a shared .cpp file.
if(WIN32)
  set(SERIAL_PORT_SRC src/network/serial_port_windows.cpp)
else()
  set(SERIAL_PORT_SRC src/network/serial_port_posix.cpp)
endif()

set(IMU_HOST_SOURCES
    src/fusion/fusion.cpp
    src/network/frame_parser.cpp
    ${SERIAL_PORT_SRC}
)

add_executable(imu_host src/main.cpp ${IMU_HOST_SOURCES})

option(BUILD_TESTS "Build the GoogleTest suite" ON)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
