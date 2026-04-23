cmake_minimum_required(VERSION 3.10.2)
project(KVH_1775_IMU_DRIVER LANGUAGES C CXX)

set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_library(
  imu_driver
  STATIC
  utils.cpp
  driver.cpp
  datagram_parser.cpp
  crc.c
  serial_unix.cpp
  serial_driver.cpp
)

add_executable(main main.cpp)
target_link_libraries(main imu_driver)