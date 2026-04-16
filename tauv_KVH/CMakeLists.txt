cmake_minimum_required(VERSION 3.8)
project(tauv_kvh LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/KVH_1775_IMU_driver)

# --------------------------------------------------------------------------
# Static library: reuse existing C/C++ driver sources unchanged
# --------------------------------------------------------------------------
add_library(imu_driver_lib STATIC
  ${DRIVER_DIR}/crc.c
  ${DRIVER_DIR}/utils.cpp
  ${DRIVER_DIR}/driver.cpp
  ${DRIVER_DIR}/datagram_parser.cpp
  ${DRIVER_DIR}/serial_unix.cpp
  ${DRIVER_DIR}/serial_driver.cpp
)
target_include_directories(imu_driver_lib PUBLIC ${DRIVER_DIR})

# --------------------------------------------------------------------------
# ROS 2 node executable
# --------------------------------------------------------------------------
add_executable(kvh_node src/kvh_node.cpp)
target_include_directories(kvh_node PRIVATE ${DRIVER_DIR})
target_link_libraries(kvh_node imu_driver_lib)
ament_target_dependencies(kvh_node rclcpp sensor_msgs)

install(TARGETS kvh_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
