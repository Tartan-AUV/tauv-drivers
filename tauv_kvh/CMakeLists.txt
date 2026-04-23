cmake_minimum_required(VERSION 3.8)
project(tauv_kvh LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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
# Must come AFTER add_library
set_target_properties(imu_driver_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --------------------------------------------------------------------------
# ROS 2 component shared library
# --------------------------------------------------------------------------
add_library(tauv_kvh SHARED src/kvh_node.cpp)
target_include_directories(tauv_kvh PRIVATE ${DRIVER_DIR})
target_link_libraries(tauv_kvh imu_driver_lib)
ament_target_dependencies(tauv_kvh rclcpp sensor_msgs rclcpp_components nav_msgs tf2 tf2_ros tf2_geometry_msgs)

rclcpp_components_register_node(tauv_kvh
  PLUGIN "tauv_kvh::KvhNode"
  EXECUTABLE kvh_node
)

install(TARGETS kvh_node tauv_kvh
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()