cmake_minimum_required(VERSION 3.5)
project(tauv_depth)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
)

add_library(ms5837
  src/ms5837.cpp
)
target_include_directories(ms5837 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(depth
  src/depth_node.cpp
)
target_link_libraries(depth ms5837)
ament_target_dependencies(depth
  rclcpp
  nav_msgs
)

# Install C++ headers and targets
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ms5837
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  depth
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
