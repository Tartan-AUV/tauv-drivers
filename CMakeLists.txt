cmake_minimum_required(VERSION 3.22)
project(tauv_drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ---------------------------------------------------------
# DEPENDENCIES
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tauv_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------
# BUILD XSENS C++ DRIVER
# ---------------------------------------------------------

# 1. Compilation Flags
add_compile_definitions(XDA_DLL_API=)
add_definitions(-D__GNU__ -D__LINUX__)

# 2. Include Directories
include_directories(src)
include_directories(src/xspublic)
include_directories(src/xspublic/xscontroller)
include_directories(src/xspublic/xscommon)
include_directories(src/xspublic/xstypes)
include_directories(${EIGEN3_INCLUDE_DIR})

# 3. Find Sources
file(GLOB_RECURSE XSENS_SDK_SRCS
    "src/xspublic/xscontroller/*.cpp" "src/xspublic/xscontroller/*.c"
    "src/xspublic/xscommon/*.cpp"     "src/xspublic/xscommon/*.c"
    "src/xspublic/xstypes/*.cpp"      "src/xspublic/xstypes/*.c"
)
file(GLOB XSENS_PUB_SRCS "src/messagepublishers/*.cpp")

# 4. Define Executable
# CRITICAL: Added src/xdacallback.cpp here explicitly!
add_executable(xsens_driver
  src/xsens_node.cpp
  src/xdainterface.cpp
  src/xdacallback.cpp  
  ${XSENS_PUB_SRCS}
  ${XSENS_SDK_SRCS}
)

# 5. Link Dependencies
ament_target_dependencies(xsens_driver
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  mavros_msgs
  tauv_msgs
)

target_link_libraries(xsens_driver
  Eigen3::Eigen
  pthread
  dl
)

# 6. Install C++ Node
install(TARGETS xsens_driver
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------
# INSTALL PYTHON SCRIPTS
# ---------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/waterlinked_driver
  scripts/can_driver
  scripts/depth_actuators_i2c
  DESTINATION lib/${PROJECT_NAME}
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
