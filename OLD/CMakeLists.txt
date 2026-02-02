cmake_minimum_required(VERSION 3.22)
project(tauv_drivers)

# Find build tools and dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tauv_msgs REQUIRED)

# 1. Install the Python module (the inner tauv_drivers folder)
# This allows 'import tauv_drivers' to work across the workspace
ament_python_install_package(${PROJECT_NAME})

# 2. Install Python Executables (scripts/)
# These are the files that 'ros2 run' actually looks for
install(PROGRAMS
  scripts/waterlinked_driver
  scripts/can_driver
  scripts/depth_actuators_i2c
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install Launch and Config files
# This ensures launch files are found by 'ros2 launch' ADD LAUNCH HERE LATER
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

# 4. Standard ROS 2 Testing logic
if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()