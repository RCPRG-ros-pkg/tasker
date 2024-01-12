cmake_minimum_required(VERSION 3.5)
project(tasker)

# if(POLICY CMP0148)
#   cmake_policy(SET CMP0148 NEW)
# endif()

# Find packages in ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tasker_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install Python script
install(PROGRAMS
  tasker/task_harmonizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(tasker PACKAGE_DIR src/TaskER)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()