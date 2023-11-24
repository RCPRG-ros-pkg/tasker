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

# # Explicitly find Python3
# find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Generate custom messages and services


# # Install Python modules
ament_python_install_package(tasker PACKAGE_DIR src/TaskER)

# Specify install targets (optional, but recommended for C++ nodes, launch files, etc.)
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Build the ROS 2 package
ament_package()
