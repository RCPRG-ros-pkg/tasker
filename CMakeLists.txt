cmake_minimum_required(VERSION 3.8)

project(tasker)

# Find packages in ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tasker_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate Python modules from message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CMD.msg"
  "msg/RobotResource.msg"
  "msg/ScheduleParams.msg"
  "msg/ShdlData.msg"
  "msg/ShdlDataStamped.msg"
  "msg/Status.msg"
  "msg/TaskState.msg"
  "srv/CostConditions.srv"
  "srv/HoldConditions.srv"
  "srv/LaunchConditions.srv"
  "srv/SchedulingConditions.srv"
  "srv/StartTask.srv"
  "srv/SuspendConditions.srv"
  "srv/TaskRequest.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Build the ROS 2 package
ament_package()