cmake_minimum_required(VERSION 3.8)
project(adaptive_assembly_planning)

set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python3 interpreter" FORCE)
set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter" FORCE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(pre_grasp_planning_node src/pre_grasp_planning_node.cpp)
ament_target_dependencies(
  pre_grasp_planning_node
  rclcpp
  geometry_msgs
  std_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(
  panda_pre_grasp_pose_adapter_node
  src/panda_pre_grasp_pose_adapter_node.cpp
)
ament_target_dependencies(
  panda_pre_grasp_pose_adapter_node
  rclcpp
  geometry_msgs
)

install(
  TARGETS
    pre_grasp_planning_node
    panda_pre_grasp_pose_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
