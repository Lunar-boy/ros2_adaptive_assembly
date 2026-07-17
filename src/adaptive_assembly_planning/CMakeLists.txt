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
find_package(geometric_shapes REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(
  grasp_clearance_validation
  src/grasp_clearance_validation.cpp
)
target_include_directories(
  grasp_clearance_validation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  grasp_clearance_validation
  moveit_ros_planning
  moveit_msgs
  geometric_shapes
)

add_executable(
  panda_pre_grasp_pose_adapter_node
  src/panda_pre_grasp_pose_adapter_node.cpp
)
ament_target_dependencies(
  panda_pre_grasp_pose_adapter_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(
  panda_assembly_pose_adapter_node
  src/panda_assembly_pose_adapter_node.cpp
)
ament_target_dependencies(
  panda_assembly_pose_adapter_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(
  assembly_sequence_planning_node
  src/assembly_sequence_planning_node.cpp
)
ament_target_dependencies(
  assembly_sequence_planning_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
)

target_include_directories(assembly_sequence_planning_node PRIVATE include)
target_link_libraries(assembly_sequence_planning_node grasp_clearance_validation)

add_executable(
  physical_target_planning_scene_node
  src/physical_target_planning_scene_node.cpp
)
ament_target_dependencies(
  physical_target_planning_scene_node
  rclcpp
  geometry_msgs
  std_msgs
  moveit_ros_planning
  moveit_msgs
  shape_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(physical_target_planning_scene_node PRIVATE include)

add_executable(
  grasp_clearance_calibration_node
  src/grasp_clearance_calibration_node.cpp
)
ament_target_dependencies(
  grasp_clearance_calibration_node
  rclcpp
  geometry_msgs
  moveit_ros_planning
  moveit_msgs
  trajectory_msgs
)
target_include_directories(grasp_clearance_calibration_node PRIVATE include)
target_link_libraries(grasp_clearance_calibration_node grasp_clearance_validation)

add_executable(static_planning_scene_node src/static_planning_scene_node.cpp)
ament_target_dependencies(
  static_planning_scene_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
)

add_executable(planning_scene_audit_node src/planning_scene_audit_node.cpp)
ament_target_dependencies(
  planning_scene_audit_node
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(
  payload_planning_scene_manager_node
  src/payload_planning_scene_manager_node.cpp
)
ament_target_dependencies(
  payload_planning_scene_manager_node
  rclcpp geometry_msgs sensor_msgs std_msgs moveit_ros_planning moveit_msgs shape_msgs
  tf2 tf2_ros tf2_geometry_msgs
)

install(
  TARGETS
    grasp_clearance_validation
    panda_pre_grasp_pose_adapter_node
    panda_assembly_pose_adapter_node
    assembly_sequence_planning_node
    static_planning_scene_node
    planning_scene_audit_node
    payload_planning_scene_manager_node
    physical_target_planning_scene_node
    grasp_clearance_calibration_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_assembly_sequence_planner_contract
    test/test_assembly_sequence_planner_contract.py
  )
  ament_add_pytest_test(
    test_grasp_clearance_calibration
    test/test_grasp_clearance_calibration.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_payload_planning_contract
    test/test_payload_planning_contract.py
  )
  ament_add_gtest(test_planning_contract test/test_planning_contract.cpp)
  target_include_directories(test_planning_contract PRIVATE include)
  ament_target_dependencies(
    test_planning_contract geometry_msgs moveit_msgs shape_msgs
  )
  target_link_libraries(test_planning_contract grasp_clearance_validation)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_ros_planning moveit_msgs geometric_shapes)

ament_package()
