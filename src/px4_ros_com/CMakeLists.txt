cmake_minimum_required(VERSION 3.5)
project(px4_ros_com)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# traj_test
add_executable(traj_test src/traj_test.cpp src/motionProfiling.cpp src/rviz_utils.cpp src/lib/frame_transforms.cpp)
ament_target_dependencies(traj_test rclcpp px4_msgs rclcpp geometry_msgs visualization_msgs Eigen3)
install(TARGETS traj_test DESTINATION lib/${PROJECT_NAME})

# Build your test executable
add_executable(test_trajectory test/test_trajectory.cpp src/trajectory.cpp src/rviz_utils.cpp src/lib/frame_transforms.cpp)
ament_target_dependencies(test_trajectory rclcpp px4_msgs rclcpp geometry_msgs visualization_msgs Eigen3)
install(TARGETS test_trajectory DESTINATION lib/${PROJECT_NAME})

# detection_node (C++)
add_executable(detection_node src/detection.cpp)
ament_target_dependencies(detection_node rclcpp sensor_msgs cv_bridge OpenCV)
install(TARGETS detection_node DESTINATION lib/${PROJECT_NAME})

# offboard_control
add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# image_stitching_node (C++) (Optional — you can remove if only using Python)
add_executable(image_stitching_node src/image_stitching_node.cpp)
ament_target_dependencies(image_stitching_node rclcpp sensor_msgs cv_bridge OpenCV)
install(TARGETS image_stitching_node DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)

############
# Python Nodes
############

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	src/detection.py
	src/image_stitching_node.py  # <<<<<< Added your new Python ROS 2 node
	DESTINATION lib/${PROJECT_NAME}
)

############
# Testing ##
############

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
