cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)

project(cobot_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(gazebo_msgs REQUIRED)

find_package(linkattacher_msgs REQUIRED)  
find_package(ros2_linkattacher REQUIRED)  
find_package(gazebo_ros REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

find_package(geometry_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(test_ik src/test_ik.cpp)
ament_target_dependencies(test_ik rclcpp moveit_ros_planning_interface)

add_executable(test_pick_care src/test_pick_care.cpp)
ament_target_dependencies(test_pick_care rclcpp moveit_ros_planning_interface gazebo_msgs linkattacher_msgs ros2_linkattacher)

# add_executable(points_processing src/points_processing.cpp)
# ament_target_dependencies(points_processing rclcpp moveit_ros_planning_interface gazebo_msgs linkattacher_msgs ros2_linkattacher sensor_msgs pcl_conversions pcl_ros)

add_executable(save_pcd src/save_pcd.cpp)
ament_target_dependencies(save_pcd rclcpp moveit_ros_planning_interface gazebo_msgs linkattacher_msgs ros2_linkattacher sensor_msgs pcl_conversions pcl_ros PCL)

add_executable(process_points_image src/process_points_image.cpp)
ament_target_dependencies(process_points_image rclcpp moveit_ros_planning_interface gazebo_msgs linkattacher_msgs ros2_linkattacher sensor_msgs pcl_conversions pcl_ros PCL)

add_executable(process_points_stream src/process_points_stream.cpp)
ament_target_dependencies(process_points_stream rclcpp moveit_ros_planning_interface gazebo_msgs linkattacher_msgs ros2_linkattacher sensor_msgs pcl_conversions pcl_ros PCL geometry_msgs)

add_executable(test_pick_care_waypoints src/test_pick_care_waypoints.cpp)
ament_target_dependencies(test_pick_care_waypoints rclcpp moveit_ros_planning_interface gazebo_msgs linkattacher_msgs ros2_linkattacher sensor_msgs pcl_conversions pcl_ros PCL geometry_msgs)



install(TARGETS 
  test_ik test_pick_care save_pcd process_points_image process_points_stream test_pick_care_waypoints
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  clouds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
