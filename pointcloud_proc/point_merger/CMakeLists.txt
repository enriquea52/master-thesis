cmake_minimum_required(VERSION 3.8)
project(point_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)
include_directories("include" ${Eigen3_DIR})

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

set(SOURCES src/main.cpp )

add_executable(point_merger_node ${SOURCES})
target_include_directories(point_merger_node PRIVATE libs)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_target_dependencies(point_merger_node Eigen3 rclcpp sensor_msgs std_msgs tf2_ros tf2 PCL pcl_ros laser_geometry message_filters )


add_executable(icp_odom_node src/icp_odom.cpp)
target_include_directories(icp_odom_node PRIVATE libs)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_target_dependencies(icp_odom_node Eigen3 rclcpp sensor_msgs std_msgs tf2_ros tf2 PCL pcl_ros laser_geometry message_filters )


install(TARGETS
  point_merger_node
  icp_odom_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/converter.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
