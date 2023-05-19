cmake_minimum_required(VERSION 3.8)
project(ekf_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(april_detect_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ekf_interfaces REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(rcpputils REQUIRED)

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

add_executable(ekf_slam_node ${SOURCES})
target_include_directories(ekf_slam_node PRIVATE libs)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_target_dependencies(ekf_slam_node Eigen3 rcpputils rclcpp rclcpp_components sensor_msgs visualization_msgs tf2_ros nav_msgs ekf_interfaces geometry_msgs april_detect_msgs std_msgs pcl_ros PCL)


add_executable(fusion_node src/fusion.cpp)
target_include_directories(fusion_node PRIVATE libs)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_target_dependencies(fusion_node Eigen3 rclcpp rcpputils rclcpp_components sensor_msgs visualization_msgs tf2_ros nav_msgs ekf_interfaces geometry_msgs april_detect_msgs std_msgs pcl_ros PCL)


install(TARGETS
  ekf_slam_node
  fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()