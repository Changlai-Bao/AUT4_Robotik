cmake_minimum_required(VERSION 3.8)
project(xnucleo_iks01a3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(xnucleo_iks01a3_node src/xnucleo_iks01a3_node.cpp src/SerialPort.cpp)
ament_target_dependencies(xnucleo_iks01a3_node rclcpp sensor_msgs)

add_executable(xnucleo_iks01a3_fusion_node src/xnucleo_iks01a3_fusion_node.cpp src/Quaternion.cpp src/Vector3.cpp)
ament_target_dependencies(xnucleo_iks01a3_fusion_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  xnucleo_iks01a3_node
  xnucleo_iks01a3_fusion_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
