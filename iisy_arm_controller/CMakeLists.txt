cmake_minimum_required(VERSION 3.8)
project(iisy_arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mpnp_interfaces REQUIRED)

add_executable(iisy_arm_controller src/iisy_arm_controller.cpp)
target_include_directories(iisy_arm_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(iisy_arm_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  iisy_arm_controller
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
  mpnp_interfaces
)

install(
  DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS iisy_arm_controller
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
  mpnp_interfaces
)

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
