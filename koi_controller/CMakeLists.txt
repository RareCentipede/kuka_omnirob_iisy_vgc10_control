cmake_minimum_required(VERSION 3.8)
project(koi_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  mpnp_interfaces
  nav_msgs
  tf2_geometry_msgs
  tf2_ros
  ros_gz_bridge
  moveit_task_constructor_core
)

# find dependencies
foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(koi_controller src/koi_controller.cpp)
add_executable(koi_pick_place_controller
               src/koi_pick_place_controller.cpp
               src/koi_pick_place_stages.cpp
               src/koi_pick_place_task.cpp)
add_executable(omnirob_controller src/omnirob_controller.cpp)
add_executable(iisy_arm_controller src/iisy_arm_controller.cpp)

target_include_directories(koi_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(koi_pick_place_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(omnirob_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(iisy_arm_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(koi_controller PUBLIC c_std_99)  # Require C99 and C++17
target_compile_features(koi_pick_place_controller PUBLIC c_std_99)  # Require C99 and C++17
target_compile_features(omnirob_controller PUBLIC c_std_99)  # Require C99 and C++17
target_compile_features(iisy_arm_controller PUBLIC c_std_99)  # Require C99 and C++17

ament_target_dependencies(koi_controller
  ${PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(koi_pick_place_controller
  ${PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(iisy_arm_controller
  ${PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(omnirob_controller
  ${PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS koi_controller omnirob_controller iisy_arm_controller koi_pick_place_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
