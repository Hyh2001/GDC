cmake_minimum_required(VERSION 3.8)
project(base_hardware_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

find_package(end_effector_msgs REQUIRED)
find_package(humanoid_msgs REQUIRED)
find_package(manipulator_msgs REQUIRED)
find_package(quadruped_msgs REQUIRED)

add_library(base_hardware_interface INTERFACE)
target_include_directories(base_hardware_interface INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(base_hardware_interface INTERFACE
        rclcpp 
        rclcpp_lifecycle
        realtime_tools
        end_effector_msgs
        humanoid_msgs
        manipulator_msgs
        quadruped_msgs
)
install(TARGETS base_hardware_interface
  EXPORT export_${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  realtime_tools
  humanoid_msgs
  quadruped_msgs
  manipulator_msgs
  end_effector_msgs
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
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
