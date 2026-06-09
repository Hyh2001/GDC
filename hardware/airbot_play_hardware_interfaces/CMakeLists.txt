cmake_minimum_required(VERSION 3.8)
project(airbot_play_hardware_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(airbot_hardware REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(manipulator_msgs REQUIRED)
find_package(base_hardware_interfaces REQUIRED)
find_package(Boost REQUIRED)
# message(WARN "AirbotSdk_DIR = ${AirbotSdk_DIR}")

set(HARDWARE_INTERFACE_TARGETS
  airbot_play_arm_hardware_interface
  airbot_play_arm_g2_hardware_interface
)

foreach(target_name IN LISTS HARDWARE_INTERFACE_TARGETS)
  set(target_src "${CMAKE_CURRENT_SOURCE_DIR}/src/${target_name}.cpp")
  if(NOT EXISTS "${target_src}")
    message(FATAL_ERROR "Missing source file for target '${target_name}': ${target_src}")
  endif()

  add_executable(${target_name}
    ${target_src}
  )
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  ament_target_dependencies(${target_name} PUBLIC
    rclcpp
    rclcpp_lifecycle
    manipulator_msgs
    base_hardware_interfaces
  )
  target_link_libraries(${target_name} PUBLIC
    airbot_hardware::airbot_hardware
    Boost::headers
    pthread
  )
  install(
    TARGETS ${target_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

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
