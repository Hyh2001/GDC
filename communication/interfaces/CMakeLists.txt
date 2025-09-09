cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadruped_msgs REQUIRED)
find_package(common_msgs REQUIRED)
# hardware interfaces
add_library(pendulum_mujoco_hardware_interface SHARED
        hardware/pendulum_mujoco_hardware_interface.cpp
        ${source})
target_include_directories(pendulum_mujoco_hardware_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pendulum_mujoco_hardware_interface PUBLIC 
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        sensor_msgs 
        common_msgs 
        quadruped_msgs)
target_link_libraries(pendulum_mujoco_hardware_interface PUBLIC mujoco_sim glfw pthread rt ${CMAKE_DL_LIBS} ${PLUGIN_LIBS})
pluginlib_export_plugin_description_file(hardware_interface
        ros2_control_mujoco_sim.xml)
install(TARGETS pendulum_mujoco_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(pendulum_mujoco_hardware_interface)
ament_export_targets(
  export_${PROJECT_NAME}
)

install(
        DIRECTORY hardware/include/
        DESTINATION include/
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
