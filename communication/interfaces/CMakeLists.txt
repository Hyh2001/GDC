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
find_package(realtime_tools REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadruped_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(pend_msgs REQUIRED)
# hardware interfaces
file(GLOB_RECURSE HARDWARE_SOURCES hardware/*.cpp)
# message(WARNING "HARDWARE_SOURCES: ${HARDWARE_SOURCES}")
add_library(hardware_interfaces SHARED
        ${HARDWARE_SOURCES}
        ${source})
target_include_directories(hardware_interfaces PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
        # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include/quadrupeds>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(hardware_interfaces PUBLIC 
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        sensor_msgs 
        common_msgs 
        quadruped_msgs
        pend_msgs
)
pluginlib_export_plugin_description_file(hardware_interface
        hardware_config.xml)
install(TARGETS hardware_interfaces
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(hardware_interfaces)
ament_export_targets(
  export_${PROJECT_NAME}
)

install(
        DIRECTORY hardware/include/
        DESTINATION include/
)

install(DIRECTORY 
        test
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
