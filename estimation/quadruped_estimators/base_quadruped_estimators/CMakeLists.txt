cmake_minimum_required(VERSION 3.8)
project(base_quadruped_estimators)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

find_package(quadruped_msgs REQUIRED)

file(GLOB_RECURSE SOURCES src/*.cpp src/*.cc)
add_library(quadruped_estimators SHARED
        ${SOURCES}
        )
target_include_directories(quadruped_estimators PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(quadruped_estimators PUBLIC 
        rclcpp 
        rclcpp_lifecycle
        controller_interface
        hardware_interface
        pluginlib
        realtime_tools
        quadruped_msgs
)
pluginlib_export_plugin_description_file(controller_interface base_quadruped_estimators_config.xml)
install(TARGETS quadruped_estimators
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(quadruped_estimators)
ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)
# message(WARNING "CMake PROJECT_NAME='${PROJECT_NAME}'")

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
