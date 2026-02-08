cmake_minimum_required(VERSION 3.10)
project(base_humanoid_estimators)

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

find_package(humanoid_msgs REQUIRED)

file(GLOB_RECURSE SOURCES src/*.cpp src/*.cc)
add_library(humanoid_estimators SHARED
        ${SOURCES}
        )
target_include_directories(humanoid_estimators PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(humanoid_estimators PUBLIC
        rclcpp
        rclcpp_lifecycle
        controller_interface
        hardware_interface
        pluginlib
        realtime_tools
        humanoid_msgs
)
pluginlib_export_plugin_description_file(controller_interface base_humanoid_estimators_config.xml)
install(TARGETS humanoid_estimators
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(humanoid_estimators)
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
