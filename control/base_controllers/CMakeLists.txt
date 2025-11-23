cmake_minimum_required(VERSION 3.8)
project(base_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(rclcpp REQUIRED)

# ONNX Runtine
if(DEFINED ENV{ONNXRUNTIME_ROOT_PATH})
    set(ONNXRUNTIME_ROOT_PATH $ENV{ONNXRUNTIME_ROOT_PATH})
    message(STATUS "Using ONNXRUNTIME_ROOT_PATH from environment: ${ONNXRUNTIME_ROOT_PATH}")
else()
    # Fallback to default path
    set(ONNXRUNTIME_ROOT_PATH /usr/local/onnxruntime)
    message(STATUS "Using default ONNXRUNTIME_ROOT_PATH: ${ONNXRUNTIME_ROOT_PATH}")
    message(STATUS "To override, set environment variable: export ONNXRUNTIME_ROOT_PATH=/your/path")
endif()
set(ONNXRUNTIME_INCLUDE_DIRS ${ONNXRUNTIME_ROOT_PATH}/include)
set(ONNXRUNTIME_LIB_DIRS ${ONNXRUNTIME_ROOT_PATH}/lib)

find_library(ONNXRUNTIME_LIB onnxruntime 
    PATHS ${ONNXRUNTIME_LIB_DIRS}
    NO_DEFAULT_PATH
)
if(NOT ONNXRUNTIME_LIB)
    message(FATAL_ERROR "ONNX Runtime library not found in ${ONNXRUNTIME_LIB_DIRS}")
endif()

file(GLOB_RECURSE BASE_CONTROLLER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

add_library(${PROJECT_NAME} SHARED 
    ${BASE_CONTROLLER_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
    control_toolbox
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ONNXRUNTIME_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${ONNXRUNTIME_LIB}
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "${ONNXRUNTIME_LIB_DIRS}"
    BUILD_RPATH "${ONNXRUNTIME_LIB_DIRS}"
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install header files
install(DIRECTORY include/
  DESTINATION include
)
# Export include directories for downstream packages
ament_export_include_directories(include ${ONNXRUNTIME_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp control_toolbox)

# Create a custom target for ONNX Runtime
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/onnxruntime-config.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
    OPTIONAL
)
# Alternative: Export via ament_export_libraries
ament_export_libraries(${PROJECT_NAME} ${ONNXRUNTIME_LIB})
# Export library directories so downstream packages know where to find ONNX Runtime
set(ONNXRUNTIME_LIBRARIES ${ONNXRUNTIME_LIB})
ament_export_libraries(${ONNXRUNTIME_LIBRARIES})
# Export include directories
ament_export_include_directories(include ${ONNXRUNTIME_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Add ONNX policy test executable
  add_executable(test_onnx_policy test/test_onnx_policy.cpp)
  
  target_include_directories(test_onnx_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ONNXRUNTIME_INCLUDE_DIRS}
  )
  
  target_link_libraries(test_onnx_policy 
    ${PROJECT_NAME}
    ${ONNXRUNTIME_LIB}
  )
  
  # Set RPATH for the test executable
  set_target_properties(test_onnx_policy PROPERTIES
    INSTALL_RPATH "${ONNXRUNTIME_LIB_DIRS}"
    BUILD_RPATH "${ONNXRUNTIME_LIB_DIRS}"
  )
  
  # Optionally install the test executable
  install(TARGETS test_onnx_policy
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
