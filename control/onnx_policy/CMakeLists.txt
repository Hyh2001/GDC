cmake_minimum_required(VERSION 3.16)
project(onnx_policy)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(onnxruntime_vendor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(mdp INTERFACE)
target_compile_features(mdp INTERFACE cxx_std_17)
target_include_directories(mdp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mdp INTERFACE Eigen3::Eigen)

add_library(policy_runtime SHARED src/onnx_policy.cpp)
target_compile_features(policy_runtime PUBLIC cxx_std_17)
target_include_directories(policy_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(policy_runtime PUBLIC onnxruntime::onnxruntime)

add_library(controller_base SHARED src/onnx_policy_controller.cpp)
target_compile_features(controller_base PUBLIC cxx_std_17)
target_link_libraries(controller_base PUBLIC policy_runtime mdp)
ament_target_dependencies(controller_base PUBLIC
  ament_index_cpp rclcpp rclcpp_lifecycle controller_interface hardware_interface)

foreach(target policy_runtime controller_base)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
foreach(target policy_runtime mdp controller_base)
  add_library(${PROJECT_NAME}::${target} ALIAS ${target})
endforeach()

install(TARGETS policy_runtime mdp controller_base EXPORT export_onnx_policy
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY examples/ DESTINATION share/${PROJECT_NAME}/examples)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_onnx_policy HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 onnxruntime_vendor ament_index_cpp rclcpp
  rclcpp_lifecycle controller_interface hardware_interface)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test runtime mdp controller)
    ament_add_gtest(test_${test} test/test_${test}.cpp)
    if(test STREQUAL "runtime")
      target_link_libraries(test_${test} policy_runtime)
    elseif(test STREQUAL "mdp")
      target_link_libraries(test_${test} mdp)
    else()
      target_link_libraries(test_${test} controller_base)
    endif()
    target_compile_definitions(test_${test} PRIVATE
      FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  endforeach()
  add_library(example_controller OBJECT examples/example_controller.cpp)
  target_link_libraries(example_controller PRIVATE controller_base)
  add_test(NAME installed_consumer COMMAND ${CMAKE_COMMAND}
    -DCONSUMER_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/test/consumer
    -DCONSUMER_BUILD=${CMAKE_CURRENT_BINARY_DIR}/installed_consumer
    -DPACKAGE_DIR=${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake
    -DCONSUMER_COMPILER=${CMAKE_CXX_COMPILER}
    -DMODEL=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/mlp.onnx
    -P ${CMAKE_CURRENT_SOURCE_DIR}/test/verify_consumer.cmake)
endif()

ament_package()
