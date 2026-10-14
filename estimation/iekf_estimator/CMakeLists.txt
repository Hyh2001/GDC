cmake_minimum_required(VERSION 3.16)
project(iekf_estimator)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(estimator SHARED
  src/IEKF.cpp src/MOB.cpp src/IEKF/InEKF.cpp src/IEKF/LieGroup.cpp
  src/IEKF/NoiseParams.cpp src/IEKF/RobotState.cpp)
target_compile_features(estimator PUBLIC cxx_std_17)
target_include_directories(estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(estimator PUBLIC Eigen3::Eigen pinocchio::pinocchio)
set_target_properties(estimator PROPERTIES OUTPUT_NAME iekf_estimator)

add_library(controller_base SHARED src/iekf_estimator.cpp)
target_compile_features(controller_base PUBLIC cxx_std_17)
target_link_libraries(controller_base PUBLIC estimator)
ament_target_dependencies(controller_base PUBLIC
  ament_index_cpp rclcpp rclcpp_lifecycle controller_interface hardware_interface)
set_target_properties(controller_base PROPERTIES OUTPUT_NAME iekf_estimator_controller_base)

foreach(target estimator controller_base)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  add_library(${PROJECT_NAME}::${target} ALIAS ${target})
endforeach()
install(TARGETS estimator controller_base EXPORT export_iekf_estimator
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY examples/ DESTINATION share/${PROJECT_NAME}/examples)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_iekf_estimator HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio ament_index_cpp rclcpp
  rclcpp_lifecycle controller_interface hardware_interface)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_iekf test/test_iekf.cpp)
  target_link_libraries(test_iekf estimator)
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller controller_base)
  target_compile_definitions(test_controller PRIVATE FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_library(example_controller OBJECT examples/example_controller.cpp)
  target_link_libraries(example_controller PRIVATE controller_base)
  add_test(NAME installed_consumer COMMAND ${CMAKE_COMMAND}
    -DCONSUMER_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/test/consumer
    -DCONSUMER_BUILD=${CMAKE_CURRENT_BINARY_DIR}/installed_consumer
    -DPACKAGE_DIR=${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake
    -DCONSUMER_COMPILER=${CMAKE_CXX_COMPILER}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/test/verify_consumer.cmake)
endif()

ament_package()
