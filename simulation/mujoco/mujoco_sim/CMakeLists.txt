cmake_minimum_required(VERSION 3.5)
project(mujoco_sim)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
        cmake/CheckAvxSupport.cmake
        cmake/FindOrFetch.cmake
        cmake/MujocoHarden.cmake
        cmake/MujocoLinkOptions.cmake
        cmake/MujocoMacOS.cmake
        cmake/SimulateDependencies.cmake
        cmake/SimulateOptions.cmake
        )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadruped_msgs REQUIRED)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib;${CMAKE_CURRENT_SOURCE_DIR}/lib/mujoco_plugin")
set(CMAKE_SKIP_INSTALL_RPATH FALSE)


# BLASFEO Option
if(NOT TARGET mujoco_sim)
  add_library(mujoco_sim SHARED IMPORTED)
  set_target_properties(mujoco_sim PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libmujoco.so)
endif()

# ros2 node plugins
file(GLOB_RECURSE plugin_source 
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp 
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.hpp
)
add_library(mujoco_sim_node_plugins SHARED ${plugin_source})
ament_target_dependencies(mujoco_sim_node_plugins 
        rclcpp 
        pluginlib
        sensor_msgs
        common_msgs 
        quadruped_msgs)
target_include_directories(mujoco_sim_node_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plugins/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(mujoco_sim mujoco_sim_plugins.xml)
install(TARGETS mujoco_sim_node_plugins
    EXPORT export_mujoco_sim_node_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_libraries(mujoco_sim_node_plugins)
ament_export_targets(
  export_mujoco_sim_node_plugins
)
# install (FILES mujoco_sim_plugins.xml
#     DESTINATION share/${PROJECT_NAME}
# )

# mujoco plugins 
file(GLOB SO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/lib/mujoco_plugin/*.so")
set (PLUGIN_LIBS "")
foreach(SO_FILE ${SO_FILES})
    get_filename_component(LIB_NAME ${SO_FILE} NAME_WE)
    add_library(${LIB_NAME} SHARED IMPORTED)
    set_target_properties(${LIB_NAME} PROPERTIES
        IMPORTED_LOCATION "${SO_FILE}"
    )
    list(APPEND PLUGIN_LIBS ${LIB_NAME})  # Save the target name
endforeach()

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
add_executable(simulation ${source})
target_link_libraries(simulation mujoco_sim_node_plugins mujoco_sim glfw pthread rt ${CMAKE_DL_LIBS} ${PLUGIN_LIBS})
ament_target_dependencies(simulation pluginlib rclcpp)
target_include_directories(simulation PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)



list(APPEND MUJOCO_CXX_FLAGS
"-march=native"
"-mtune=native"
"-fPIC"
"-pthread"
"-Wfatal-errors"
"-Wl,--no-as-needed"
)

# Force Boost dynamic linking
list(APPEND MUJOCO_CXX_FLAGS
  "-DBOOST_ALL_DYN_LINK"
  )

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

install(DIRECTORY 
        launch 
        config
        DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY plugins/include/
        DESTINATION include/
)

install(TARGETS simulation
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY lib/mujoco_plugin
        DESTINATION lib/${PROJECT_NAME})

ament_package()
