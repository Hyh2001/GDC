
- [ ] Double check the contents and modify to the correct ones

## Steps to build a custom estimators
1. Create the package
   - Use ament_cmake and declare runtime/build dependencies in package.xml:
     - build/exec: rclcpp, controller_interface, pluginlib, ament_index_cpp (if needed), realtime_tools (optional)
     - example:
       ```xml
       <build_depend>controller_interface</build_depend>
       <exec_depend>pluginlib</exec_depend>
       <exec_depend>rclcpp</exec_depend>
       ```
2. Implement the estimator class
   - Inherit from controller_interface::ChainableControllerInterface (or ControllerInterface).
   - Implement required lifecycle/interface methods and export the plugin:
     ```cpp
     PLUGINLIB_EXPORT_CLASS(my_pkg::MyEstimator, controller_interface::ChainableControllerInterface);
     ```
3. Provide a plugin XML (example file: my_pkg_config.xml)
   - Ensure the file contains a correct library path and class/type mapping:
     ```xml
     <library path="library">
       <class name="my_pkg::MyEstimator"
              type="my_pkg/MyEstimator"
              base_class_type="controller_interface::ChainableControllerInterface">
         <description>My estimator</description>
       </class>
     </library>
     ```
4. CMakeLists.txt essentials
   - Build shared library, link dependencies, install library and plugin XML, and export plugin:
     ```cmake
     add_library(${PROJECT_NAME} SHARED src/dummy_estimator.cpp)
     ament_target_dependencies(${PROJECT_NAME} rclcpp controller_interface pluginlib)
     install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
     install(FILES my_pkg_config.xml DESTINATION share/${PROJECT_NAME})
     pluginlib_export_plugin_description_file(controller_interface my_pkg_config.xml)
     ament_package()
     ```
5. Register controller in controller_manager params (robot_control.yaml)
   - Add an entry so the spawner can find the controller `type`:
     ```yaml
     controller_manager:
       ros__parameters:
         update_rate: 100
     my_estimator:
       type: "my_pkg/MyEstimator"
       ros__parameters:
         # optional params
     ```
6. Build and source
   - Prefer cleaning the CMake cache for packages that changed plugin/xml:
     ```bash
     colcon build --packages-select my_pkg --cmake-clean-cache
     source install/setup.bash
     ```
7. Test loading
   - Spawn the controller:
     ```bash
     ros2 run controller_manager spawner my_estimator --controller-manager /controller_manager
     ```
   - Verify plugin xml and library are installed:
     ```bash
     ls -l install/my_pkg/share/my_pkg/my_pkg_config.xml
     ls -l install/lib | grep my_pkg
     ```



## Troubleshooting: plugin not found
If ros2_control/controller_manager reports "Loader for controller ... not found" or a plugin cannot be located after building:
- Clean the package CMake cache and rebuild the package that provides the plugin:
  ```bash
  colcon build --packages-select <package_name> --cmake-clean-cache
  source install/setup.bash
