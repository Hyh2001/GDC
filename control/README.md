




## Troubleshooting: plugin not found
If ros2_control/controller_manager reports "Loader for controller ... not found" or a plugin cannot be located after building:
- Clean the package CMake cache and rebuild the package that provides the plugin:
  ```bash
  colcon build --packages-select <package_name> --cmake-clean-cache
  source install/setup.bash