

## Installation
### Installation of onnxruntime
1. Download the latest release at `https://github.com/microsoft/onnxruntime/releases` for your corresponding architecture.
2. Extract the downloaded archive to a temporary location
3. Install to system paths:
   ```bash
   # Extract (adjust filename as needed)
   tar -xzf onnxruntime-linux-x64-*.tgz
   cd onnxruntime-linux-x64-*

   # Install to system directories
   sudo cp -r include/* /usr/local/include/
   sudo cp -r lib/* /usr/local/lib/

   # Update linker cache
   sudo ldconfig

   # Verify installation
   ldconfig -p | grep onnxruntime
  ```

## Troubleshooting: plugin not found
If ros2_control/controller_manager reports "Loader for controller ... not found" or a plugin cannot be located after building:
- Clean the package CMake cache and rebuild the package that provides the plugin:
  ```bash
  colcon build --packages-select <package_name> --cmake-clean-cache
  source install/setup.bash
  ```
