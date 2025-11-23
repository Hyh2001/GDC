

## Installation
### Installation of onnxruntime
1. Download the latest release at `https://github.com/microsoft/onnxruntime/releases` for your corresponding architecture. 
2. Extract all the files to a folder `onnxruntime` and move the folder to `/usr/local`
3. Set an environment variable:
  - Temporarily: `export ONNXRUNTIME_ROOT_PATH=/usr/local/onnxruntime`
  - Permanently: 
  ```bash
  echo 'export ONNXRUNTIME_ROOT_PATH=/usr/local/onnxruntime' >> ~/.bashrc source ~/.bashrc
  ```

## Troubleshooting: plugin not found
If ros2_control/controller_manager reports "Loader for controller ... not found" or a plugin cannot be located after building:
- Clean the package CMake cache and rebuild the package that provides the plugin:
  ```bash
  colcon build --packages-select <package_name> --cmake-clean-cache
  source install/setup.bash
  ```