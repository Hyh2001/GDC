# mujoco_sim

This package provides a standalone MuJoCo simulation implemented in C++ with convenient ROS 2 integration.

## Preparation

Before launching, ensure you have a valid MJCF model file in the corresponding description folder. Check the following:

- Sensors
  - Verify **sensor names**, associated **sites**, and **orientations** match your expectations.
- Actuators
  - Verify **actuator types**, **gains**, and **joint parameters** are correct for your controller.
- Simulation configuration
  - Verify **timestep**, **solver settings** (e.g., `solimp`, `solref` / solver parameters), and **friction** values are appropriate.

## Realtime simulation with ROS 2 interfaces
To add a ROS 2 interface (plugin) for realtime simulation:

1. Write a simulation node inherited from `MujocoSimNodeBase` and declare it as a plugin by adding the following declaration at the end of the `.cpp` file
```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(<NamespaceName>::<NodeName>, mujoco_sim::MujocoSimNodeBase)
```
2. Declare the plugin and export the plugin config file in your `CMakeLists.txt`:
```CMake
pluginlib_export_plugin_description_file(mujoco_sim <plugin_file>.xml)
```
A detailed demonstration of how to use ROS2 plugins can be found in [ROS2 pluginlib tutorial](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwj3up6nrKeRAxVkVjUKHfLrFgQQFnoECCIQAQ&url=https%3A%2F%2Fdocs.ros.org%2Fen%2Ffoxy%2FTutorials%2FBeginner-Client-Libraries%2FPluginlib.html&usg=AOvVaw1wf_-I170_Do_gldBjt3cw&opi=89978449)

After implementing the plugin, build and source the workspace as release version with debug info:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-select mujoco_sim
source install/setup.bash
```

Then launch the simulator via the provided launch file and configurations (see `launch/` for details):
```bash
ros2 launch mujoco_sim sim_launch.py robot_type:=pendulum
```

