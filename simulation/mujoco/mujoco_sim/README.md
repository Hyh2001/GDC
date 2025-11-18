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

1. Create plugin files in `plugins/`: `<your_name>_sim_node.hpp` and `<your_name>_sim_node.cpp`.
2. Register the class in `config/mjcf_mappings.yaml` (add the nodes -> class mapping).
3. Declare the plugin in `mujoco_sim_plugins.xml`. 

After implementing the plugin, build and source the workspace as release version with debug info:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-select mujoco_sim
source install/setup.bash
```

Then launch the simulator via the provided launch file and configurations (see `launch/` for details):
```bash
ros2 launch mujoco_sim sim_launch.py robot_type:=pendulum

```

