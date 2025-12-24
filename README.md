# General Deployment Codebase
This repository is dedicated to the deployment of planning, control, and estimation algorithms for a wide range of robotic platforms, including **manipulators**, **quadrupeds**, and **humanoids** systems. 

It provides modular and extensible tools to facilitate the integration and deployment of advanced robotics algorithms including both **model-based** and **learning-based** methods in **sim2sim** and **sim2real** scenarios, enabling seamless and rapid transitions from simulation to real-world applications.

# Installation

This pipeline is built on ROS2 Jazzy. Two recommended methods are available for installing ROS2 Jazzy:

## Standard 
This method is ideal for pure C++ users, where the control pipeline does not require specific Python packages that should be excluded from the base environment (such as PyTorch, JAX, etc.). Since policies are inferred through ONNX Runtime, pure policy-based approaches are also supported in this pipeline.

To install using this method, please refer to the official [ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#ubuntu-deb-packages).

## Robotstack
This method is recommended for Python users who need to implement modules that depend on specific Python packages. Typical use cases include continual learning, where learning libraries are required, and sampling-based predictive control algorithms, where simulation environments are parallelized on the GPU.

To install using this method, checkout the installation guideline of [RoboStack](https://robostack.github.io/GettingStarted.html). 

## Install This Project
To install this project, set up a standard ROS2 workspace:
```bash
mkdir -p ~/<workspace_name>_ws/src
```

Then clone the project and initialize only the required submodules:
```bash
cd ~/<workspace_name>_ws/src
git clone https://github.com/Hyh2001/deployment_code_base.git
cd deployment_code_base
git submodule update --init --recursive descriptions command/keyboard_joy
cd ../../
```

Install the project dependencies using `rosdep`:
```bash
rosdep install --from-paths src --ignore-src -r -y
```
**Note:** If ROS2 is installed through RoboStack, make sure to activate the virtual environment before installation.

Then build the necessary packages:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo \
interfaces common_msgs base_controllers scene_description base_estimators base_planners base_hardware_interfaces mujoco_sim loggers fsm keyboard_joy rl_utils
```