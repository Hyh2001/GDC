# General Deployment Codebase
This repository is dedicated to the deployment of planning, control, and estimation algorithms for a wide range of robotic platforms, including **manipulators**, **quadrupeds**, and **humanoids** systems. 

It provides modular and extensible tools to facilitate the integration and deployment of advanced robotics algorithms including both **model-based** and **learning-based** methods in **sim2sim** and **sim2real** scenarios, enabling seamless and rapid transitions from simulation to real-world applications.

# Installation

This pipeline is built on ROS2 Jazzy. Two recommended methods are available for installing ROS2 Jazzy:

## Standard 
This method is ideal for pure C++ users, where the control pipeline does not require specific Python packages that should be excluded from the base environment (such as PyTorch, JAX, etc.). Since policies are inferred through ONNX Runtime, pure policy-based approaches are also supported in this pipeline.

To install using this method, please refer to the official [ROS2 Jazzy installation guide]().

## Robotstack
This method is recommended for Python users who need to implement modules that depend on specific Python packages. Typical use cases include continual learning, where learning libraries are required, and sampling-based predictive control algorithms, where simulation environments are parallelized on the GPU.

To install the 