# Robot Descriptions

This folder contains the URDF, MJCF and USD files for robots.

Supported robot descriptions 
    * [Go2](unitree/go2_description/)

## Build you own robot descriptions
### preparation 
An URDF file of the robot you want to control with correct joint numbers and types. 

### Transfer urdf to Mujoco model
1. Install [Mujoco](https://github.com/google-deepmind/mujoco)
2. Transfer the mesh files to mujoco supported format, like stl.
3. Adjust the urdf tile to match the mesh file. Transfer the mesh file from .dae to .stl may change the scale size of the mesh file.
4. use mujoco to convert the urdf file to mujoco model.
  ```
  compile robot.urdf robot.xml
  ```
  5. Note