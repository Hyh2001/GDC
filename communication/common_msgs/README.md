# Common Messages Package

This ROS2 package provides common message definitions for robotic systems, particularly focused on motor control, sensor feedback, and force/torque sensing applications.

## Overview

The `common_msgs` package contains standardized message types that facilitate communication between different components in robotic systems. These messages are designed to be reusable across various robotic platforms and applications.

## Message Types

### Motor Control Messages

#### `MotorCmd.msg`
Command message for motor control with position, velocity, and torque control modes.

```
uint8 mode      # Control mode (position, velocity, torque, etc.)
float32 q       # Target position (rad)
float32 dq      # Target velocity (rad/s)
float32 tau     # Target torque (N⋅m)
float32 kp      # Proportional gain for position control
float32 kd      # Derivative gain for velocity control
```

**Usage Example:**
```cpp
common_msgs::msg::MotorCmd cmd;
cmd.mode = 1;           // Position control mode
cmd.q = 1.57;           // Target position (90 degrees)
cmd.dq = 0.0;           // Target velocity
cmd.tau = 0.0;          // Target torque
cmd.kp = 100.0;         // Position gain
cmd.kd = 10.0;          // Velocity gain
```

#### `MotorState.msg`
Feedback message containing current motor state information.

```
uint8 mode      # Current control mode
float32 q       # Current position (rad)
float32 dq      # Current velocity (rad/s)
float32 ddq     # Current acceleration (rad/s²)
float32 tau     # Current torque (N⋅m)
```

**Usage Example:**
```cpp
common_msgs::msg::MotorState state;
// state.mode, state.q, state.dq, state.ddq, state.tau
// are populated by motor driver/controller
```

### Sensor Messages

#### `ContactSensor.msg`
Simple binary contact sensor for detecting physical contact or collision.

```
bool contact    # True if contact is detected, false otherwise
```

**Usage Example:**
```cpp
common_msgs::msg::ContactSensor contact_msg;
contact_msg.contact = true;  // Contact detected
```

#### `ForceSensor.msg`
3-axis force sensor measurement.

```
geometry_msgs/Vector3 force  # Force vector [Fx, Fy, Fz] in Newtons
```

**Usage Example:**
```cpp
common_msgs::msg::ForceSensor force_msg;
force_msg.force.x = 10.5;    // Force in X direction (N)
force_msg.force.y = -2.3;    // Force in Y direction (N)
force_msg.force.z = 15.8;    // Force in Z direction (N)
```

#### `FTSensor.msg`
6-axis force/torque sensor measurement using standard geometry_msgs.

```
geometry_msgs/Wrench wrench  # Combined force and torque measurement
```

**Usage Example:**
```cpp
common_msgs::msg::FTSensor ft_msg;
// Force components
ft_msg.wrench.force.x = 10.0;   // Fx (N)
ft_msg.wrench.force.y = 5.0;    // Fy (N)
ft_msg.wrench.force.z = 20.0;   // Fz (N)
// Torque components
ft_msg.wrench.torque.x = 1.5;   // Tx (N⋅m)
ft_msg.wrench.torque.y = 0.8;   // Ty (N⋅m)
ft_msg.wrench.torque.z = 2.1;   // Tz (N⋅m)
```

## Control Modes

### Motor Control Modes
The `mode` field in motor messages typically follows these conventions:

| Mode | Value | Description |
|------|-------|-------------|
| **Free Mode** | 0 | No control to the motor |
| **Position Control** | 1 | Control motor to target position using PD control |
| **Velocity Control** | 2 | Control motor to target velocity |
| **Torque Control** | 3 | Direct torque control |
| **Torque Feedforward + Position and Velocity Feedback** | 4 | MIT actuator like torque control with feedback |
| **Actuator Network**| 5 | Control with a neural network based actuator model that takes in $q_{\text{des}},\dot{q}_{\text{des}}$ and output $\tau$ |

**Note**: not all the `mode`s are supported for all the robots, double check the simulation node implementation before calling.

## Building the Package

```bash
# Navigate to your ROS2 workspace
cd ~/your_ros2_workspace

# Build the messages package
colcon build --packages-select common_msgs

# Source the workspace
source install/setup.bash
```

## Usage in Other Packages

### CMakeLists.txt
Add the following to your package's `CMakeLists.txt`:

```cmake
find_package(common_msgs REQUIRED)

# For C++ nodes
ament_target_dependencies(your_node
  common_msgs
)
```

### package.xml
Add the dependency to your `package.xml`:

```xml
<depend>common_msgs</depend>
```
