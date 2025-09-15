# Keyboard to Joystick Mapper

This package addresses the following tasks:
1. Reading keyboard input status using Linux input subsystem
2. Interpreting the inputs as gamepad commands
3. Converting the commands into the `sensor_msgs/msg/Joy` format and publishing them to the `/joy` topic
4. Automatically detecting the correct keyboard device

## Features

- **Automatic Keyboard Detection**: Intelligently finds and selects the main keyboard device
- **Dual Input Support**: Both WASD and Arrow keys for movement
- **Complete Gamepad Emulation**: All standard gamepad buttons and axes
- **No Dependencies**: Uses Linux input subsystem directly (no SDL required)

## Keyboard Mapping

### Joystick Axes
| Keyboard | Gamepad Function | Joy Message |
|----------|------------------|-------------|
| **W/S** or **↑/↓** | Left Stick Y-axis | axes[1] |
| **A/D** or **←/→** | Left Stick X-axis | axes[0] |
| **I/K** | Right Stick Y-axis | axes[3] |
| **J/L** | Right Stick X-axis | axes[2] |
| **Q** | Left Trigger | axes[4] |
| **E** | Right Trigger | axes[5] |

### Gamepad Buttons
| Keyboard | Gamepad Button | Joy Message | Description |
|----------|----------------|-------------|-------------|
| **Space** | A Button | buttons[0] | Primary action |
| **X** | B Button | buttons[1] | Secondary action |
| **Z** | X Button | buttons[2] | Alternative action |
| **C** | Y Button | buttons[3] | Alternative action |
| **Tab** | Left Bumper | buttons[4] | Left shoulder |
| **R** | Right Bumper | buttons[5] | Right shoulder |
| **Shift** | Back/Select | buttons[6] | Menu/back |
| **Enter** | Start | buttons[7] | Start/menu |
| **F** | Left Stick Click | buttons[8] | L3 |
| **V** | Right Stick Click | buttons[9] | R3 |

### D-Pad (Optional)
| Keyboard | D-Pad Direction | Joy Message |
|----------|-----------------|-------------|
| **1** | D-Pad Up | buttons[14] |
| **2** | D-Pad Down | buttons[15] |
| **3** | D-Pad Left | buttons[16] |
| **4** | D-Pad Right | buttons[17] |

### Extra Buttons
| Keyboard | Joy Message | Notes |
|----------|-------------|-------|
| **T** | buttons[10] | Extra button |
| **G** | buttons[11] | Extra button |
| **H** | buttons[12] | Extra button |
| **M** | buttons[13] | Extra button |
| **Ctrl** | buttons[18] | Extra button |
| **Alt** | buttons[19] | Extra button |
| **5** | buttons[20] | Extra button |

## Usage

```bash
# Build the package
colcon build --packages-select keyboard

# Source the workspace
source install/setup.bash

# Run the keyboard node
ros2 run keyboard keyboard

# Monitor joystick output (in another terminal)
ros2 topic echo /joy
```

## Technical Details

- **Device Detection**: Automatically scans `/dev/input/event*` devices
- **Capability Scoring**: Selects keyboard with highest capability score
- **Publishing Rate**: 100Hz (10ms timer)
- **Message Format**: Standard `sensor_msgs/msg/Joy` with 6 axes and 21 buttons


