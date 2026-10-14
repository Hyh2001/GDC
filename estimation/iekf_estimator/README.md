# iekf_estimator

Standalone IEKF estimation and an abstract Jazzy ros2_control base, using C++17.
Existing `base_estimators` and robot deployments are unchanged. No controller
plugin is registered by this package.

```cmake
find_package(iekf_estimator REQUIRED)
target_link_libraries(my_controller PUBLIC iekf_estimator::controller_base)
# For a non-controller consumer:
target_link_libraries(my_estimator PUBLIC iekf_estimator::estimator)
```

The core header is `iekf_estimator/IEKF.hpp`. Existing names `robot_IEKF`,
`robot_params`, and `robot_store` are preserved inside `iekf_estimator`.
The extracted InEKF helpers live in `iekf_estimator::inekf`, and the momentum
observer in `iekf_estimator`, avoiding collisions with the original library.
Original InEKF author notices are retained. The source package has no resolved
license declaration; this extraction does not assign a new license to that code.

## Controller base

Inherit directly from `iekf_estimator::IEKFEstimator`, declared in
`iekf_estimator/iekf_estimator.hpp`. Implement
`on_estimator_configure(const rclcpp_lifecycle::State&)`. Common parameters,
model, input buffers and core are ready when this hook runs. Use it for
robot-specific setup; do not mutate the common configuration in hooks.

Optional hooks are `on_estimator_init()`, `on_estimator_activate(state)`,
`on_estimator_deactivate(state)`, `on_estimator_cleanup(state)` and
`prepare_estimator_inputs(time, period)`. The latter runs after hardware inputs
have been converted to core conventions and before initialization/update. It can
apply a robot-specific orientation alignment using a safe subscriber handoff.
Subscriber ownership and synchronization belong to the derived class.

The common lifecycle methods and update pipeline are final. The base claims no
hardware command interfaces and exports no writable references. It exports
read-only state interfaces usable by downstream ros2_control controllers,
including when chained mode is enabled. See `examples/example_controller.cpp`.
Concrete robot packages own their pluginlib registration and plugin XML.

## Parameters

Parameters are declared at init and read at configure. Changes require cleanup
and reconfiguration; they are not polled during the update loop.

| Parameter | Meaning / default |
| --- | --- |
| `model.path` | Absolute URDF path, or path relative to `model.package`; empty uses the controller manager's robot description |
| `model.package` | Package share directory for a relative URDF path |
| `joint_names` | Required model joint names; arbitrary order, covering all non-base movable joints |
| `contact_names` | Required contact frame names in desired output order |
| `interfaces.joint_names` | Hardware resource names in `joint_names` order; empty uses those joint names |
| `interfaces.imu` | IMU resource name, default `imu` |
| `interfaces.contacts` | Full state interface names in `contact_names` order; required with external contacts, empty with MOB |
| `using_lo_p`, `using_lo_v` | Boolean position/velocity contact correction flags, both default true |
| `using_mob` | Boolean momentum-based contact detection flag, default false |
| `contact_effort_threshold` | Nonnegative vertical contact force threshold, default 10 |
| `ground_height_offset` | Finite height adjustment in metres after planar contact correction, default 0.05 to preserve the original core |
| `N` | Positive core measurement history length, default 1 |
| `num_legs`, `dim_legs` | Optional legacy count / noise compatibility settings, default 0 |
| `visualize` | Core console diagnostics, default false |
| `q_body_imu` | Quaternion **wxyz**, body-from-IMU rotation, default identity |
| `p_body_imu` | Body-origin to IMU-origin offset expressed in body axes, default zero |

The integration interval is fixed to `1 / get_update_rate()`, using ros2_control's
controller update rate. One core update runs per controller update. The incoming
period must be positive; it does not change the inherited core's fixed interval.

Noise parameters retain the core names: `p_init_std`, `v_init_std`,
`orientation_init_std`, `foot_init_std`, `accel_bias_init_std`,
`angular_bias_init_std`, `accel_input_std`, `gyro_input_std`,
`accel_bias_process_std`, `angular_bias_process_std`, `joint_position_std`,
`joint_velocity_std`, `foot_slide_std`, `foot_swing_std`.
All values must be finite and nonnegative. Vector noises require three elements,
except encoder noises: one value broadcasts, one per configured joint follows
`joint_names`, and legacy `dim_legs` values expand along equally sized disjoint
contact chains that cover every measured joint. Shared chains or unrelated arm
joints require scalar or per-joint encoder noise. Defaults are illustrative;
tune noise and contact detection for the robot.

Example derived-controller parameters (model description supplied by the manager):

```yaml
robot_estimator:
  ros__parameters:
    joint_names: [right_joint, arm_joint, left_joint]
    contact_names: [right_foot, left_foot]
    interfaces:
      joint_names: [robot/right, robot/arm, robot/left]
      imu: robot/imu
      contacts: [right_sensor/state, left_sensor/state]
    using_mob: false
    joint_position_std: [0.01, 0.02, 0.03]
    joint_velocity_std: [0.1]
```

## Inputs and exported state

Joint inputs require `position` and `velocity`, plus `effort` only with MOB.
IMU inputs use standard `orientation.w/x/y/z`, `angular_velocity.x/y/z` and
`linear_acceleration.x/y/z` names under `interfaces.imu`. Quaternion input must
be finite and nonzero. External contact flags must be exactly 0 or 1.
Assigned interface order is irrelevant: binding uses full names at activation.
The base rotates IMU acceleration and gyro into body axes and converts
world-from-IMU orientation to world-from-body. Joint input vectors are scattered
into compact Pinocchio model order before calling the core.

All exports have the controller instance name as prefix:

| Suffix | Value |
| --- | --- |
| `status/valid` | 1 after successful publication, otherwise 0 |
| `status/sample_time` | Successful input sample timestamp in seconds |
| `base/position.x/y/z` | Base-origin position in the world frame |
| `base/orientation.w/x/y/z` | World-from-body unit quaternion, continuous sign while valid |
| `base/linear_velocity.x/y/z` | Base-origin linear velocity in body axes |
| `base/angular_velocity.x/y/z` | Bias-corrected gyro in body axes |
| `imu/gyroscope_bias.x/y/z` | Estimated bias in body axes |
| `imu/accelerometer_bias.x/y/z` | Estimated bias in body axes |
| `joints/<model_joint>/position`, `/velocity` | Measured joint state, not a filtered estimate |
| `joints/<model_joint>/effort` | Measured effort, only with MOB |
| `contacts/<contact_frame>/state` | External or MOB-derived contact flag |

For example, a downstream controller requests
`robot_estimator/base/linear_velocity.x` as a state interface. Derived classes
may extend `on_export_state_interfaces()` by calling the base and appending
interfaces backed by their own storage.

Activation resets filter/history/MOB state without resizing exported storage.
The first update reads current measurements and initializes the core. A failed
read, hook, estimation or nonfinite estimate returns ERROR and latches failure
until reactivation. It clears `status/valid`, preserves the previous successful
estimate and timestamp, and stops subsequent core updates. Deactivation and
cleanup invalidate all values (NaN except validity). Cleanup calls the derived
hook before releasing core resources. Exported handles must be released before
reconfiguration, as with other ros2_control controllers.

## Core behavior and limits

The free-flyer root and scalar non-base joints remain required; continuous and
multi-DOF joints are rejected. Contact frame Jacobians cover every configured
joint; unrelated branches contribute zero columns. Shared ancestor contacts use
the coupled regularized force solve. That solve does not make underdetermined
contact forces uniquely observable. Velocity covariance retains the existing
approximation omitting `dJ/dq * qdot`.

This extraction preserves the existing core's numerics, including initialization
with zero position/velocity/bias, previous-sample IMU propagation, planar contact
height correction and the default **0.05 m height offset**, configurable through
`ground_height_offset`. The controller
converts the core's IMU-origin position to the base origin. No external-position
fusion, terrain model, or robot migration is added. Core allocations and
ros2_control handle access are not a hard realtime guarantee.

## Build and test

```bash
colcon build --packages-select iekf_estimator --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select iekf_estimator --event-handlers console_direct+
colcon test-result --verbose
```

Tests include the generalized core regressions, reset with/without MOB,
controller lifecycle and interface ordering, IMU extrinsics and frame mapping,
failure handling, independent instances, and a separate installed CMake consumer.
The committed URDF fixture requires no model generation during tests.
