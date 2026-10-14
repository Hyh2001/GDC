# onnx_policy

Reusable ONNX inference, MDP observation/action managers, and an abstract
`OnnxPolicyController` for ROS 2 Jazzy (C++17).

Existing `base_controllers`, `rl_utils`, and robot deployments are unchanged.
This package contains no registered controller plugin. Concrete robot packages
inherit the base and register their own plugins.

## Build and test

Install the Jazzy controller dependencies and Eigen, then put
[onnxruntime_vendor](https://github.com/ros-controls/onnxruntime_vendor) in your
workspace's `src/` directory:

```bash
git clone https://github.com/ros-controls/onnxruntime_vendor src/onnxruntime_vendor
colcon build --packages-up-to onnx_policy --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select onnx_policy --event-handlers console_direct+
colcon test-result --verbose
```

The vendor handles the ONNX Runtime headers, library, and runtime discovery.
There is no `/usr/local` library search or manual ONNX Runtime installation.
Tests use committed deterministic ONNX fixtures; Python ONNX is not needed to
build or run them. `test/generate_fixtures.py` optionally regenerates those
fixtures for maintenance.

## Installed CMake targets

```cmake
find_package(onnx_policy REQUIRED)
target_link_libraries(my_policy PRIVATE onnx_policy::policy_runtime onnx_policy::mdp)
target_link_libraries(my_controller PRIVATE onnx_policy::controller_base)
```

Declare `<depend>onnx_policy</depend>` in the consuming package's `package.xml`.
`onnx_policy::mdp` is header-only and links Eigen, without linking ONNX Runtime.
The ROS/controller dependencies are linked only through `controller_base`.
All targets provide C++17 and their installed include paths.

## Inference

```cpp
#include <onnx_policy/onnx_policy.hpp>

onnx_policy::OnnxPolicy policy("/absolute/model.onnx", 3, 2);
std::vector<double> input{1, 2, 3}, output(2);
policy.infer_into(input, output);  // Does not resize caller-owned output.
auto another_output = policy.infer(input);
```

Input/output tensors must be float32 with shapes `[1, observation_dim]` and
`[1, action_dim]`. Symbolic dimensions are bound to the configured sizes. MLP
models have exactly one input and one output; their tensor names are discovered.

`OnnxRNNPolicyCfg` additionally supplies positive `hidden_layers` and
`hidden_dim`. Recurrent models have exactly two inputs and outputs: the primary
observation/action tensors plus `h_in` and `h_out`, both shaped
`[hidden_layers, 1, hidden_dim]`. Primary names are discovered independently of
metadata ordering. Separate LSTM cell-state tensors and other model contracts
are unsupported. `OnnxRNN::reset()` clears hidden memory; MLP reset is a no-op.
Calling `infer()` or `infer_into()` through an `OnnxPolicy` pointer preserves
recurrent behavior. Essential validation always runs, including when the
retained `validate_model_io` field is false.

Inference uses CPU execution with one intra-operation thread. A policy instance
and its managers must be used from one execution thread; they are not internally
synchronized. Tensor/history buffers are reused, but callback/convenience APIs
can allocate and ONNX Runtime is not a hard realtime guarantee.

## Observation and action managers

Headers are under `onnx_policy/mdp/observation/` and `onnx_policy/mdp/action/`.
Namespaces are `onnx_policy::observation` and `onnx_policy::action`.

Register observation terms in training-input order using
`ObservationTermCfg` and a callback returning the term's current vector.
The processing sequence is **compute → clip → scale → history**. Configuration
vectors must match `observation_dim`; absent scale means identity and absent
clipping means no clipping. Both clipping bounds must be supplied together.
History starts with zeros and runs oldest to newest. `term_order=true` (default)
concatenates each term's complete history; `false` interleaves history slot `h`
across terms, skipping terms whose history length is at most `h`. Unequal
history lengths therefore retain the original slot-based convention rather
than timestamp alignment.

`passthrough`, `joint_values` (subtract offsets), and `projected_gravity` are
provided. Gravity uses a normalized orientation quaternion; zero/nonfinite
quaternions are rejected. `compute_view()` and `observation_view()` avoid
copying manager output; the original vector-returning methods are retained.

Register action terms in training-output order. Scale and offset vectors must
match `action_dim`. Processing partitions the complete policy output, with
each term applying `action * scale + offset`. Effort terms force offsets to
zero. Position, velocity, and impedance terms retain the affine transform.
`JointImpedanceActionTerm` does not write hardware gains: the robot controller
handles gains, command interfaces, and compensation. `processed_actions_view()`
avoids copying the manager output. Reset clears history and all stored actions
to zeros, rather than applying offsets.

Views remain valid until their owner is destroyed or terms are added. Register
terms only during configuration; never resize exported reference storage while
its interfaces are registered.

## Inheriting OnnxPolicyController

See [examples/example_controller.cpp](examples/example_controller.cpp), which
is compiled during testing. It maps one measured joint position and one
exported target reference to a two-dimensional observation, then maps a
one-dimensional action to a position command.

The base owns `observation_manager_`, `action_manager_`, `policy_`,
`observations_`, `actions_`, and `processed_actions_`. Required hooks:

- `on_policy_configure(state)`: allocate robot buffers and register terms.
- `read_policy_inputs(time, period)`: read bound interfaces and prepare callback data.
- `write_policy_commands(time, period, processed_actions)`: validate and write robot commands.

Optional hooks are `on_policy_init()`, `on_policy_activate(state)`,
`on_policy_deactivate(state)`, and `on_policy_cleanup(state)`. They default to
success. Derived controllers also implement state/command interface
configuration and optionally export references. Bind interfaces by full name
during activation; do not rely on resource-manager ordering. Reference inputs
are written by upstream controllers and are not claimed as policy outputs.
Estimator/planner exported state interfaces can be claimed as observation
sources; action outputs may target hardware or downstream references.

The base declares only these inference parameters:

```yaml
my_policy:
  ros__parameters:
    model.path: policies/policy.onnx
    model.package: my_robot_description
    model.type: mlp  # Or rnn, with positive hidden_layers and hidden_dim.
    model.hidden_layers: 0
    model.hidden_dim: 0
```

Absolute paths do not require `model.package`. Relative paths resolve below the
specified package's share directory. Parameters are read during configuration;
changing them requires reconfiguration. Model dimensions come from the managers.
Robot parameters and term setup stay in the derived controller, with no generic
YAML term parser.

Configure rebuilds the managers, loads/validates the model, warms up inference,
and resets recurrent memory. Activate resets policy/history/action state before
the derived activation hook. That hook may seed `actions_` from measured robot
state and initialize references without resizing exported storage. Base lifecycle
methods and `update_and_write_commands()` are final. `update_reference_from_subscribers()`
defaults to a no-op and may be overridden for realtime-safe topic input.

Each update performs input preparation, observation computation, inference,
action processing, and command writing. Observation callbacks reading `actions_`
see the previous successful raw output (initially zero unless seeded). The new
raw output is committed only after a successful command hook. Inference runs
once per controller update; configure the standard controller update rate.

Errors/nonfinite values/exceptions latch failure until reactivation and return
`ERROR` to controller manager. No command hook runs after an earlier pipeline
failure. The command hook must validate all robot commands before writing;
the base cannot roll back hardware writes. There is no universal zero-command
fallback. Deactivate resets state without writing fallback commands. Cleanup
invokes the derived hook while policy resources still exist, then releases them.
Controller manager owns release of loaned interfaces.

Concrete plugins inherit this base directly, not alongside another ros2_control
base. Register the concrete class in its own package using pluginlib and a plugin
XML for `controller_interface::ChainableControllerInterface`; this package
intentionally exports no plugin XML.
