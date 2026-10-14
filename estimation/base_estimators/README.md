# IEKF joint and contact mappings

`robot_IEKF` supports a free-flyer root and scalar non-base joints (`nq == nv == 1`).
Continuous joints represented by two configuration coordinates, spherical joints,
and other multi-DOF joints are rejected. All non-base movable joints in the supplied
model must appear exactly once in `robot_params::joint_names_`. Supply a reduced
model if some joints are intentionally excluded.

Joint names may be interleaved or reordered. Contact frames may lie below fixed
links, have different supporting chain lengths, or share actuated ancestors.
Their supporting joints come from the Pinocchio parent tree; numeric Jacobian
zeros are not used to determine chain membership.

## Input and output ordering

- `robot_store::joint_states_position_`, `joint_states_velocity_`, and
  `joint_states_effort_` retain **compact Pinocchio model order**, excluding the
  floating base. They are not in `joint_names_` order. Efforts are required only
  with momentum-based contact detection enabled.
- Encoder noise vectors use **configured `joint_names_` order**.
- Contact flags and the public `contact_` / `contact_torque_` vectors use
  **configured `contact_names_` order**. Pair frame names with their sensor flags
  explicitly in the calling adapter. The core never sorts or mutates these names.
- Model, data, and parameters must remain valid and unchanged for the estimator's
  lifetime. Reconstruct the estimator when changing topology or configuration.

The core caches the actual configuration and velocity indices for each joint,
builds each contact's translational Jacobian over all configured joints, and maps
model-ordered sensor samples into that ordering. Unrelated branch columns remain
zero. No contiguous leg blocks or terminal-joint ordering are required.

This change does not modify the Tron1 adapter. Its existing joint/contact
permutations and supplied sensor ordering remain its responsibility. In particular,
the core preserves whichever contact order the adapter supplies after its own
preprocessing.

## Encoder noise compatibility

`joint_position_std_` and `joint_velocity_std_` accept:

1. One standard deviation, broadcast to all configured joints.
2. One standard deviation per configured joint.
3. Legacy `dim_legs_` entries, expanded along each root-to-contact joint chain,
   provided those chains are equally sized, disjoint, and cover all measured joints.

Per-joint interpretation takes precedence when its length matches. Legacy arrays
are rejected for unequal chains, shared ancestors, or uncovered arm/other joints;
use scalar or per-joint noise in those cases. Existing Tron1 four-entry per-leg
noise arrays remain supported for its two independent four-joint chains.

`num_legs_` is optional (zero); when positive, it must match `contact_names_.size()`.
`dim_legs_` is optional (zero) and is used only for legacy noise interpretation.
Noise must be finite and nonnegative, rate/history positive, and inputs correctly
sized and finite. Invalid samples are rejected before filter propagation.

## Measurement covariance and momentum contacts

Position covariance is `J C_q J^T`. Velocity measurement remains
`-J q_dot - omega x p`, with covariance
`J C_v J^T + (skew(omega) J) C_q (skew(omega) J)^T + skew(p) C_gyro skew(p)^T`.
This preserves the existing approximation that excludes `dJ/dq * q_dot`.
The previous hardcoded covariance offsets assumed three joints per leg; correcting
them can change estimates and require noise retuning on four-joint legs.

Momentum observation runs only with `using_mob_` enabled. Candidate contact forces
are solved together from `r = [J_1^T ... J_m^T] f` with damping `0.01`, then compared
against the existing vertical-force threshold. Disjoint chains reproduce the
independent damped solves; shared actuated ancestors couple the equations.
Regularization yields a solution but does not guarantee individually observable
forces. The observer's existing dynamics/integration formula is unchanged.

The existing IMU conventions, filter prediction/correction algorithms, and ground
height postprocessing are unchanged. Contacts are still assumed to be ground
contacts by that postprocessing; this is not a general wall/hand-contact estimator.
Validate covariance changes using simulation or replay before hardware deployment.

## Verification

```bash
colcon build --packages-select base_estimators --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select base_estimators --ctest-args -R test_iekf --output-on-failure
colcon test-result --verbose
```

The tests construct synthetic robot trees, permute both model/configuration order,
check finite-difference Jacobians and contact velocities, verify covariance/force
propagation independently, and exercise legacy noise, malformed inputs, and
optional momentum observation with repeated estimator reconstruction.
