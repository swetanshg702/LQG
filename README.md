# ROS2 Linear State Estimation & Control Pipeline (Kalman Filter + LQR)

A three-node ROS2 (rclcpp) pipeline demonstrating closed-loop state estimation
and control on a discrete-time linear system: a plant, a Kalman filter
estimator, and an LQR controller communicating over topics.

**Scope note:** this models a 1D double-integrator (position/velocity, single
control input) — not full quadrotor dynamics. It's a clean, validated testbed
for the ROS2 architecture and the estimation/control theory, built to be
extended toward an actual quadrotor subsystem (see "Next steps").

## System

State: `x = [position, velocity]ᵀ`. Discrete-time model (dt = 0.1 s):

```
x(k+1) = A x(k) + B u(k) + w(k)      w ~ N(0, Q)
z(k)   = x(k) + v(k)                 v ~ N(0, R)
```

```
A = [1.0  0.1]      B = [0.0]      C = [1  0]
    [0.0  1.0]          [0.1]          [0  1]
```

Process noise `Q = 0.001·I` and measurement noise `R = 0.05·I` by default,
injected in `plant_node` and correlated via Cholesky factorization to support
non-diagonal covariances if extended later.

### Kalman filter (predict/update)

```
Predict:  x̂⁻ = A x̂ + B u          P⁻ = A P Aᵀ + Q
Update:   y = z − C x̂⁻            S = C P⁻ Cᵀ + R
          K = P⁻ Cᵀ S⁻¹             (solved via LDLT, not an explicit inverse)
          x̂ = x̂⁻ + K y            P = (I − K C) P⁻
```

### LQR controller

Static-gain regulator `u = −K(x̂ − x_ref)`, gain `K` configurable via ROS2
parameters rather than hardcoded.

## Architecture

```
plant_node ──(measurement)──► estimator_node ──(estimated_state)──► controller_node
     │                                                                     │
     ├──(true_state, ground truth only)                                   │
     ▲                                                                     │
     └──────────────────────────(control)────────────────────────────────┘
```

`true_state` is published purely for offline validation (logging/plotting) —
it is never subscribed to by `estimator_node` or `controller_node`.

Each node runs on the default single-threaded executor; `plant_node` steps
the true state on a 10 Hz wall timer, `estimator_node` predicts/updates on
each incoming measurement using the most recently received control input.

**Known limitation:** the loop implicitly assumes all three nodes run in
lockstep at 10 Hz with no message loss or jitter — there's no independent
timer driving the KF predict step. Fine for this simulation; would need
explicit timestamp-based `dt` handling for anything less deterministic.

## Configuration

All system matrices (`A`, `B`, `C`, `Q`, `R`, `P0`, `x0`) and the LQR gain
(`K_gain`) are exposed as ROS2 parameters, loaded from YAML at launch time —
nothing is hardcoded or requires recompiling to retune:

```
config/
├── plant_params.yaml
├── estimator_params.yaml
└── controller_params.yaml
```

Override from the CLI without touching the YAML, e.g.:
```bash
ros2 run quadrotor_gnc controller_node --ros-args -p K_gain:="[3.0, 2.8]"
```

## Build & run

```bash
cd ~/quadrotor_ws
colcon build --packages-select quadrotor_gnc
source install/setup.bash
ros2 launch quadrotor_gnc gnc.launch.py
```

## Testing

Unit tests cover `LQRController` in isolation (no ROS dependency):
```bash
colcon test --packages-select quadrotor_gnc
colcon test-result --verbose
```

## Results

![True vs. estimated state](kf_results.png)

Validated against ground truth (`true_state`, never seen by the filter)
over a 116 s run, split into an early transient window and steady-state:

| Window | Position RMSE | Position mean error | Velocity RMSE | Velocity mean error |
|---|---|---|---|---|
| Transient (t < 5 s) | 0.075 | 0.037 | 0.049 | 0.013 |
| Steady-state (t ≥ 5 s) | 0.093 | 0.007 | 0.078 | -0.009 |

Steady-state mean error is under 10% of the standard deviation for both
states — consistent with an unbiased filter, not a systematic offset. The
transient's larger initial error reflects the deliberate mismatch between
the filter's initial estimate (`x0 = (4.2, 0.7)`) and the plant's true
initial state (`(5.0, 0.0)`), and decays within a few seconds as expected
for a directly-observed 2-state system.

**Observed limitation:** with the default `Q`/`R` ratio, the filter visibly
under-responds to fast, large excursions in the true state (clipped peak
amplitude, some lag) while tracking slower variation well — a direct
consequence of trusting the process model more than each individual
measurement. Retuning `Q` relative to `R` in `estimator_params.yaml` trades
off this responsiveness against measurement-noise rejection; rerun the bag
capture and `analyze_bag.py` (see `scripts/`) after any change to verify.

## Repo structure

```
quadrotor_gnc/
├── include/quadrotor_gnc/     # kalman_filter.hpp, lqr_controller.hpp
├── src/                       # node + library implementations
├── launch/                    # gnc.launch.py
├── config/                    # YAML params (A, B, C, Q, R, P0, x0, K_gain)
├── msg/                       # Measurement, EstimatedState, Control
├── test/                      # gtest for LQRController
├── scripts/                   # analyze_bag.py: RMSE + true-vs-estimate plot
├── CMakeLists.txt
├── package.xml
├── LICENSE
└── .gitignore
```

## Next steps

- Extend the plant to a quadrotor altitude channel (thrust-to-acceleration
  with gravity, linearized about hover) — the next credible step up from a
  generic double integrator, and the current planned direction for this repo.
- Add a second, decoupled attitude channel (roll/pitch) once the altitude
  channel is validated the same way as this one.
- Integrate with Gazebo: bridge sensor/actuator topics via `ros_gz_bridge`,
  then replace `plant_node` with an actual simulated quadrotor as the plant.
- Longer-term: move from decoupled linear channels to full 6DOF nonlinear
  dynamics with an EKF and a cascaded position → attitude → motor-mixer
  controller.
