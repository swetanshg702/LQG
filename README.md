ROS2 1D Altitude-Channel GNC Pipeline (Kalman Filter + LQR)

A three-node ROS2 (rclcpp) closed-loop simulation demonstrating state
estimation and LQR control of a simplified 1D quadrotor altitude channel.

The project began as a discrete-time double-integrator testbed and has now
been extended to use a physically meaningful vertical thrust model:

[
m\ddot z = T - mg
]

The three ROS2 nodes are:

plant_node: simulates the true vertical dynamics and noisy measurements

estimator_node: estimates position and velocity using a Kalman filter

controller_node: computes LQR feedback, adds hover trim thrust, and
applies actuator saturation

This is still a simplified 1D subsystem, not a full quadrotor model. The
purpose is to validate the ROS2 estimation/control architecture and introduce
realistic aerospace control concepts incrementally before moving to attitude
dynamics and eventually Gazebo.

System

State:

x = [position, velocity]^T

The vertical dynamics are:

m*z_ddot = T - m*g

where:

T is actual thrust in newtons

m is vehicle mass

g is gravitational acceleration

With dt = 0.1 s, the discrete-time plant is represented as:

x(k+1) = A*x(k) + B*T(k) + g_vec + w(k)

B = [0.0     ]
    [dt / m  ]

g_vec = [0.0        ]
        [-g*dt      ]

The constant gravity term makes the physical model affine rather than a
purely linear A*x + B*u model.

The process noise is:

w ~ N(0, Q)

and measurement noise is:

v ~ N(0, R)

with the default values:

Q = 0.001 * I
R = 0.05 * I

Noise is injected in plant_node using Cholesky factorization so the
implementation also supports non-diagonal covariance matrices.

Kalman Filter

The estimator maintains the linear perturbation model around the hover
operating point.

For the deviation input:

u_KF = T - m*g

the existing linear Kalman filter uses:

Predict:

x_hat^- = A*x_hat + B*u_KF
P^-     = A*P*A^T + Q

Update:

y = z - C*x_hat^-
S = C*P^-*C^T + R
K = P^-*C^T*S^-1

x_hat = x_hat^- + K*y
P     = (I - K*C)*P^-

The covariance solve uses LDLT rather than explicitly forming a matrix
inverse.

This allows the physical plant to operate with actual thrust while keeping
the estimator expressed in terms of deviations from the hover equilibrium.

LQR Controller

The LQR controller regulates the deviation from the reference state:

u_fb = -K*(x_hat - x_ref)

The LQR gain K is configurable through ROS2 parameters.

Because a quadrotor requires nonzero thrust to hover, the controller adds the
trim/feedforward term:

T_trim = m*g

The raw actuator command is therefore:

T_raw = T_trim + u_fb
      = m*g + u_fb

A physical actuator cannot produce arbitrary thrust, so the final command is
saturated:

T_cmd = clamp(T_raw, 0, T_max)

The resulting control pipeline is:

estimated state
      |
      v
     LQR
      |
      v
    u_fb
      |
      +---- m*g
      |
      v
    T_raw
      |
      v
 saturation
      |
      v
    T_cmd

Architecture

                         measurement
plant_node ─────────────────────────────► estimator_node
    ▲                                           |
    |                                           | estimated_state
    |                                           v
    |                                      controller_node
    |                                           |
    |                                           | control (T)
    └───────────────────────────────────────────┘

plant_node also publishes:

    true_state
        |
        └──► offline validation only

true_state is ground truth and is never provided to either the estimator or
controller.

The nodes currently use the default single-threaded ROS2 executor.
plant_node advances the simulated system at 10 Hz using a wall timer.
estimator_node performs its prediction/update when measurements arrive,
using the most recently received thrust command.

Known limitation

The current simulation implicitly assumes approximately synchronized
10 Hz execution with no significant message loss or timing jitter.

The estimator does not independently schedule prediction using measured
timestamps. A more realistic implementation should use timestamp-based
dt handling and explicitly account for asynchronous sensor and control
timing.

Configuration

ROS2 parameters are used for the plant, estimator, and controller.

config/
├── plant_params.yaml
├── estimator_params.yaml
└── controller_params.yaml

Important parameters include:

mass
gravity
dt
max_thrust
K_gain
A
C
Q
R
P0
x0

The thrust-input matrix B is derived from dt and mass rather than being
manually specified as an independent physical parameter.

For example, the LQR gain can be overridden from the command line:

ros2 run quadrotor_gnc controller_node \
  --ros-args -p K_gain:="[3.0, 2.8]"

Build & Run

cd ~/quadrotor_ws

colcon build --symlink-install --packages-select quadrotor_gnc

source install/setup.bash

ros2 launch quadrotor_gnc gnc.launch.py

Testing

Unit tests cover the LQRController independently of ROS2:

colcon test --packages-select quadrotor_gnc

colcon test-result --verbose

Validation

The repository includes scripts/analyze_bag.py for offline comparison of
the estimated state against the plant's ground truth.

A validation run can be recorded with:

ros2 bag record -o long_run_bag \
  /true_state \
  /estimated_state \
  /control

and analyzed with:

cd ~/quadrotor_ws/scripts
python3 analyze_bag.py ../long_run_bag

The recorded true_state is ground truth for validation only; it is never
used by the controller or Kalman filter.

Results



Phase 1 was validated against ground truth over a 78.3 s ROS2 bag containing
724 matched state samples.

The test includes the physical thrust/gravity plant, hover trim/feedforward,
thrust saturation, and the corresponding Kalman-filter input transformation.

Estimation error

Window

Position RMSE

Position mean error

Velocity RMSE

Velocity mean error

Transient (t < 5 s)

0.0982

0.0463

0.0755

0.0234

Steady-state (t >= 5 s)

0.0872

0.0028

0.0736

0.0026

The steady-state mean errors are close to zero, while the estimated position
and velocity track the true state throughout the validation run without
divergence.

The transient error is larger because the estimator starts from an initial
state that differs from the plant's initial state. The estimator then
converges as measurements are incorporated.

The validation demonstrates consistency between:

physical plant:
m*z_ddot = T - mg

controller:
T = mg + u_fb

estimator:
u_KF = T - mg

Repository Structure

quadrotor_gnc/
├── include/quadrotor_gnc/     # kalman_filter.hpp, lqr_controller.hpp
├── src/                       # ROS2 node implementations
├── launch/                    # gnc.launch.py
├── config/                    # ROS2 parameter YAML files
├── msg/                       # Measurement, EstimatedState, Control
├── test/                      # gtest for LQRController
├── scripts/                   # analyze_bag.py
├── kf_results.png             # latest validation plot
├── CMakeLists.txt
├── package.xml
├── LICENSE
└── .gitignore

Next Steps

Add a second decoupled attitude channel for roll/pitch and validate the
combined altitude + attitude architecture with ROS2 bags and quantitative
metrics.

Integrate the controller with Gazebo using ros_gz_bridge, replacing
plant_node with an actual simulated quadrotor.

Replace the simplified decoupled model with a full 6DOF nonlinear
quadrotor model.

Develop an EKF for nonlinear state estimation.

Build a cascaded position → attitude → motor-mixer control architecture.