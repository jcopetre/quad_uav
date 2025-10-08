# Quadrotor 6DOF Simulation with LQR Control

A complete MATLAB/Simulink implementation of a linearized 6 degree-of-freedom quadrotor UAV with Linear Quadratic Regulator (LQR) control and optimal trajectory generation.

## Overview

This system includes:
- **Linearized 6DOF quadrotor dynamics** (position, attitude, velocities, angular rates)
- **LQR controller** with optimal state feedback gains
- **Minimum-snap trajectory generation** through waypoints
- **Simulink simulation** with closed-loop control
- **3D visualization and animation**

## Files Included

1. **complete_setup.m** - Main setup and execution script (START HERE!)
2. **quadrotor_params_setup.m** - Parameter definition and LQR design
3. **generate_trajectory.m** - Optimal trajectory generator
4. **simulation_main.m** - Simulation runner with analysis
5. **animate_quadrotor.m** - 3D animation function
6. **Simulink_Instructions.md** - Guide for building the Simulink model

## Quick Start

### Method 1: All-in-One (Recommended)

1. **Create the Simulink model** first using the programmatic script in the instructions:
   ```matlab
   % Run the model creation code from Simulink_Instructions.md
   % This creates quadrotor_6dof_lqr.slx
   ```

2. **Add function code** to each MATLAB Function block in Simulink:
   - Double-click each MATLAB Function block
   - Copy and paste the corresponding code from the instructions

3. **Run the complete setup**:
   ```matlab
   complete_setup
   ```

This single script will:
- Set up all parameters
- Design the LQR controller
- Generate the optimal trajectory
- Run the Simulink simulation
- Plot all results
- Offer to create a 3D animation

### Method 2: Step-by-Step

```matlab
% 1. Setup parameters and LQR controller
run('quadrotor_params_setup.m');

% 2. Generate trajectory
trajectory = generate_trajectory();

% 3. Create Simulink model (see instructions)
% Build quadrotor_6dof_lqr.slx manually or programmatically

% 4. Run simulation and analyze
run('simulation_main.m');
```

## System Description

### State Vector (12 states)
```
x = [x, y, z, φ, θ, ψ, ẋ, ẏ, ż, p, q, r]ᵀ
```
Where:
- `x, y, z`: Position in inertial frame (m)
- `φ, θ, ψ`: Roll, pitch, yaw angles (rad)
- `ẋ, ẏ, ż`: Linear velocities (m/s)
- `p, q, r`: Angular velocities (rad/s)

### Control Input (4 inputs)
```
u = [F, τ_φ, τ_θ, τ_ψ]ᵀ
```
Where:
- `F`: Total thrust force (N)
- `τ_φ, τ_θ, τ_ψ`: Roll, pitch, yaw torques (N·m)

### Default Parameters
- **Mass**: 0.5 kg
- **Arm length**: 0.25 m
- **Inertia**: Ixx = Iyy = 0.0075 kg·m², Izz = 0.013 kg·m²

### Trajectory Waypoints
The default trajectory includes:
1. Takeoff from origin
2. Forward motion with climb
3. Lateral motion with yaw rotation
4. Return path with altitude change
5. Landing at origin

Total flight time: 15 seconds

## Customization

### Modify Physical Parameters
Edit values in `complete_setup.m` or `quadrotor_params_setup.m`:
```matlab
params.m = 0.5;      % Change mass
params.L = 0.25;     % Change arm length
params.Ixx = 0.0075; % Change inertia
```

### Adjust LQR Weights
Tune the Q and R matrices for different performance:
```matlab
% State weights (position, attitude, velocity, angular rate)
Q = diag([100 100 100 10 10 1 10 10 10 1 1 0.1]);

% Control weights (thrust, torques)
R = diag([0.1 1 1 1]);
```
- Increase Q → better tracking, more aggressive control
- Increase R → smoother control, slower tracking

### Create Custom Trajectories
Modify waypoints in `complete_setup.m` or `generate_trajectory.m`:
```matlab
waypoints = [
    x,  y,  z,  yaw, time;
    0,  0,  0,  0,   0;     % Start
    1,  1,  1,  0,   5;     % Custom waypoint
    % Add more waypoints...
];
```

## Simulation Results

The simulation produces:
- **3D trajectory plot** comparing reference and actual paths
- **Position tracking** for X, Y, Z coordinates
- **Attitude tracking** for roll, pitch, yaw
- **Control inputs** (thrust and torques) over time
- **Performance metrics** (RMSE, max error, control effort)
- **Optional 3D animation** of the quadrotor flight

## Performance Metrics

The system computes and displays:
- **RMSE** (Root Mean Square Error) for position and attitude
- **Maximum errors** for all states
- **Control effort** (integral of squared control inputs)
- **Closed-loop eigenvalues** (system stability)

## Troubleshooting

### "Model not found" error
- Create the Simulink model using the instructions
- Ensure the model is named `quadrotor_6dof_lqr.slx`

### "Cannot find trajectory.mat" error
- Run `generate_trajectory()` before simulation
- Or use `complete_setup.m` which handles this automatically

### Poor tracking performance
- Increase Q matrix values for states with large errors
- Check that the trajectory is physically feasible
- Verify control saturation limits are appropriate

### Simulation crashes or unstable
- Check that the LQR gains are computed correctly
- Verify initial conditions are near hover equilibrium
- Ensure time step (dt = 0.01) is small enough

## Requirements

- MATLAB R2019b or later
- Simulink
- Control System Toolbox (for LQR design)

## Theory

### Linearization
The nonlinear quadrotor dynamics are linearized around hover equilibrium:
- All states = 0
- Thrust = m·g (hover thrust)
- Small angle approximation for attitudes

### LQR Control
The control law minimizes the cost function:
```
J = ∫(xᵀQx + uᵀRu)dt
```

The optimal control is:
```
u = u_hover - K(x - x_ref)
```
where K is the LQR gain matrix.

### Trajectory Generation
Uses 5th-order polynomial interpolation between waypoints to ensure:
- C² continuity (continuous position, velocity, acceleration)
- Zero velocity/acceleration at waypoints
- Smooth, dynamically feasible paths

## Extensions

Consider adding:
- **Nonlinear dynamics model** for high-speed maneuvers
- **Disturbance rejection** (wind gusts, parameter uncertainties)
- **State estimation** (Extended Kalman Filter, observer)
- **Motor dynamics** and individual rotor control
- **Obstacle avoidance** algorithms
- **MPC (Model Predictive Control)** for constraint handling
- **Adaptive control** for varying payload

## Example Output

After running `complete_setup.m`, you should see:

```
===================================
Quadrotor 6DOF LQR Control System
===================================

Step 1/5: Setting up quadrotor parameters...
  LQR gains computed successfully
  Closed-loop poles (real part): -8.45 to -1.23

Step 2/5: Generating optimal trajectory...
  Generated 1501 trajectory points over 15.0 seconds

Step 3/5: Checking Simulink model...
  Model already loaded

Step 4/5: Running Simulink simulation...
  Simulation completed in 2.34 seconds

Step 5/5: Analyzing results and generating plots...

=== PERFORMANCE METRICS ===
Position RMSE:  X=0.0234m  Y=0.0198m  Z=0.0156m
Position Max:   X=0.0876m  Y=0.0723m  Z=0.0534m
Attitude RMSE:  φ=1.23°  θ=1.45°  ψ=0.87°
Attitude Max:   φ=3.45°  θ=4.12°  ψ=2.34°
Control effort: 145.67

=== SIMULATION COMPLETE ===
Total simulation time: 15.0 seconds
Computation time: 2.34 seconds
All results plotted successfully!
```

## References

- **LQR Control**: Anderson, B. D., & Moore, J. B. (1990). *Optimal Control: Linear Quadratic Methods*
- **Quadrotor Dynamics**: Bouabdallah, S. (2007). *Design and Control of Quadrotors*
- **Trajectory Generation**: Mellinger, D., & Kumar, V. (2011). *Minimum Snap Trajectory Generation*

## License

This code is provided for educational purposes. Feel free to modify and extend for your research or projects.

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Verify all files are in the same directory
3. Ensure MATLAB version compatibility
4. Review the Simulink model structure

## Acknowledgments

Based on standard quadrotor control literature and optimal control theory.