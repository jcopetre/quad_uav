# Getting Started - Quadrotor 6DOF LQR Control System

## 🚀 Quick Start (5 Minutes)

### Step 1: Download All Files
Save these MATLAB files to a single folder:
- `build_simulink_model.m`
- `complete_setup.m`
- `generate_trajectory.m`
- `animate_quadrotor.m`

### Step 2: Build the Simulink Model
Open MATLAB, navigate to your folder, and run:
```matlab
build_simulink_model()
```

This creates `quadrotor_6dof_lqr.slx` automatically with all necessary blocks and code.

### Step 3: Run the Complete Simulation
```matlab
complete_setup
```

That's it! The script will:
1. ✅ Design the LQR controller
2. ✅ Generate an optimal trajectory
3. ✅ Simulate the quadrotor flight
4. ✅ Display performance metrics
5. ✅ Create comprehensive plots
6. ✅ Offer a 3D animation

---

## 📊 What You'll Get

### Performance Metrics
- Position tracking errors (RMSE and maximum)
- Attitude tracking errors
- Control effort analysis
- System stability information

### Visualizations
1. **3D trajectory comparison** - Reference vs. Actual path
2. **Position tracking plots** - X, Y, Z over time
3. **Attitude tracking plots** - Roll, pitch, yaw over time
4. **Control inputs** - Thrust and torques
5. **Optional 3D animation** - Real-time quadrotor visualization

---

## 🎯 Understanding the System

### The Quadrotor Model
- **12 states**: Position (x,y,z), Attitude (φ,θ,ψ), Velocities (ẋ,ẏ,ż), Angular rates (p,q,r)
- **4 control inputs**: Total thrust and 3 torques
- **Linearized dynamics**: Valid for small angles and velocities near hover

### The LQR Controller
- Optimal state feedback control
- Balances tracking performance vs. control effort
- Guaranteed stability for the linear system

### The Trajectory
Default path includes:
- Smooth takeoff
- Forward and lateral motion
- Coordinated turns
- Altitude changes
- Precision landing

Total duration: 15 seconds

---

## 🔧 Customization Options

### Change Physical Parameters
In `complete_setup.m`, modify:
```matlab
params.m = 0.5;      % Mass (kg) - try 0.3 to 2.0
params.L = 0.25;     % Arm length (m)
params.Ixx = 0.0075; % Inertia (kg·m²)
```

### Tune Controller Performance
Adjust Q and R matrices:
```matlab
% More aggressive tracking (higher Q)
Q = diag([200 200 200 20 20 2 20 20 20 2 2 0.2]);

% Smoother control (higher R)
R = diag([0.5 2 2 2]);
```

**Effect of changing weights:**
- ↑ Q = Better tracking, more aggressive
- ↑ R = Smoother control, slower response
- Balance them for your application

### Create Custom Trajectories
In `complete_setup.m`, edit the waypoints:
```matlab
waypoints = [
    % x    y    z    yaw  time
    0,   0,   0,   0,   0;      % Start
    3,   0,   2,   0,   4;      % Your waypoint 1
    3,   3,   2,   pi/2, 8;     % Your waypoint 2
    0,   3,   1,   pi,   12;    % Your waypoint 3
    0,   0,   0,   0,   16;     % Land
];
```

---

## 📁 File Structure

```
quadrotor_project/
├── build_simulink_model.m      [Run this FIRST]
├── complete_setup.m             [Then run this]
├── generate_trajectory.m        [Called automatically]
├── animate_quadrotor.m          [Called if requested]
├── quadrotor_6dof_lqr.slx      [Auto-generated]
├── quadrotor_params.mat         [Auto-generated]
└── trajectory.mat               [Auto-generated]
```

---

## 🐛 Troubleshooting

### "Cannot find build_simulink_model"
Make sure all files are in your current MATLAB directory. Check with `pwd` and `ls`.

### Model doesn't simulate
1. Ensure `quadrotor_params.mat` and `trajectory.mat` exist
2. Try running `complete_setup` which handles everything
3. Check MATLAB version (needs R2019b+)

### Poor tracking performance
**Problem**: Large position or attitude errors

**Solutions:**
- Increase Q matrix values for problem states
- Check if trajectory is too aggressive (reduce waypoint velocities)
- Verify physical parameters are reasonable

### Controls saturating
**Problem**: Thrust hitting limits (0 or 15 N)

**Solutions:**
- Make trajectory less aggressive (slower, smaller accelerations)
- Increase maximum thrust limit in LQR controller
- Add waypoints to make path smoother

### Animation is slow
Set higher speedup factor in `animate_quadrotor.m`:
```matlab
speedup = 4;  % Default is 2
```

---

## 📚 Understanding the Results

### Good Performance Indicators
- Position RMSE < 0.05 m
- Attitude RMSE < 2°
- Smooth control inputs (no chattering)
- Stable closed-loop poles (all negative real parts)

### What the Plots Show

**Top Row**: 3D path and X,Y,Z position tracking
- Blue dashed line = desired trajectory
- Red solid line = actual flight path
- Look for close overlap

**Middle Row**: Roll, pitch, yaw tracking
- Angles should track reference with small errors
- Some lag is normal due to dynamics

**Bottom Row**: Control efforts
- Thrust should hover around 4.9 N (m·g)
- Torques should be smooth, not oscillating
- Large spikes indicate aggressive maneuvers

---

## 🎓 Learning Extensions

Once comfortable with the basic system, try:

1. **Add disturbances**: Modify dynamics to include wind
2. **Sensor noise**: Add measurement noise to states
3. **Observer design**: Implement Kalman filter
4. **Different controllers**: Try PID, MPC, or nonlinear control
5. **Formation flight**: Simulate multiple quadrotors
6. **Obstacle avoidance**: Add constraints to trajectory

---

## 💡 Tips for Success

1. **Start simple**: Run default settings first
2. **One change at a time**: Modify one parameter, observe effect
3. **Save your work**: Use version control or dated folders
4. **Document changes**: Keep notes on what works
5. **Check stability**: Always verify closed-loop poles

---

## ✅ Expected Terminal Output

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
```

These are excellent results showing tight tracking!

---

## 🆘 Getting Help

If you encounter issues:

1. **Check file locations**: All files in same folder?
2. **MATLAB version**: R2019b or newer?
3. **Toolboxes**: Control System Toolbox installed?
4. **Clean start**: Clear workspace and try again
   ```matlab
   clear; clc; close all;
   build_simulink_model();
   complete_setup;
   ```

---

## 🎉 Next Steps

After successful simulation:
- Try the 3D animation (type 'y' when prompted)
- Modify waypoints for different maneuvers
- Tune LQR weights to see effects
- Experiment with physical parameters
- Read the full README for advanced topics

Happy flying! 🚁