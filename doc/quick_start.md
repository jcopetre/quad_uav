# Quick Start Guide

Get running with your first quadrotor simulation in 5 minutes.

## Prerequisites

- MATLAB R2019b or later
- Completed [Installation](./installation.md)
- Project initialized (`init_project` run successfully)

## Your First Simulation (1 minute)

### Step 1: Navigate to Examples

```matlab
cd examples
```

### Step 2: Run Basic Simulation

```matlab
basic_simulation
```

**That's it!** You'll see three example simulations run with plots showing:
- 3D trajectory tracking
- Position and attitude time series
- Control inputs

## Understanding What Happened

The `basic_simulation` script demonstrates three usage patterns:

### Example 1: Default Settings

```matlab
simulate_trajectory('basic_maneuver.wpt');
```

This:
1. Loads waypoints from `trajectories/basic_maneuver.wpt`
2. Designs an LQR controller with default tuning
3. Generates trajectory using automatic method selection
4. Simulates 6DOF quadrotor dynamics
5. Computes performance metrics
6. Displays results and plots

### Example 2: Custom Tuning

```matlab
Q_aggressive = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
R_aggressive = diag([0.01 0.5 0.5 0.5]);
simulate_trajectory('basic_maneuver.wpt', Q_aggressive, R_aggressive);
```

This uses more aggressive gains for tighter tracking.

### Example 3: Batch Mode

```matlab
opts.verbose = false;
opts.plot = false;
results = simulate_trajectory('basic_maneuver.wpt', [], [], [], opts);
```

Runs silently and captures results for programmatic use.

## Running Your Own Trajectory

### Option 1: Use Existing Waypoint Files

Try different trajectories in the `trajectories/` directory:

```matlab
simulate_trajectory('hover_test.wpt')        % Simple hover
simulate_trajectory('simple_square.wpt')     % Square pattern
simulate_trajectory('aggressive_slalom.wpt') % Fast maneuvers
```

### Option 2: Create Your Own Waypoint File

Create `trajectories/my_flight.wpt`:

```json
{
  "metadata": {
    "name": "My First Flight",
    "description": "Takeoff, move, land"
  },
  "waypoints": [
    {"label": "start", "time": 0, "x": 0, "y": 0, "z": 0, "yaw": 0},
    {"label": "climb", "time": 2, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "forward", "time": 5, "x": 2, "y": 0, "z": 1, "yaw": null},
    {"label": "land", "time": 8, "x": 0, "y": 0, "z": 0, "yaw": 0}
  ]
}
```

Then run it:

```matlab
simulate_trajectory('my_flight.wpt')
```

**Tips**:
- `yaw` is in radians
- `yaw: null` means auto-calculate from velocity direction
- `yaw: 0` means face north (in NED frame)
- Keep segment durations ≥ 3 seconds for smooth trajectories
- Start and end at z=0 (ground level)

## Understanding the Output

### Console Output

```
================================================================
PERFORMANCE SUMMARY
================================================================
Position Tracking:
  RMSE:             0.0234 m      ← Root-mean-square error
  Max error:        0.0456 m      ← Worst-case deviation
  
Attitude:
  RMSE:             1.23 deg      ← Angular tracking error
  Max roll:         5.67 deg      ← Maximum bank angle
  Max pitch:        6.12 deg      ← Maximum pitch angle
  
Control Effort:
  Mean thrust:      4.91 N        ← Average thrust (≈hover thrust)
  Thrust sat.:      0.0% of time  ← Control saturation check
  
Success:            YES           ← All criteria met
================================================================
```

### Plots Generated

1. **3D Trajectory Plot**
   - Red line: Reference trajectory
   - Blue line: Actual quadrotor path
   - Shows tracking performance spatially

2. **Time Series**
   - Position components (x, y, z)
   - Attitude angles (roll, pitch, yaw)
   - Reference vs. actual

3. **Control Inputs**
   - Thrust (F)
   - Torques (τ_φ, τ_θ, τ_ψ)
   - Shows control effort and saturation

### Metrics Structure

When capturing results:

```matlab
results = simulate_trajectory('my_flight.wpt', [], [], [], opts);

% Access metrics
results.metrics.tracking.rmse_position    % Position error (m)
results.metrics.tracking.rmse_attitude    % Attitude error (rad)
results.metrics.control.mean_thrust       % Average thrust (N)
results.metrics.success.overall           % Pass/fail (boolean)
```

## Common Tasks

### Task 1: Compare Different Tunings

```matlab
% Define weight sets
Q_loose = diag([50 50 50 5 5 1 10 10 10 1 1 0.1]);
Q_tight = diag([200 200 200 20 20 2 20 20 20 2 2 0.2]);

% Run and compare
opts.plot = false;
results_loose = simulate_trajectory('basic_maneuver.wpt', Q_loose, [], [], opts);
results_tight = simulate_trajectory('basic_maneuver.wpt', Q_tight, [], [], opts);

% Compare
fprintf('Loose tuning RMSE: %.4f m\n', results_loose.metrics.tracking.rmse_position);
fprintf('Tight tuning RMSE: %.4f m\n', results_tight.metrics.tracking.rmse_position);
```

### Task 2: Test Initial Condition Sensitivity

```matlab
% Start with initial roll angle
x0 = zeros(12, 1);
x0(4) = deg2rad(5);  % 5° initial roll

simulate_trajectory('basic_maneuver.wpt', [], [], x0);
```

### Task 3: Batch Process Multiple Trajectories

```matlab
trajectories = {'hover_test.wpt', 'simple_square.wpt', 'basic_maneuver.wpt'};

opts.verbose = false;
opts.plot = false;

for i = 1:length(trajectories)
    results(i) = simulate_trajectory(trajectories{i}, [], [], [], opts);
    fprintf('%s: RMSE = %.4f m\n', trajectories{i}, ...
            results(i).metrics.tracking.rmse_position);
end
```

## Next Steps

### Learn the System

- **Architecture**: Understand data flow → [Architecture Overview](./architecture.md)
- **API Reference**: Function details → [API Reference](./api_reference.md)
- **Coordinate Frames**: NED conventions → [Architecture](./architecture.md#coordinate-frames)

### Try Advanced Features

- **Monte Carlo Analysis**: Robustness studies → [Monte Carlo Guide](./monte_carlo_guide.md)
- **Custom Controllers**: Modify LQR weights → [API Reference](./api_reference.md#quadrotor_linear_6dof)
- **Trajectory Generation**: Create complex paths → [API Reference](./api_reference.md#trajectory-generation)

### Run the Test Suite

Verify everything works:

```matlab
cd test
run_tests  % Runs 61+ tests
```

## Troubleshooting

### Problem: "Trajectory file not found"

**Solution**: Ensure you're in project root or use full path
```matlab
cd /path/to/quad_uav
simulate_trajectory('basic_maneuver.wpt')
```

### Problem: Poor tracking performance

**Possible causes**:
1. Trajectory segments too short (< 2s) → Excessive accelerations
2. LQR weights too conservative → Increase position Q values
3. Reference attitude exceeds ±15° → Check with `traj_feasibility_check()`

**Solution**: Let auto-selector choose method, or use longer segments

### Problem: "Linearization assumptions violated"

**Meaning**: Attitude angles exceeded ~15° (small-angle approximation invalid)

**Solution**:
1. Design gentler trajectories (longer segment times)
2. Reduce velocity/acceleration demands
3. Check: `traj_feasibility_check(trajectory, params)`

### Problem: Plots don't appear

**Solution**: Check `nargout`:
```matlab
% No output → plots shown automatically
simulate_trajectory('basic_maneuver.wpt')

% Output captured → plots suppressed (set opts.plot = true)
results = simulate_trajectory('basic_maneuver.wpt');
```

## Quick Reference Commands

```matlab
% Initialize
init_project

% Basic simulation
simulate_trajectory('basic_maneuver.wpt')

% Custom tuning
Q = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
simulate_trajectory('basic_maneuver.wpt', Q)

% Batch mode
opts.plot = false;
results = simulate_trajectory('basic_maneuver.wpt', [], [], [], opts);

% Monte Carlo
cd examples
robustness_study

% Run tests
cd test
run_tests
```

## Where to Go From Here

1. ✅ You've run a basic simulation
2. 📖 Read [Architecture Overview](./architecture.md) to understand system design
3. 🔧 Try [Monte Carlo Guide](./monte_carlo_guide.md) for robustness analysis
4. 📚 Reference [API Documentation](./api_reference.md) when coding

**Ready to dig deeper?** Check out the full [Architecture](./architecture.md) next!
