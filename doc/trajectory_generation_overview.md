# Trajectory Generation: Technical Overview

Complete technical documentation of trajectory generation methods used in the quadrotor control framework.

## Table of Contents
- [Overview](#overview)
- [Method Selection Logic](#method-selection-logic)
- [MAKIMA Interpolation](#makima-interpolation)
- [Minimum Snap Optimization](#minimum-snap-optimization)
- [Lead-in and Lead-out](#lead-in-and-lead-out)
- [Feasibility Checking](#feasibility-checking)
- [Design Guidelines](#design-guidelines)

## Overview

The framework implements two trajectory generation methods with intelligent automatic selection:

1. **MAKIMA Interpolation** - Fast, C² continuous, local control
2. **Minimum Snap Optimization** - Smooth, globally optimal, higher computational cost

Both methods take discrete waypoints and generate continuous time trajectories with position, velocity, acceleration, and jerk profiles.

## Method Selection Logic

The framework automatically selects the appropriate method based on waypoint timing:
```matlab
function trajectory = generate_trajectory_auto(waypoints, dt, params)
    % Analyze segment durations
    segment_times = diff(waypoints.time);
    min_segment = min(segment_times);
    
    % Decision threshold (empirically validated)
    THRESHOLD = 3.0;  % seconds
    
    if min_segment < THRESHOLD
        % Short segments → Use MAKIMA (avoids excessive accelerations)
        method = 'makima';
        reason = sprintf('Short segment detected (%.2fs < %.2fs)', ...
                        min_segment, THRESHOLD);
    else
        % All segments ≥ threshold → Use minimum snap
        method = 'minimum_snap';
        reason = sprintf('All segments ≥ %.2fs threshold', THRESHOLD);
    end
    
    % Generate trajectory
    trajectory = generate_trajectory(waypoints, dt, method);
end
```

**Key Insight:** The 3-second threshold was determined empirically through testing. Minimum snap with short segments produces excessive accelerations that violate actuator limits or linearization assumptions.

## MAKIMA Interpolation

**Method:** Modified Akima piecewise cubic Hermite interpolation

### Mathematical Formulation

MAKIMA constructs a C² continuous piecewise cubic polynomial:

**For each segment [t_i, t_{i+1}]:**
```
p(t) = a₀ + a₁(t-t_i) + a₂(t-t_i)² + a₃(t-t_i)³
```

**Coefficients determined by:**
1. Position values at waypoints (interpolation conditions)
2. First derivatives estimated using weighted differences
3. Weights chosen to suppress oscillations near sharp corners

**Derivatives:**
- Velocity: `v(t) = a₁ + 2a₂(t-t_i) + 3a₃(t-t_i)²`
- Acceleration: `a(t) = 2a₂ + 6a₃(t-t_i)`
- Jerk: `j(t) = 6a₃` (constant per segment)

### Properties

**Advantages:**
- ✅ **Fast** - O(n) complexity, closed-form solution
- ✅ **Local control** - Changing one waypoint affects only nearby segments
- ✅ **No overshoots** - Modified Akima weighting suppresses oscillations
- ✅ **C² continuity** - Smooth acceleration profiles
- ✅ **Handles short segments** - Works well with rapid maneuvers

**Limitations:**
- ❌ Not globally optimal
- ❌ Jerk discontinuous at waypoints
- ❌ No explicit constraint handling
- ❌ Heuristic (not derived from optimality principle)

### When to Use

Use MAKIMA when:
- Rapid maneuvers (segment times < 3s)
- Many waypoints with varying segment durations
- Real-time trajectory generation needed
- Local modifications without global recomputation

### Implementation Details
```matlab
% Core MATLAB function
pp = makima(t_waypoints, p_waypoints);
p = ppval(pp, t_trajectory);

% Derivatives via finite differences
v = gradient(p, dt);
a = gradient(v, dt);
j = gradient(a, dt);
```

**Note:** Velocity, acceleration, and jerk computed via numerical differentiation rather than analytical derivatives for simplicity.

## Minimum Snap Optimization

**Method:** Quadratic programming to minimize integrated squared jerk

### Problem Formulation

**Objective:** Minimize trajectory "snap" (4th derivative):
```
minimize: ∫[t₀ to tf] ||d⁴p/dt⁴||² dt
```

Subject to:
- **Waypoint constraints**: p(t_i) = p_waypoint_i for all waypoints
- **Boundary conditions**: v(t₀) = 0, a(t₀) = 0, v(tf) = 0, a(tf) = 0
- **Continuity**: C³ continuous (position, velocity, acceleration, jerk)

### Mathematical Formulation

**Trajectory representation:**
Each axis represented as piecewise polynomial:
```
For segment k: p_k(t) = Σ[i=0 to n] c_{k,i} * (t - t_k)^i
```

Typically n=7 (7th order polynomials) for sufficient DOF.

**Cost function (matrix form):**
```
J = pᵀ Q p
```

Where:
- p: Coefficient vector [c₀, c₁, ..., c_n]ᵀ
- Q: Hessian matrix encoding snap integral

**Constraints (matrix form):**
```
A_eq * p = b_eq    (equality constraints: waypoints, continuity)
```

**Solution:**
```matlab
p_opt = quadprog(Q, [], [], [], A_eq, b_eq);
```

### Properties

**Advantages:**
- ✅ **Globally optimal** - Minimizes snap across entire trajectory
- ✅ **Smooth** - C³ continuous (including jerk)
- ✅ **Principled** - Derived from optimization problem
- ✅ **Physically motivated** - Snap relates to control smoothness
- ✅ **Better for long segments** - Fully utilizes available time

**Limitations:**
- ❌ **Computationally expensive** - O(n³) for n waypoints
- ❌ **Global coupling** - One waypoint change affects entire trajectory
- ❌ **Struggles with short segments** - Can produce excessive accelerations
- ❌ **No explicit actuator constraints** - May violate thrust/torque limits

### When to Use

Use minimum snap when:
- Smooth, graceful trajectories desired
- Segment times ≥ 3 seconds
- Few waypoints (< 20)
- Offline trajectory planning (not real-time)
- Professional/cinematic flight paths

### Implementation Details

**Key reference:** Mellinger & Kumar (2011), "Minimum Snap Trajectory Generation and Control for Quadrotors," ICRA 2011

**Framework implementation:**
```matlab
function traj = generate_trajectory_minsnap(waypoints, dt)
    % Build cost matrix Q (snap integral)
    Q = build_snap_cost_matrix(waypoints);
    
    % Build constraint matrices (waypoints + continuity)
    [A_eq, b_eq] = build_constraints(waypoints);
    
    % Solve QP
    coeffs = quadprog(Q, [], [], [], A_eq, b_eq);
    
    % Evaluate polynomial at trajectory points
    traj = evaluate_polynomial(coeffs, t_eval);
end
```

**Derivative computation:**
Analytical derivatives from polynomial coefficients:
```
p(t) = c₇t⁷ + c₆t⁶ + ... + c₁t + c₀
v(t) = 7c₇t⁶ + 6c₆t⁵ + ... + c₁
a(t) = 42c₇t⁵ + 30c₆t⁴ + ...
j(t) = 210c₇t⁴ + 120c₆t³ + ...
```

## Lead-in and Lead-out

### Why They Matter

**Problem:** Starting or ending with rapid motion can cause:
- Integration instability (large initial accelerations)
- Actuator saturation
- Linearization assumption violations
- Reference state discontinuities

**Solution:** Add stationary "dwell time" at start/end

### Lead-in Phase

**Purpose:** Give controller time to stabilize before motion begins

**Typical structure:**
```json
{
  "waypoints": [
    {"label": "lead_in_start", "time": 0, "x": 0, "y": 0, "z": 0},
    {"label": "lead_in_end", "time": 2, "x": 0, "y": 0, "z": 0},  ← Hover
    {"label": "maneuver_start", "time": 4, "x": 1, "y": 0, "z": 0},
    ...
  ]
}
```

**Characteristics:**
- Duration: 2-3 seconds recommended
- Zero velocity and acceleration
- Allows controller to reach steady state

### Lead-out Phase

**Purpose:** Controlled deceleration to stop

**Typical structure:**
```json
{
  "waypoints": [
    ...
    {"label": "maneuver_end", "time": 18, "x": 2, "y": 1, "z": 0},
    {"label": "lead_out_start", "time": 20, "x": 0, "y": 0, "z": 0},
    {"label": "lead_out_end", "time": 22, "x": 0, "y": 0, "z": 0}  ← Return to hover
  ]
}
```

### Observed Failures

**Trajectories without lead-in:**
- `figure_eight_long.wpt` - Starts immediately with lateral motion → Integration instability
- `race_track.wpt` - Similar issue

**Success pattern:**
- `simple_square.wpt` - Has 2s lead-in → Stable
- `figure_eight_with_leadin.wpt` - Has 2s lead-in → Stable

**Recommendation:** All trajectories should include 2-3 second lead-in and lead-out phases.

## Feasibility Checking

The framework includes trajectory feasibility validation:
```matlab
function [feasible, violations] = check_trajectory_feasibility(trajectory, params)
    % Check if trajectory respects physical limits
    
    % 1. Acceleration limits (related to max thrust)
    max_accel = norm([trajectory.acceleration], 2, 2);
    accel_limit = (params.u_max(1) / params.m) - params.g;
    
    % 2. Attitude angle requirements (small-angle approximation)
    required_roll = atan2(trajectory.acceleration(:,2), params.g);
    required_pitch = atan2(trajectory.acceleration(:,1), params.g);
    
    ANGLE_LIMIT = deg2rad(15);  % Linearization validity
    
    feasible = all(abs(required_roll) < ANGLE_LIMIT) && ...
               all(abs(required_pitch) < ANGLE_LIMIT) && ...
               all(max_accel < accel_limit);
end
```

**Critical angles:**
- < 10° - Linearization highly accurate
- 10-15° - Acceptable for LQR control
- > 15° - Small-angle approximation questionable
- > 30° - Linearization invalid, expect poor tracking

## Design Guidelines

### Waypoint Timing Rules

1. **Minimum segment duration:**
   - MAKIMA: 0.5s minimum (very aggressive)
   - Minimum snap: 3.0s minimum (recommended)
   - Conservative: 5.0s for smooth flight

2. **Lead-in/lead-out:**
   - Always include 2-3s stationary at start
   - Always include 2-3s stationary at end
   - Start and end at z=0 (ground level)

3. **Segment consistency:**
   - Avoid mixing very short (<2s) and very long (>10s) segments
   - Auto-selector may choose suboptimal method

### Spatial Constraints

1. **Acceleration limits:**
   - Max lateral acceleration ≈ 3 m/s² (conservative)
   - Max vertical acceleration ≈ 5 m/s² (thrust dependent)

2. **Velocity limits:**
   - Max velocity ≈ 3 m/s (for good tracking)
   - Higher velocities require larger attitude angles

3. **Position constraints:**
   - Stay within simulation volume
   - Avoid ground plane (z > 0 in NED)

### Trajectory Design Workflow
```matlab
% 1. Create waypoint file with lead-in
wpt = create_waypoints_with_leadin(...);

% 2. Generate trajectory
traj = generate_trajectory_auto(wpt, 0.01);

% 3. Check feasibility
[feasible, violations] = check_trajectory_feasibility(traj, params);

if ~feasible
    % 4a. Increase segment times
    wpt_new = scale_waypoint_times(wpt, 1.5);
    
    % 4b. Reduce spatial distances
    % OR modify waypoint positions
    
    % 5. Regenerate and recheck
    traj = generate_trajectory_auto(wpt_new, 0.01);
end

% 6. Simulate
results = simulate_trajectory(wpt);
```

## Comparison Summary

| Feature | MAKIMA | Minimum Snap |
|---------|--------|--------------|
| **Optimality** | Heuristic | Globally optimal |
| **Continuity** | C² (acceleration) | C³ (jerk) |
| **Speed** | Very fast O(n) | Slow O(n³) |
| **Short segments** | ✅ Handles well | ❌ May produce excessive accel |
| **Long segments** | ⚠️ Adequate | ✅ Better utilizes time |
| **Local control** | ✅ Yes | ❌ Global coupling |
| **Overshoots** | ✅ Suppressed | ⚠️ Possible with poor spacing |
| **When to use** | < 3s segments, rapid maneuvers | ≥ 3s segments, smooth flight |

## References

1. **MAKIMA:**
   - Akima, H. (1970). "A New Method of Interpolation and Smooth Curve Fitting Based on Local Procedures." *Journal of the ACM*
   - MATLAB Documentation: `makima()`

2. **Minimum Snap:**
   - Mellinger, D., & Kumar, V. (2011). "Minimum Snap Trajectory Generation and Control for Quadrotors." *IEEE ICRA 2011*
   - Richter, C., Bry, A., & Roy, N. (2016). "Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments." *ISRR 2013*

3. **Framework Implementation:**
   - `src/trajectory/generate_trajectory_auto.m` - Auto-selection logic
   - `src/trajectory/generate_trajectory_interp.m` - MAKIMA implementation
   - `src/trajectory/generate_trajectory_minsnap.m` - Minimum snap implementation