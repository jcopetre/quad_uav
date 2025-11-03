# Trajectory Design: Practical Examples

Hands-on guide with working examples and common patterns.

## Basic Patterns

### Hover Test

**Purpose:** Controller validation, system identification
```json
{
  "metadata": {
    "name": "Hover Test",
    "description": "Stationary hover for 10 seconds"
  },
  "waypoints": [
    {"label": "ground", "time": 0, "x": 0, "y": 0, "z": 0, "yaw": 0},
    {"label": "climb", "time": 2, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "hover_start", "time": 4, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "hover_end", "time": 14, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "descend", "time": 16, "x": 0, "y": 0, "z": 0, "yaw": 0}
  ]
}
```

**Characteristics:**
- 2s climb (smooth takeoff)
- 10s hover (data collection)
- 2s descent (controlled landing)
- Total: 16s

**Expected performance:**
- Position RMSE < 0.01m
- Attitude RMSE < 1°
- Validates baseline controller

---

### Simple Square

**Purpose:** Basic waypoint tracking
```json
{
  "metadata": {
    "name": "Simple Square",
    "description": "2m × 2m square at 1m altitude"
  },
  "waypoints": [
    {"label": "start", "time": 0, "x": 0, "y": 0, "z": 0, "yaw": 0},
    {"label": "hover", "time": 2, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "corner1", "time": 7, "x": 2, "y": 0, "z": 1, "yaw": null},
    {"label": "corner2", "time": 12, "x": 2, "y": 2, "z": 1, "yaw": null},
    {"label": "corner3", "time": 17, "x": 0, "y": 2, "z": 1, "yaw": null},
    {"label": "corner4", "time": 22, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "land", "time": 24, "x": 0, "y": 0, "z": 0, "yaw": 0}
  ]
}
```

**Characteristics:**
- 2s lead-in (hover after takeoff)
- 5s per side (0.4 m/s average velocity)
- 2s lead-out (hover before landing)
- Total: 24s

**Design rationale:**
- 5s segments → Minimum snap selected
- Smooth corners with C³ continuity
- Low velocity → Small attitude angles

---

### Figure Eight

**Purpose:** Continuous curved path, attitude coupling test
```json
{
  "metadata": {
    "name": "Figure Eight with Lead-in",
    "description": "Lemniscate curve in horizontal plane"
  },
  "waypoints": [
    {"label": "start", "time": 0, "x": 0, "y": 0, "z": 0, "yaw": 0},
    {"label": "hover", "time": 2, "x": 0, "y": 0, "z": 1, "yaw": null},
    
    {"label": "center", "time": 5, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "right_apex", "time": 10, "x": 2, "y": 0, "z": 1, "yaw": null},
    {"label": "center_cross", "time": 15, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "left_apex", "time": 20, "x": -2, "y": 0, "z": 1, "yaw": null},
    {"label": "return_center", "time": 25, "x": 0, "y": 0, "z": 1, "yaw": null},
    
    {"label": "settle", "time": 28, "x": 0, "y": 0, "z": 1, "yaw": null},
    {"label": "land", "time": 30, "x": 0, "y": 0, "z": 0, "yaw": 0}
  ]
}
```

**Characteristics:**
- 3s lead-in after takeoff
- 5s per figure-eight lobe
- Continuous curved motion (tests acceleration tracking)
- 3s lead-out before landing
- Total: 30s

**Challenge:** Continuous curvature requires coordinated roll/pitch

---

### Aggressive Slalom

**Purpose:** Performance limits, robustness testing
```json
{
  "metadata": {
    "name": "Aggressive Slalom",
    "description": "Rapid lateral maneuvers"
  },
  "waypoints": [
    {"label": "start", "time": 0, "x": 0, "y": 0, "z": 0, "yaw": 0},
    {"label": "hover", "time": 2, "x": 0, "y": 0, "z": 1, "yaw": null},
    
    {"label": "gate1", "time": 4, "x": 1, "y": 1, "z": 1, "yaw": null},
    {"label": "gate2", "time": 6, "x": 2, "y": -1, "z": 1, "yaw": null},
    {"label": "gate3", "time": 8, "x": 3, "y": 1, "z": 1, "yaw": null},
    {"label": "gate4", "time": 10, "x": 4, "y": -1, "z": 1, "yaw": null},
    
    {"label": "decel", "time": 12, "x": 5, "y": 0, "z": 1, "yaw": null},
    {"label": "hover", "time": 14, "x": 5, "y": 0, "z": 1, "yaw": null},
    {"label": "land", "time": 16, "x": 5, "y": 0, "z": 0, "yaw": 0}
  ]
}
```

**Characteristics:**
- 2s segments (SHORT → MAKIMA selected automatically)
- High lateral accelerations
- Tests actuator limits and attitude coupling
- 2s deceleration zone

**Expected:** Moderate tracking errors (RMSE ~0.1-0.2m), attitudes ~10-15°

---

## Common Failures and Fixes

### Failure 1: No Lead-in

**Symptom:** Simulation crashes immediately or has huge initial errors

**Bad trajectory:**
```json
{"label": "start", "time": 0, "x": 0, "y": 0, "z": 0},
{"label": "move", "time": 2, "x": 2, "y": 0, "z": 0}  ← Immediate motion!
```

**Fix:** Add stationary lead-in
```json
{"label": "start", "time": 0, "x": 0, "y": 0, "z": 0},
{"label": "hover", "time": 2, "x": 0, "y": 0, "z": 0},  ← Dwell time
{"label": "move", "time": 4, "x": 2, "y": 0, "z": 0}
```

---

### Failure 2: Segments Too Short

**Symptom:** "Linearization assumptions violated" warning, poor tracking

**Bad trajectory:**
```json
{"label": "p1", "time": 0, "x": 0, "y": 0, "z": 1},
{"label": "p2", "time": 1, "x": 2, "y": 2, "z": 1}  ← 1s for 2.8m!
```

**Required acceleration:** ~5.6 m/s² → attitude angle ~30° → linearization invalid!

**Fix:** Longer segment times
```json
{"label": "p1", "time": 0, "x": 0, "y": 0, "z": 1},
{"label": "p2", "time": 4, "x": 2, "y": 2, "z": 1}  ← 4s for 2.8m
```

---

### Failure 3: Mixed Segment Durations

**Symptom:** Jerky motion, suboptimal method selection

**Bad trajectory:**
```json
{"label": "p1", "time": 0, "x": 0, "y": 0, "z": 1},
{"label": "p2", "time": 2, "x": 1, "y": 0, "z": 1},  ← 2s segment
{"label": "p3", "time": 12, "x": 2, "y": 0, "z": 1} ← 10s segment!
```

**Problem:** Auto-selector sees 2s (< threshold) → Uses MAKIMA for all segments → Suboptimal for 10s segment

**Fix:** Consistent spacing
```json
{"label": "p1", "time": 0, "x": 0, "y": 0, "z": 1},
{"label": "p2", "time": 5, "x": 1, "y": 0, "z": 1},
{"label": "p3", "time": 10, "x": 2, "y": 0, "z": 1}
```

---

## Design Checklist

Before running simulation:

- [ ] **Lead-in included?** (2-3s stationary at start)
- [ ] **Lead-out included?** (2-3s stationary at end)
- [ ] **Start/end at z=0?** (ground level)
- [ ] **Minimum segment duration?**
  - Smooth flight: ≥5s
  - Aggressive: ≥3s
  - Absolute minimum: 0.5s (expert only)
- [ ] **Consistent segment durations?** (avoid mixing very short and very long)
- [ ] **Maximum distance per segment?**
  - Conservative: < 3m per 5s
  - Aggressive: < 2m per 2s
- [ ] **Yaw handling?**
  - `null` for auto (velocity-aligned)
  - `0` for north-facing (or specific angle)

## Validation Workflow
```matlab
% 1. Load and inspect
wpt = load_waypoints('my_trajectory.wpt');
fprintf('Duration: %.1fs\n', wpt.time(end));
fprintf('Segments: %d\n', length(wpt.time)-1);
fprintf('Min segment: %.1fs\n', min(diff(wpt.time)));

% 2. Generate trajectory
traj = generate_trajectory_auto(wpt, 0.01);
fprintf('Method: %s\n', traj.method);

% 3. Check feasibility
params = quadrotor_linear_6dof();
[feasible, violations] = check_trajectory_feasibility(traj, params);

if ~feasible
    fprintf('⚠️ Infeasible trajectory!\n');
    fprintf('Violations: %s\n', violations);
end

% 4. Simulate
results = simulate_trajectory(wpt);
```

## References

See `doc/trajectory_generation_overview.md` for technical details on methods.