# Installation Guide

Complete setup instructions for the Quadrotor LQR Control Framework.

## System Requirements

### MATLAB Version

- **Minimum**: MATLAB R2019b
- **Recommended**: MATLAB R2021a or later
- **Tested on**: R2021a, R2023b

### Required Toolboxes

The following MATLAB toolboxes are **required**:

1. **Control System Toolbox**
   - Used for: `lqr()` function in controller design
   - Essential for LQR gain computation

2. **Optimization Toolbox**
   - Used for: `quadprog()` in minimum snap trajectory generation
   - Required for: Optimal trajectory planning

### Optional Toolboxes

These toolboxes enhance functionality but are not required:

1. **Parallel Computing Toolbox**
   - Enables: `parfor` loops in Monte Carlo analysis
   - Benefit: ~4-8x speedup on multi-core systems
   - Without it: Monte Carlo still runs (uses `for` loop fallback)

2. **Statistics and Machine Learning Toolbox**
   - Enables: `boxplot()` in visualization
   - Without it: Statistics computed, but boxplot figure skipped

### Operating System

- **Windows**: Fully supported
- **macOS**: Fully supported  
- **Linux**: Fully supported

The framework is pure MATLAB - no OS-specific dependencies.

## Installation Steps

### Step 1: Clone or Download Repository

```bash
# Option 1: Clone with git
git clone <repository-url>
cd quad_uav

# Option 2: Download and extract ZIP
# (Navigate to download location)
cd quad_uav-main
```

### Step 2: Verify File Structure

Check that you have this structure:

```
quad_uav/
├── init_project.m
├── README.md
├── src/
│   ├── simulation/
│   ├── monte_carlo/
│   ├── trajectory/
│   └── ...
├── vehicle/
├── control/
├── dynamics/
├── trajectories/
├── examples/
└── test/
```

### Step 3: Initialize MATLAB Environment

Open MATLAB and navigate to the project root directory:

```matlab
cd /path/to/quad_uav
init_project
```

You should see:

```
================================================================
QUADROTOR LQR CONTROL FRAMEWORK
================================================================
Project root: /path/to/quad_uav

✓ All paths initialized successfully
✓ Critical files verified (8/8)

💡 Call savepath to make these changes permanent
================================================================
```

### Step 4: Verify Installation

Run the quick test:

```matlab
cd test
quick_test_sim
```

If successful, you'll see:

```
========================================
Quadrotor Quick Test
========================================

Step 1: Loading quadrotor model...
Step 2: Creating simple trajectory...
...
Test PASSED! ✓
```

### Step 5: Make Paths Permanent (Optional)

To avoid running `init_project` every session:

```matlab
savepath
```

**Note**: This requires write permission to MATLAB's `pathdef.m`. If you get an error, you can:
1. Run MATLAB as administrator (Windows) or with sudo (Linux/macOS)
2. Or just run `init_project` at the start of each session

## Troubleshooting

### "Function not found" Errors

**Problem**: MATLAB can't find functions like `simulate_trajectory`

**Solution**: Run `init_project` from the project root directory

### Missing Toolbox Errors

**Problem**: Error about `lqr` not found

**Solution**: Install Control System Toolbox via MATLAB Add-Ons

**Check installed toolboxes**:
```matlab
ver  % Lists all installed toolboxes
```

### Path Permission Issues

**Problem**: Can't save path permanently

**Solution**: Either:
1. Run `init_project` at start of each session
2. Add to your personal `startup.m`:
   ```matlab
   % In ~/Documents/MATLAB/startup.m
   if exist('/path/to/quad_uav/init_project.m', 'file')
       cd /path/to/quad_uav
       init_project
   end
   ```

### Test Failures

**Problem**: Tests fail unexpectedly

**Steps**:
1. Ensure you're in project root: `pwd`
2. Clear workspace: `clear; clc`
3. Reinitialize: `init_project`
4. Run single test: `cd test; test_linear_6dof`

If problems persist, check:
- MATLAB version (R2019b minimum)
- Required toolboxes installed
- No naming conflicts with other code on path

## Validation Checklist

Use this checklist to verify your installation:

- [ ] `init_project` runs without errors
- [ ] `cd test; quick_test_sim` passes
- [ ] `cd examples; basic_simulation` runs successfully
- [ ] Can load waypoints: `load_waypoints('basic_maneuver.wpt')`
- [ ] (Optional) `cd test; run_tests` shows all passing

## Next Steps

Once installed:

1. **Learn the basics**: See [Quick Start Guide](./quick_start.md)
2. **Understand the system**: Read [Architecture Overview](./architecture.md)
3. **Run examples**: Try scripts in `examples/` directory
4. **Read API docs**: See [API Reference](./api_reference.md)

## Getting Help

**Check installation**: Run `cd test; quick_test_sim`

**Verify toolboxes**: Run `ver` and check for required toolboxes

**Path issues**: Make sure `init_project` completed successfully

**Still stuck?** Check that:
- You're in the correct directory (project root)
- MATLAB version is R2019b or later
- Required toolboxes are installed
