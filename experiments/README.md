# Experiments

This directory contains research investigations and analyses that support 
claims made in publications. These are exploratory scripts that document 
the scientific process.

## Directory Organization

Each subdirectory corresponds to a specific research question or analysis:

### `trajectory_limits/`
**Question**: Are trajectory failures due to excessive accelerations (trajectory 
generation problem) or linearization violations (control problem)?

**Scripts**:
- `diagnose_failure_cause.m` - Separates trajectory vs. control limitations
- `test_segment_duration.m` - Tests 3-second threshold hypothesis
- `compare_methods.m` - MAKIMA vs minimum snap at same timing

**Results**: Support the claim that minimum snap generates infeasible trajectories
for segment durations < 3s due to excessive reference attitudes (>15°).

---

## Running Experiments

Experiments are independent analyses and may take time to run. They are not
part of the automated test suite.
```matlab
% From project root
init_project
cd experiments/trajectory_limits
diagnose_failure_cause  % Run specific experiment
```

## Notes

- Experiments may generate large output files (add to .gitignore if needed)
- Scripts are documented but may be less polished than examples/
- Results are preserved for reproducibility
```

## Add to `.gitignore`
```
# Experiment outputs (optional - decide what to track)
experiments/*/results/*.mat
experiments/*/results/*.png
experiments/*/results/*.txt

# Or keep everything for reproducibility (then don't add these)