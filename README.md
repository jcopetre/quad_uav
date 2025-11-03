# Quadrotor 6DOF LQR Control Framework

A complete MATLAB simulation framework for trajectory tracking control of quadrotor UAVs using Linear Quadratic Regulator (LQR) control. Features intelligent trajectory generation, comprehensive Monte Carlo robustness analysis, and publication-quality output generation.

## 🚀 Quick Start

### 1. Initialize Environment

```matlab
init_project  % Run once per MATLAB session
```

### 2. Run Your First Simulation

```matlab
cd examples
basic_simulation  % Simple demonstrations
```

### 3. Try a Trajectory

```matlab
simulate_trajectory('basic_maneuver.wpt')
```

**That's it!** You just ran a full 6DOF quadrotor simulation with LQR control.

## 📚 Documentation

**New users:** Start with the [Quick Start Guide](./doc/quick_start.md)

**Complete documentation:**
- [Installation Guide](./doc/installation.md) - Requirements and setup
- [Quick Start Tutorial](./doc/quick_start.md) - Your first simulation in 5 minutes
- [Architecture Overview](./doc/architecture.md) - System design and data flow
- [API Reference](./doc/api_reference.md) - Complete function documentation
- [Monte Carlo Guide](./doc/monte_carlo_guide.md) - Robustness analysis workflow
- [Trajectory Generation](./doc/trajectory_generation_overview.md) - Technical deep dive on trajectory methods
- [Trajectory Design Examples](./doc/trajectory_design_examples.md) - Practical waypoint design patterns

## 📁 Project Structure

```
<root>/
├── src/                     # Core framework code
│   ├── simulation/          # Single trajectory simulation
│   ├── monte_carlo/         # Robustness analysis
│   ├── trajectory/          # Trajectory generation
│   ├── output/              # Figure and report generation
│   ├── data/                # Data management and validation
│   └── utils/               # Utility functions
├── vehicle/                 # Vehicle models and parameters
├── control/                 # Control law implementations
├── dynamics/                # Nonlinear dynamics models
├── trajectories/            # Waypoint files (*.wpt)
├── examples/                # Example scripts
├── test/                    # Comprehensive test suite
└── doc/                     # Documentation
```

## ✨ Key Features

- **Intelligent Trajectory Generation**: Automatic method selection (MAKIMA interpolation vs. minimum snap optimization) based on waypoint timing
- **Trajectory Feasibility Checking**: Pre-flight validation of yaw rate/acceleration demands against vehicle constraints
- **Robust LQR Control**: Linearized control with comprehensive feasibility checking
- **Monte Carlo Analysis**: Parallel parameter perturbation studies with statistical analysis
- **Data Validation**: Strict data contracts ensure simulation integrity
- **Publication Ready**: Automated figure generation with LaTeX metrics
- **Pure MATLAB**: No Simulink required, minimal toolbox dependencies

## 🎯 Quick Command Reference
```matlab
% Initialize
init_project

% Single simulation
simulate_trajectory('basic_maneuver.wpt')

% Custom tuning
Q = diag([200 200 200 20 20 5 20 20 20 2 2 1]);
simulate_trajectory('basic_maneuver.wpt', Q)

% Monte Carlo
cd examples
robustness_study

% Run tests
cd test
run_tests
```

## 🎯 Example Output

A typical simulation produces:

```
Position Tracking:
  RMSE:             0.0234 m
  Max error:        0.0456 m

Attitude:
  RMSE:             1.23 deg
  Max roll:         5.67 deg

Control Effort:
  Thrust sat.:      0.0% of time
  
Success:            ✓ YES
```

## 🧪 Testing

```matlab
cd test
run_tests  % Runs 61+ unit and integration tests
```
## Standard Test Trajectories

- `hover_test.wpt` - Stationary hover (10s) for stability validation
- `simple_square.wpt` - Basic square pattern for integration testing
- `figure_eight_long.wpt` - Aggressive maneuver for stress testing

## 🔬 Monte Carlo Robustness Analysis

```matlab
cd examples
robustness_study  % Complete workflow example
```

This runs parameter uncertainty analysis (500+ trials) and generates:
- Performance distribution plots
- Parameter sensitivity analysis
- Correlation studies
- Publication-ready figures

See [Monte Carlo Guide](./doc/monte_carlo_guide.md) for details.

## 🛠️ Requirements

- **MATLAB** R2019b or later
- **Control System Toolbox** (for `lqr`)
- **Optimization Toolbox** (for minimum snap trajectories)
- Parallel Computing Toolbox (optional, for faster Monte Carlo)
- Statistics Toolbox (optional, for boxplots)

**No Simulink required** - Pure MATLAB implementation

## 📊 State and Control

**State Vector** (12 states):
```
x = [x, y, z, φ, θ, ψ, ẋ, ẏ, ż, p, q, r]ᵀ
```

**Control Vector** (4 inputs):
```
u = [F, τ_φ, τ_θ, τ_ψ]ᵀ
```

**Coordinate Frame**: NED (North-East-Down) aerospace convention

## ❓ Common Issues

**"Function not found"**: Run `init_project` from project root

**Poor tracking**: Check trajectory segments ≥ 3s, tune LQR weights (increase position Q)

**Plots don't appear**: If capturing output, set `options.plot = true`

**See** [Installation Guide](./doc/installation.md#troubleshooting) for complete troubleshooting.

## 🔜 Future Work

- Simulation extension
    - sensor and motor dynamics
    - environmental disturbances
    - aero effects such as drag, ground effect and rotor interaction
- Abstraction layer to facilitate extension
    - trajectory generation
    - controller
- Configuration based study definition
- Advanced visualization
- Extended Kalman Filter (EKF) state estimation
- Multiple controller comparison framework
- Gain scheduling for large-angle maneuvers
- Hardware deployment utilities

## 📚 Key References

- **LQR Theory**: Anderson & Moore (1990). *Optimal Control: Linear Quadratic Methods*
- **Quadrotor Dynamics**: Bouabdallah (2007). *Design and Control of Quadrotors*
- **Minimum Snap**: Mellinger & Kumar (2011). *Minimum Snap Trajectory Generation*. ICRA 2011

## 📖 Citation

If you use this framework in your research, please cite:
```bibtex
@software{quadrotor_lqr_framework,
  author = {Copeland, Trey},
  title = {Quadrotor 6DOF LQR Control Framework},
  year = {2025},
  institution = {University of Tennessee, Knoxville},
  course = {ME590}
}
```
## 📧 Project Information

Graduate research project (ME590) at University of Tennessee, Knoxville.  
Focused on quadrotor control and Monte Carlo robustness analysis.

Author: Trey Copeland

**Last Updated**: November 2025
