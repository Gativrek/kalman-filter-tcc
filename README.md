# Kalman Filter Variants for Mobile Robot Localization

MATLAB implementation and comparative analysis of the linear Kalman Filter (KF), Extended Kalman Filter (EKF), and Unscented Kalman Filter (UKF) applied to the mobile robot localization problem, developed for an undergraduate thesis (TCC). The undergraduate thesis is currently only available in Portuguese.

---

## Overview

This repository contains the simulation code supporting a comparative study of three Kalman Filter variants under the **Wake-Up Robot Problem** framework, restricted to the Gaussian tracking localization scenario. The robot is modeled as a **unicycle (differential-drive)** system, and all filters are evaluated through simulation across two operating scenarios: nominal and aggressive.

Performance is assessed using two metrics:

- **RMSE** (Root Mean Square Error) — measures estimation accuracy
- **NEES** (Normalized Estimation Error Squared) — measures statistical consistency

---

## Repository Structure

```
kalman-filter-tcc/
├── analysis/           # NEES computation and Monte Carlo simulations
├── errorMeasurement/   # RMSE computation
├── heatMapping/        # Spatial heatmap visualization
└── obstacleMapping/    # Obstacle environment
```

### Folder Descriptions

**`analysis/`**  
Contains the 4th version of the script. Runs alls filters and showcases NEES evolution, alongside covariance evolution and heatmaps. A separate script runs Monte Carlo simulation.

**`errorMeasurement/`**  
Contains the 1st version of the scripts. Runs all filters and showcases RMSE. Outputs are used to generate the performance comparison tables and plots reported in the thesis.

**`heatMapping/`**  
Contains the 3rd version of the scripts. Generates spatial heatmaps showing where each filter accumulates the most localization error across the robot's trajectory in the environment.

**`obstacleMapping/`**  
Contains the 2nd version of the scripts. Generates obstacles and collision checks, however lacks a module to fix trajectories.

---

## Robot Model

The robot is modeled using the standard **unicycle kinematic model**:

$$\dot{x} = v \cos(\theta), \quad \dot{y} = v \sin(\theta), \quad \dot{\theta} = \omega$$

---

## Filters Implemented

| Filter | Linearity Assumption | Jacobians Required |
|--------|---------------------|--------------------|
| KF     | Linear              | Yes                 |
| EKF    | Linearized (1st order Taylor) | Yes       |
| UKF    | None (sigma points) | No                 |

All filters operate in discrete time and share the same process and measurement noise covariance matrices for fair comparison.

---

## Simulation Parameters

| Parameter | Nominal Scenario | Aggressive Scenario |
|-----------|-----------------|---------------------|
| Monte Carlo runs | 1000 | 1000 |
| Process noise | Low | High |
| Measurement noise | Low | High |
| NEES confidence interval | 90% | 90% |

---

## Requirements

- MATLAB (R2024a or later recommended)
- No additional toolboxes required

