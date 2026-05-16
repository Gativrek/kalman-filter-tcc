# Overview

MATLAB simulations developed for an undergraduate thesis on the **global localization problem** applied to a unicycle-model robot. The project compares three filtering approaches — a linear Kalman Filter (KF), an Extended Kalman Filter (EKF), and an Unscented Kalman Filter (UKF) — across four simulation versions of increasing complexity and features. The thesis is available in the [IFF Digital Library](https://bd.centro.iff.edu.br/jspui/bitstream/123456789/5289/1/Texto.pdf) in Brazilian Portuguese, with the thesis defense being available on [YouTube](https://youtu.be/98WBEQ-3_ek) with English subtitles.


## Purpose of the Project

Global localization is the task of estimating the pose of a robot (position and heading) within a known environment, starting from an uncertain initial state, using noisy sensor data. The robot is equipped with two sensors:

- **Odometry** — provides linear and angular velocity measurements, subject to drift.
- **GPS** — provides absolute position fixes at a configurable rate, subject to positional noise.

The filters fuse these two sources to produce a state estimate `[x, y, θ]` over time.


## Repository Structure

```
kalman-filter-tcc/
├── main_errorMeasurement.m
├── main_obstacleMapping.m
├── main_heatMapping.m
├── main_analysis.m
├── monteCarlo.m
│
├── shared/
│   ├── filters/
│   ├── models/
│   ├── simulation/
│   ├── mapping/
│   ├── utils/
│   └── visualization/
│
└── versions/
    ├── errorMeasurement/
    ├── obstacleMapping/
    ├── heatMapping/
    └── analysis/
```


## Versions

Each version builds on the previous one.

### Version 1 — Error Measurement (`errorMeasurement`)
Baseline simulation. The robot follows a scripted trajectory under two configurable noise scenarios. Visualizes the estimated trajectories alongside Euclidean position error and heading error over time.

### Version 2 — Obstacle Mapping (`obstacleMapping`)
Extends Version 1 with static circular obstacles and collision dynamics. When a collision occurs, the robot's actual movement deviates from the commanded input, and odometry is derived from the actual displacement rather than the control signal. Collision events are counted and reported at the end.

### Version 3 — Heat Mapping (`heatMapping`)
Extends Version 1 with occupancy heat maps that accumulate the probability-weighted position of each filter estimate over time using a Gaussian kernel weighted by the trace of the covariance matrix.

### Version 4 — Full Analysis (`analysis`)
Extends Version 3 with NEES (Normalized Estimation Error Squared) tracking, which measures filter consistency relative to its own reported uncertainty. This is the most complete single-run version, and the most important one to check out.

### Monte Carlo (`monteCarlo.m`)
Runs 1000 independent simulations using the Version 4 configuration, each with a different random seed. Reports mean and standard deviation of position RMSE, heading RMSE, and NEES for each filter. Includes pairwise Student t-tests and average computational time per iteration. NEES consistency is evaluated against a 90% chi-squared confidence interval. This was the main version utilized to justify the superiority of the EKF over the UKF.


## Robot Model

The robot is modeled using the standard **unicycle kinematic model**:

$$\dot{x} = v \cos(\theta), \quad \dot{y} = v \sin(\theta), \quad \dot{\theta} = \omega$$


## Filters Implemented

| Filter | Linearity Assumption | Jacobians Required |
|--------|---------------------|--------------------|
| KF     | Linear              | Yes                 |
| EKF    | Linearized (1st order Taylor) | Yes       |
| UKF    | None (sigma points) | No                 |

All filters operate in discrete time and share the same process and measurement noise covariance matrices for fair comparison.


## Simulation Parameters

All versions are configured through their respective `initParameters.m` file under `versions/<version>/`. The key parameters are:

| Parameter | Description |
|---|---|
| `scenario` | 1 = low noise / frequent GPS, 2 = high noise / sparse GPS |
| `dt` | Time step [s] |
| `maxIterations` | Simulation length |
| `gpsAvailabilityRate` | GPS fix every N iterations |
| `Q` | Process noise covariance |
| `R_odom` | Odometry measurement noise covariance |
| `R_gps` | GPS measurement noise covariance |
| `ukfAlpha/Beta/Kappa` | UKF sigma point tuning parameters |

The different scenarios also come with different trajectories, with the first being rather slow, utilized for validating the software, and the second one being fast with lots of curves to really test the filters.


## Requirements

- MATLAB R2020b or later
