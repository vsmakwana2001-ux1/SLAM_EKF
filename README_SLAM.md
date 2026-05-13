# Cooperative SLAM — PPADM / IPRES

> Cranfield University — 2025–2026

---

## Overview

This project implements and compares four SLAM algorithms for two cooperating autonomous vehicles navigating a simulated 50 m × 50 m urban road environment with 50 landmarks. The sections build progressively in both algorithmic complexity and realism:

| Section | Algorithm | Sensor | Data Association |
|---|---|---|---|
| **1.1** | RBPF SLAM | Range + Bearing | Known correspondence |
| **1.2** | RBPF SLAM | Bearing Only | Known correspondence |
| **1.3** | RBPF SLAM | Cartesian x-y | Nearest Neighbour + gate |
| **1.4** | Full EKF SLAM | Range + Bearing | Known correspondence |

Sections 1.1–1.3 use a **Rao-Blackwellized Particle Filter (RBPF)**, decoupling the robot pose (particles) from landmark estimation (per-particle EKFs). Section 1.4 replaces this with a single **augmented-state Extended Kalman Filter**, jointly estimating all poses and landmarks in one 103-dimensional state vector with a full cross-correlated covariance matrix.

In all sections, three **sensor fusion** strategies allow the two vehicles to share and combine their landmark estimates cooperatively.

---

## Table of Contents

- [Shared Scenario and Motion Model](#shared-scenario-and-motion-model)
- [1.1 — Range and Bearing RBPF SLAM](#11--range-and-bearing-rbpf-slam)
- [1.2 — Bearing-Only RBPF SLAM](#12--bearing-only-rbpf-slam)
- [1.3 — Data Association with Nearest Neighbour](#13--data-association-with-nearest-neighbour)
- [1.4 — Full EKF SLAM](#14--full-ekf-slam)
- [Sensor Fusion Strategies](#sensor-fusion-strategies)
- [Resampling (Sections 1.1–1.3)](#resampling-sections-1113)
- [Repository Structure](#repository-structure)
- [How to Run](#how-to-run)
- [Parameters](#parameters)
- [Results](#results)
- [References](#references)

---

## Shared Scenario and Motion Model

**File:** `Initialisation_scenario.m`, `Propagation.m`

All four sections share the same environment: 50 random landmarks constrained off the road lanes, two vehicles navigating clockwise at ~2.4 m/s with three 90° right turns over 100 timesteps (50 s total). Noisy velocity commands are generated independently per car by subtracting process noise from the shared reference.

**Unicycle kinematic motion model:**

```
x_{t+1}  =  x_t + dt * v * cos(psi + psi_dot * dt)
y_{t+1}  =  y_t + dt * v * sin(psi + psi_dot * dt)
psi_{t+1} = psi_t + psi_dot * dt
```

```
Q = diag([0.01,  0.0001])    % process noise [m^2/s^2,  rad^2/s^2]
```

| Property | Value |
|---|---|
| Simulation area | −25 m to +25 m |
| Landmarks | 50 |
| Timestep `dt` | 0.5 s |
| Timesteps | 100 (50 s total) |
| Sensor range | 10 m |
| Car 1 start pose | (−20, 20, 0) |
| Car 2 start pose | (−10, 10, 0) |
| Random seed | 2025 |

---

## 1.1 — Range and Bearing RBPF SLAM

**Folder:** `1_1_range_bearing/`

### Algorithm

**Rao-Blackwellized Particle Filter SLAM.** The joint posterior is factorised as:

```
p(x_{1:t}, m | z_{1:t}, u_{1:t}) = p(m | x_{1:t}, z_{1:t}) * p(x_{1:t} | z_{1:t}, u_{1:t})
```

- **Robot pose** `[x, y, ψ]` is represented by 500 weighted particles.
- **Each particle** maintains its own per-landmark EKF (position + 2×2 covariance).

### Measurement Model

Range and bearing to each landmark within 10 m:

```
z = [ sqrt(dx^2 + dy^2) ]    % range [m]
    [ atan2(dy, dx)      ]    % bearing [rad], wrapped to [-pi, pi]

R = diag([2.0,  0.04])       % measurement noise [m^2,  rad^2]
```

### EKF Jacobian (2×2)

```
H = [  dx/r    dy/r  ]
    [ -dy/r^2  dx/r^2 ]
```

### Update and Weight

For each particle and each in-view landmark:

```
S = H * P * H' + R             (2x2)
K = P * H' * S^{-1}            (2x2)
m = m + K * (z - z_hat)
P = (I - K*H) * P
w ∝ w * |2πS|^{-1/2} * exp(-0.5 * v' * S^{-1} * v)
```

Range directly constrains depth → **fully observable** from a single observation.

---

## 1.2 — Bearing-Only RBPF SLAM

**Folder:** `1_2_bearing_only/`

Same RBPF structure as 1.1. Only the measurement model and Jacobian change.

### Measurement Model

Bearing only — no range:

```
z = atan2(dy, dx)    % scalar [rad]
R = 0.04             % [rad^2] (scalar)
```

### EKF Jacobian (1×2)

```
H = [-dy/r^2,  dx/r^2]    % r^2 = dx^2 + dy^2  (r^2 used directly — no sqrt needed)
```

### Observability Challenge

A single bearing constrains the landmark to a **ray** from the vehicle. Landmark depth is unobservable until sufficient **vehicle motion** (parallax) allows bearing rays from different positions to intersect. The animation draws rays at fixed `max_read_distance` length since no range is measured.

### Key Difference from 1.1

| Aspect | 1.1 | 1.2 |
|---|---|---|
| `Measurement.m` | `[r; theta]` | `theta` (scalar) |
| `R` | `diag([2.0, 0.04])` | `0.04` (scalar) |
| Jacobian `H` | 2×2 | 1×2 |
| Observability | Full (single observation) | Partial (requires parallax) |
| Animation arrows | Drawn to measured range | Drawn to `max_read_distance` |

---

## 1.3 — Data Association with Nearest Neighbour

**Folder:** `1_3_data_association/`

### Measurement Model

Cartesian relative position — linear measurement:

```
z = [ xL - x ]    % relative x [m]
    [ yL - y ]    % relative y [m]

R = diag([1.0,  1.0])    % [m^2,  m^2]
```

### EKF Jacobian (2×2)

Because the measurement is a linear function of landmark position, the Jacobian is the identity — **no linearisation error**:

```
H = eye(2) = [1  0]
             [0  1]
```

### Data Association: Nearest Neighbour with Gate

This is the key contribution of section 1.3. In 1.1 and 1.2, measurement column `l` was assumed to correspond to landmark `l` (known correspondence). Here, landmark identity is **unknown** — the algorithm must associate each measurement to the most plausible landmark.

The loop structure in `SLAM_PF.m` is inverted: instead of iterating over landmarks and reading the corresponding measurement, it iterates over **measurements** and searches for the best-matching landmark.

**Algorithm (per particle, per measurement `z(:,m)`):**

```
1. Skip if measurement is empty:  norm(z(:,m)) < 1e-6

2. For each landmark l in the field of view:
       z_pred = Measurement(pose, landmark_l_pos)
       dist   = norm(z(:,m) - z_pred)

3. Select:  best_l = argmin(dist)

4. Gate:    only proceed if best_dist < 5 m

5. EKF update landmark best_l, update particle weight
```

The 5 m **gate** rejects associations that are implausibly far, preventing corrupted landmarks from being updated with unrelated measurements.

### Limitations of Nearest Neighbour

NN can misassociate measurements when two landmarks are close, degrading the filter. More robust alternatives include Joint Compatibility Branch and Bound (JCBB) or the Probabilistic Data Association Filter (PDAF).

### Key Differences from 1.1/1.2

| Aspect | 1.1 / 1.2 | 1.3 |
|---|---|---|
| Measurement | Polar `[r; theta]` or `theta` | Cartesian `[dx; dy]` |
| Jacobian `H` | Nonlinear (linearised) | `eye(2)` — exact |
| Association | Known correspondence | Nearest Neighbour + 5 m gate |
| Loop structure | Outer: landmarks → measurements | Outer: measurements → landmarks |
| `num_particles` | 500 | 200 |
| `P0` | `diag([1, 1])` | `diag([0.1, 0.1])` |

---

## 1.4 — Full EKF SLAM

**Folder:** `1_4_ekf_slam/`

### Fundamental Difference from Sections 1.1–1.3

Sections 1.1–1.3 decouple pose and landmark estimation via Rao-Blackwellization. **Section 1.4 does not** — a single EKF jointly estimates the entire state in one **103-dimensional augmented state vector** with a full **103×103 covariance matrix** that explicitly captures cross-correlations between the robot pose and all landmark positions.

```
x = [ x_robot (3x1) ]    =   [x, y, psi, xL1, yL1, xL2, yL2, ..., xL50, yL50]'
    [ landmarks (100x1) ]
```

```
P = (103 x 103) full covariance matrix
```

The off-diagonal blocks of `P` encode the correlation between robot pose uncertainty and landmark uncertainty — information that the RBPF approach approximates per-particle.

**File:** `EKF_SLAM.m`

### Measurement Model

Back to range and bearing (same as section 1.1):

```
z = [ sqrt(dx^2 + dy^2) ]    R = diag([2.0,  0.04])
    [ atan2(dy, dx)      ]
```

### Prediction Step

**1. State prediction** — only the robot pose changes; landmarks are stationary:

```
x_r_new = x_r + [ v * dt * cos(psi) ]
                 [ v * dt * sin(psi) ]
                 [ omega * dt        ]
x_new   = [x_r_new; x(4:end)]       % landmarks unchanged
```

**2. Full state Jacobian** — block-diagonal structure exploiting landmark stationarity:

```
F_r = [1,  0,  -v*dt*sin(psi) ]    (3x3 pose block)
      [0,  1,   v*dt*cos(psi) ]
      [0,  0,   1             ]

F = blkdiag(F_r, eye(2*N))          (103x103 full Jacobian)
```

**3. Control noise propagation** — maps 2D process noise `Q` into 3D pose space, then into the full 103D state:

```
F_u     = [dt*cos(psi),  0 ]    (3x2, maps [v; omega] noise to pose)
          [dt*sin(psi),  0 ]
          [0,           dt ]

Q_pose  = F_u * Q * F_u'           (3x3)

G       = [I_3; zeros(2N, 3)]      (103x3, injects into augmented state)
Q_full  = G * Q_pose * G'          (103x103, pose noise only)
```

**4. Predicted covariance:**

```
P_new = F * P * F' + Q_full
```

### Update Step

Sequential per-landmark updates. For each observed landmark `l`:

**1. Extract landmark indices:**

```
idx_lx = 3 + 2*(l-1) + 1
idx_ly = idx_lx + 1
```

**2. Predicted measurement:**

```
dx = lx - x_new(1),   dy = ly - x_new(2)
r  = sqrt(dx^2 + dy^2)
z_hat = [r;  atan2(dy, dx)]
nu    = z(:,l) - z_hat;    nu(2) = wrapToPi(nu(2))
```

**3. Observation Jacobian** (2×103, sparse — zero everywhere except robot pose and landmark `l` columns):

```
H = zeros(2, 103)

% Robot pose block (columns 1–3):
H(1, 1:3) = [-dx/r,  -dy/r,   0]
H(2, 1:3) = [ dy/r^2, -dx/r^2, 0]

% Landmark l block (columns idx_lx : idx_ly):
H(1, idx_lx:idx_ly) = [ dx/r,   dy/r ]
H(2, idx_lx:idx_ly) = [-dy/r^2,  dx/r^2]
```

**4. Kalman gain and update:**

```
S     = H * P_new * H' + R          (2x2)
K     = P_new * H' / S              (103x2)
x_new = x_new + K * nu             (full 103D state updated)
P_new = (I - K*H) * P_new          (full 103x103 covariance updated)
```

The full-state update means that **correcting one landmark updates the robot pose estimate and all other landmark estimates** through the off-diagonal cross-covariance blocks — the key advantage of monolithic EKF SLAM over the RBPF approach.

### Initialisation (`Main.m`)

```matlab
x_car1 = [initial_car1; initial_landmarks(:)];   % 103x1
P_robot = diag([1, 1, 0.1^2]);                    % 3x3 pose block
landmark_blocks = repmat({P0}, num_landmarks, 1);  % 50 x {2x2}
P_car1 = blkdiag(P_robot, landmark_blocks{:});     % 103x103 block-diagonal
```

### Visualisation

`Plot_animation.m` is now a proper MATLAB function and includes a `draw_ellipse` helper that plots 2σ uncertainty ellipses for each landmark's marginal covariance extracted from `P`:

```matlab
cov_landmark1(:,:,l,t) = P_car1(3+2*l-1 : 3+2*l,  3+2*l-1 : 3+2*l)
```

### EKF SLAM vs RBPF SLAM — Key Trade-offs

| Aspect | RBPF (1.1–1.3) | EKF SLAM (1.4) |
|---|---|---|
| State representation | N particles × 1 pose + M per-particle EKFs | Single 103D augmented state + 103×103 covariance |
| Cross-correlations | Implicit (per particle, not across landmarks) | Explicit — full cross-covariance between all states |
| Pose uncertainty | Nonparametric (particle distribution) | Gaussian (single Gaussian) |
| Scalability | Degrades with map size; O(N × M) | O(M²) — grows quadratically with number of landmarks |
| Consistency | Can remain consistent with enough particles | Can become inconsistent due to linearisation errors |
| Data association | Straightforward per particle | Assumed known (known correspondence) in this implementation |
| Computational cost | Moderate (500 particles × 50 EKFs) | High covariance matrix operations (103×103 per step) |

---

## Sensor Fusion Strategies

**Files:** `Fusion_state.m`, `Fusion_measure.m`, `Fusion_covariance.m`

Controlled by `fusion_flag` in `Main.m`. Applies per-landmark and per-particle (sections 1.1–1.3), only when both cars have previously observed that landmark. Note: the fusion module in section 1.4 is a placeholder (`x_SF = x_car1`); the fusion files remain from the RBPF framework.

---

### `fusion_flag = 0` — No Fusion (Baseline)

Each car runs SLAM independently.

---

### `fusion_flag = 1` — State Fusion (`Fusion_state.m`)

Linear KF combination assuming statistical independence:

```
m_SF = a + A * (A + B)^{-1} * (b - a)
P_SF = A - A * (A + B)^{-1} * A'
```

---

### `fusion_flag = 2` — Measurement Fusion (`Fusion_measure.m`)

Stacks both raw measurements into a joint observation:

```
z_joint = [z1; z2],     H = [I; I]
S = H * P * H' + Rm,    K = P * H' * S^{-1}
m_SF = m_SF + K * (z_joint - [z_hat_1; z_hat_2])
P_SF = (I - K * H) * P
```

`Rm` dimensions: 4×4 (1.1, 1.4), 2×2 (1.2), 4×4 (1.3).

---

### `fusion_flag = 3` — Covariance Intersection (`Fusion_covariance.m`)

Consistent fusion with no independence assumption. Optimal `omega ∈ [0, 1]` minimises the determinant of the fused covariance per-landmark:

```matlab
covariance_det = @(omega) 1/det(omega*inv_A + (1-omega)*inv_B);
omega = fminbnd(covariance_det, 0, 1);
```

```
P_SF^{-1} = omega * A^{-1} + (1-omega) * B^{-1}
m_SF      = P_SF * (omega * A^{-1} * a + (1-omega) * B^{-1} * b)
```

---

## Resampling (Sections 1.1–1.3)

**File:** `Resample.m`

Sequential Importance Resampling (SIR). Triggered only when at least one landmark update occurred (`doResample = true`).

| Condition | Behaviour |
|---|---|
| `weightSum > 1e50` | All weight on the single highest-likelihood particle |
| `1e-50 < weightSum ≤ 1e50` | Standard normalisation and resampling |
| `weightSum ≤ 1e-50` | Uniform weights reset |

---

## Repository Structure

```
slam/
├── 1_1_range_bearing/          # RBPF SLAM, sensor: [r; theta]
│   ├── SLAM_PF.m               # Predict + per-particle EKF update + SIR
│   ├── Measurement.m           # Returns [r; theta]
│   └── ...
│
├── 1_2_bearing_only/           # RBPF SLAM, sensor: [theta] only
│   ├── SLAM_PF.m               # 1×2 Jacobian H, scalar R
│   ├── Measurement.m           # Returns theta (scalar)
│   └── ...
│
├── 1_3_data_association/       # RBPF SLAM, sensor: [dx; dy], NN association
│   ├── SLAM_PF.m               # Outer loop: measurements → NN search → gate → EKF
│   ├── Measurement.m           # Returns [dx; dy]
│   └── ...
│
└── 1_4_ekf_slam/               # Full augmented-state EKF SLAM, sensor: [r; theta]
    ├── EKF_SLAM.m              # 103D state, full 103×103 covariance, sequential updates
    ├── Main.m                  # EKF initialisation, no particles
    ├── Measurement.m           # Returns [r; theta]
    ├── Plot_animation.m        # Function form, draws 2-sigma covariance ellipses
    ├── Plot_road.m             # Function form (uses global block, width)
    └── ...
```

Shared/identical files across sections: `Propagation.m`, `Resample.m` (1.1–1.3 only), `Fusion_state.m`, `Fusion_measure.m`, `Fusion_covariance.m`, `Plot_analysis.m`.

---

## How to Run

1. Open MATLAB and navigate to the relevant section folder.
2. Open `Main.m` and set `fusion_flag`:

| `fusion_flag` | Mode |
|---|---|
| `0` | No fusion — independent SLAM per car |
| `1` | State fusion |
| `2` | Measurement fusion |
| `3` | Covariance Intersection |

3. Run:
```matlab
Main
```

`Plot_analysis` runs automatically at the end, producing three subplots: car position RMSE, landmark RMSE, and landmark covariance norm — all vs. time.

---

## Parameters

| Parameter | 1.1 | 1.2 | 1.3 | 1.4 |
|---|---|---|---|---|
| `num_particles` | 500 | 500 | 200 | — (no PF) |
| `num_landmarks` | 50 | 50 | 50 | 50 |
| `dt` | 0.5 s | 0.5 s | 0.5 s | 0.5 s |
| Sensor range | 10 m | 10 m | 10 m | 10 m |
| `σ²_v` | 0.01 | 0.01 | 0.01 | 0.01 |
| `σ²_ψ̇` | 0.0001 | 0.0001 | 0.0001 | 0.0001 |
| `σ²_r` | 2.0 m² | — | — | 2.0 m² |
| `σ²_θ` | 0.04 rad² | 0.04 rad² | — | 0.04 rad² |
| `σ²_x, σ²_y` | — | — | 1.0 m² | — |
| `P0` | `diag([1,1])` | `diag([1,1])` | `diag([0.1,0.1])` | `diag([1,1])` |
| `P_robot` init | per particle | per particle | per particle | `diag([1,1,0.01])` |
| Association gate | — | — | 5 m | — |
| State size | N × 103 | N × 103 | N × 103 | 103 (single) |

---

## Results

### Section 1.1 — Range and Bearing RBPF SLAM (no fusion)

| Vehicle | Avg Car RMSE (m) | Avg Landmark RMSE (m) |
|---|---|---|
| Car 1 | 0.7689 | 7.9728 |
| Car 2 | 0.5595 | 8.1010 |

### Section 1.2 — Bearing-Only RBPF SLAM (no fusion)

| Vehicle | Avg Car RMSE (m) | Avg Landmark RMSE (m) |
|---|---|---|
| Car 1 | 0.6322 | 8.2970 |
| Car 2 | 0.4933 | 8.6064 |

Bearing-only achieves slightly lower car RMSE than range-and-bearing, but higher landmark RMSE due to the depth-unobservable initialisation challenge and reliance on parallax for convergence.

### Section 1.3 — Data Association (Nearest Neighbour, Cartesian x-y)

| Vehicle | Avg Car RMSE (m) | Avg Landmark RMSE (m) |
|---|---|---|
| Car 1 | 0.9802 | 8.9475 |
| Car 2 | 0.4769 | 8.8344 |

Highest landmark RMSE across sections 1.1–1.3, reflecting occasional misassociations from the nearest-neighbour strategy in a cluttered 50-landmark environment.

### Section 1.4 — EKF SLAM vs RBPF SLAM (Range & Bearing, no fusion)

| Algorithm | Car 1 RMSE (m) | Car 2 RMSE (m) | Car 1 Landmark RMSE (m) | Car 2 Landmark RMSE (m) |
|---|---|---|---|---|
| RBPF (Particle Filter) | 0.7689 | 0.5595 | 7.9728 | 8.1010 |
| Full EKF SLAM | 0.7581 | 0.7381 | 7.9145 | 8.2370 |

EKF SLAM achieves marginally lower landmark RMSE for Car 1 (7.91 vs 7.97 m) through explicit cross-correlated covariance updates, but Car 2 position RMSE is higher (0.74 vs 0.56 m), suggesting the single Gaussian pose approximation is less flexible than the particle distribution in this scenario.

---

## References

- Montemerlo, M., et al. (2002). "FastSLAM: A factored solution to the simultaneous localization and mapping problem." *AAAI/IAAI*.
- Dissanayake, G., et al. (2001). "A solution to the simultaneous localization and map building (SLAM) problem." *IEEE TRA*, 17(3), 229–241.
- Julier, S. J., & Uhlmann, J. K. (1997). "A non-divergent estimation algorithm in the presence of unknown correlations." *ACC*.
- Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
- Bar-Shalom, Y., Fortmann, T. E. (1988). *Tracking and Data Association*. Academic Press.
- Doucet, A., de Freitas, N., & Gordon, N. (Eds.) (2001). *Sequential Monte Carlo Methods in Practice*. Springer.

---

*Cranfield University — PPADM / IPRES — 2025–2026*
