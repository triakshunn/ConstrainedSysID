# ConstrainedSysID — Developer Guide & Usage Reference

> **This guide is a companion to the existing `README.md`.**
> It explains what each file does, how they connect, and the exact order in which to use them.
> For background on the algorithm, see the paper: [arXiv:2408.08830](https://arxiv.org/abs/2408.08830).

---

## Table of Contents

1. [Background](#background)
2. [Repository Layout](#repository-layout)
3. [Required Toolboxes](#required-toolboxes)
4. [Step-by-Step Usage Guide](#step-by-step-usage-guide)
5. [File Reference](#file-reference)
   - [SysIDMain.m](#sysidmainm--entry-point)
   - [SysIDData.m](#sysiddatam--data-loading-pipeline)
   - [SysIDDigit.m](#sysiddigitm--digit-specific-data-extractor)
   - [dataProcessing.m](#dataprocessingm--signal-processing)
   - [centralDifference.m](#centraldifferencem--numerical-differentiation)
   - [QRDecomposition.m](#qrdecompositionm--parameter-regrouping)
   - [SysIDAlg.m](#sysidalg.m--algorithm-router)
   - [SysIDAlgFull.m](#sysidalg-variants)
6. [Algorithm Option Reference](#algorithm-option-reference)
7. [Parameter Vector Structure](#parameter-vector-structure)
8. [Adapting to a New Robot](#adapting-to-a-new-robot)
9. [Known Issues](#known-issues)

---

## Background

### Problem
Standard robot system identification (SysID) assumes an **open kinematic chain** (e.g., a serial manipulator).
Legged humanoid robots like Digit have **closed kinematic chains** — passive joints constrained by mechanical loops.
These constraints introduce reaction forces/wrenches that invalidate the standard unconstrained regressor equations.

### Solution (from the paper)
The algorithm projects the equations of motion through a **constraint projection matrix** derived from the
constraint Jacobian `J(q)`. This eliminates the unknown reaction forces and yields a modified observation
matrix that depends only on the actuated joints and known parameters.

The full identification pipeline is:
```
1. Collect trajectory data: q, q_dot, tau (actuated joints only)
2. Filter & differentiate to get q_ddot
3. Fill in unactuated joint states by solving constraints
4. Build Newton-Euler regressor matrix Y(q, q_dot, q_ddot)
5. [Optional] Regroup to base parameters via QR decomposition
6. Apply constraint projection to get constrained observation matrix
7. Run IRLS optimization with LMI constraints => identified parameters
```

The optimizer enforces **physical consistency** via a Linear Matrix Inequality (LMI) constraint that guarantees
the identified pseudo-inertia matrix is positive definite (i.e., no negative masses or inconsistent inertia tensors).

---

## Repository Layout

```
ConstrainedSysID/
├── README.md                        # Original project README (do not edit)
├── README_guide.md                  # This file
│
├── SysIDMain.m                      # ← START HERE: user config + entry point
├── SysIDData.m                      # Data loading/processing pipeline (robot-specific)
├── SysIDDigit.m                     # Digit-specific data extractor (replace for new robots)
├── dataProcessing.m                 # Signal filtering, downsampling, differentiation
├── centralDifference.m              # 4th-order finite difference for q_ddot estimation
│
├── QRDecomposition.m                # Base parameter regrouping via QR (Gautier 1990)
├── SysIDAlg.m                       # Algorithm router: picks correct variant
│
├── SysIDAlgFull.m                   # IRLS, all params, with GLS normalization
├── SysIDAlgFullNoWeights.m          # IRLS, all params, without normalization
├── SysIDAlgFMParamOnly.m            # IRLS, friction+motor only, with GLS normalization
├── SysIDAlgFMParamOnlyNoWeights.m   # IRLS, friction+motor only, without normalization
├── SysIDAlgInParamOnly.m            # IRLS, inertial only, with GLS normalization
├── SysIDAlgInParamOnlyNoWeights.m   # IRLS, inertial only, without normalization
│
└── Supplementary_Materials/
    └── Contrained_SysID_Appendix.pdf  # Mathematical derivations appendix
```

> **Execution order:** `SysIDMain` → `SysIDData` → `SysIDDigit` + `dataProcessing` → `SysIDAlg` → one of the 6 variants.

---

## Required Toolboxes

| Toolbox | Used For |
|---|---|
| **Optimization Toolbox** | `fmincon`, `createOptimProblem` |
| **Global Optimization Toolbox** | `MultiStart`, `run(gs, ...)` |
| **Signal Processing Toolbox** | `butter`, `filtfilt`, `downsample` |
| **Robotics / custom model** | `createFloatingBaseFixedSpringDigit`, `Yphi`, `model.fillUnactJoints` (Digit-specific) |

The Digit-specific functions (`createFloatingBaseFixedSpringDigit`, `Yphi`, `model.getJLeft`, etc.) are **not** part
of this repository — they come from Agility Robotics' MATLAB model library. You will need to add this to your path.

---

## Step-by-Step Usage Guide

### Step 0 — Prerequisites

1. Add the `ConstrainedSysID/` folder to your MATLAB path.
2. Add your robot model library to your MATLAB path (for Digit: the ROAHM floating-base model).
3. Place your raw data files (e.g., `Digit_datap11.txt`) in a location accessible to MATLAB.

---

### Step 1 — Configure `SysIDMain.m`

Open `SysIDMain.m`. This is the **only file you need to edit** for a standard run.
Set the following sections:

#### 1a. Algorithm flags
```matlab
k = 0.3;              % IRLS outlier rejection threshold (residuals > k are ignored)
tol = 0.0001;         % Convergence tolerance for all while loops
MS1 = 3;              % Number of MultiStart restarts for main optimization
MS2 = 3;              % Number of MultiStart restarts for friction exponent optimization
regroup = false;      % true: use QR regrouping to reduce to identifiable parameters
SearchAll = true;     % true: identify inertial + friction + motor params simultaneously
SearchFM = true;      % true: identify friction/motor only (if SearchAll = false)
SearchAlpha = false;  % true: also optimize the friction exponent alpha
includeOffset = false;% true: add a constant torque offset beta to friction model
includeFMDynamics = true; % true: include friction/motor in model (used when SearchAll=false)
includeConstraints = true;% true: system has kinematic constraints (set false for arms)
includeWeighting = true;  % true: use GLS normalization (recommended)
constraintVariant = 1;    % 1: general constraint projection, 2: fully actuated projection
```

**Quick decision guide for `SearchAll` / `SearchFM`:**
| Goal | SearchAll | SearchFM |
|---|---|---|
| Identify everything from scratch | `true` | (ignored) |
| Inertial params known, identify friction/motor | `false` | `true` |
| Friction/motor known, identify inertial | `false` | `false` |

**constraintVariant guide:**
| System type | constraintVariant |
|---|---|
| General constraints (most systems) | `1` |
| Fully actuated (nu == nc), computationally lighter | `2` |

#### 1b. Initial conditions and bounds
```matlab
% If you have prior values (e.g., from the manufacturer), fill these in.
% Otherwise leave as empty [], and fmincon will search freely within bounds.
X0_ip = [];   % Initial inertial parameters  [Ixx, Ixy, Ixz, Iyy, Iyz, Izz, hx, hy, hz, m] per link
X0_fm = [];   % Initial friction/motor params [Im_1,...,Im_n, Fc_1,Fv_1,beta_1,...,Fc_n,Fv_n,beta_n]
lb_ip = []';  % Lower bounds for inertial params (same structure as X0_ip)
ub_ip = []';  % Upper bounds for inertial params
lb_fm = []';  % Lower bounds for friction/motor params
ub_fm = []';  % Upper bounds for friction/motor params
```

> **Tip:** Setting reasonable bounds dramatically improves optimization convergence.
> At minimum, enforce `lb_ip` to respect physical constraints (masses > 0, etc.).

#### 1c. fmincon options
```matlab
options = optimoptions('fmincon', ...
    'MaxFunctionEvaluations', 30000, ...
    'Display', 'iter', ...
    'Algorithm', 'sqp', ...
    'SpecifyObjectiveGradient', true, ...
    'SpecifyConstraintGradient', true, ...
    'UseParallel', true);  % Parallel requires Parallel Computing Toolbox
```

---

### Step 2 — Configure `SysIDData.m` (robot-specific)

`SysIDData.m` is called automatically by `SysIDMain.m` via `run('SysIDData.m')`. Edit it to match your data:

```matlab
% Paths to your raw data files
datapaths = {'Digit_datap11.txt', 'Digit_datap21.txt'};

% Time window: crop data to [cutTimeBefore, cutTimeAfter] seconds
cutTimeBefore = [5 5];   % One value per file
cutTimeAfter  = [10 14]; % One value per file

% Approximate number of downsampled data points (after filtering)
pointNumber = 2100;

% Butterworth filter cutoff frequencies (Hz)
cfT = [33 33];  % Torque
cfV = [18 18];  % Velocity
cfA = [15 15];  % Acceleration

% Butterworth filter orders
orderT = 4; orderV = 4; orderA = 4;

% Digit-specific: which limb to identify
bodypart = "LeftLeg";  % Options: "LeftLeg", "RightLeg", "LeftArm", "RightArm"
```

The script calls `SysIDDigit` (Step 1: raw extraction) and `dataProcessing` for each file,
concatenates results, then calls `SysIDDigit` (Step 2: build observation matrix).

---

### Step 3 — Run

From the MATLAB command window:
```matlab
cd('<path-to-ConstrainedSysID>')
SysIDMain
```

**What happens internally:**
```
SysIDMain
  └─> run('SysIDData.m')
        └─> SysIDDigit(..., step=1)   % Extract raw q, qdot, tau from .txt files
        └─> dataProcessing(...)       % Filter + downsample, compute q_ddot via centralDifference
        └─> SysIDDigit(..., step=2)   % Fill unactuated joints, build W_ip and T
  └─> SysIDAlg(...)                   % Dispatch to the correct algorithm variant
        └─> [optional] QRDecomposition(W_ip, ones(...))  % If regroup=true
        └─> SysIDAlgFull(...)         % (or whichever variant matches your flags)
              └─> fmincon + MultiStart (LMI constrained IRLS)
```

### Step 4 — Outputs

```matlab
[X, Wfull, alphanew, Ginv] = SysIDAlg(...)
```

| Output | Type | Description |
|---|---|---|
| `X` | `p_full x 1` vector | **Identified parameters** (inertial + motor + friction, ordered per parameter vector structure below) |
| `Wfull` | `n*m x p_full` matrix | Full observation matrix (all params combined) |
| `alphanew` | `n x 1` vector | Identified friction exponent per joint (1.0 if `SearchAlpha=false`) |
| `Ginv` | `10n x 10n` matrix | Inverse regrouping map (only meaningful if `regroup=true`) |

---

## File Reference

### `SysIDMain.m` — Entry Point

**Role:** User-facing configuration script. The only file you should edit for a standard run.

**What it does:**
- Declares all algorithm options as named variables (`k`, `tol`, `MS1`, `regroup`, `SearchAll`, etc.)
- Packs them into `AlgOptions = {k, tol, MS1, MS2, regroup, SearchAll, SearchFM, SearchAlpha, ...}`
- Sets initial parameter guesses (`X0_ip`, `X0_fm`) and bounds (`lb_ip`, `lb_fm`, `ub_ip`, `ub_fm`)
- Calls `run('SysIDData.m')` to populate `W_ip`, `T`, `data`, `dataFull`, `J`, `na_idx`, `nu_idx`
- Calls `SysIDAlg(...)` and receives identified parameters `X`, `Wfull`, `alphanew`, `Ginv`

**Inputs:** _(none — it is a script, not a function)_
**Outputs:** Workspace variables `X`, `Wfull`, `alphanew`, `Ginv`

---

### `SysIDData.m` — Data Loading Pipeline

**Role:** Robot-specific data acquisition script. Called by `SysIDMain` via `run('SysIDData.m')`.

**What it does:**
1. Specifies raw data file paths, time windows, downsampling target, and filter parameters
2. For each data file:
   - Calls `SysIDDigit(..., step=1)` → extracts raw `pos`, `vel`, `torq`, `t`
   - Calls `dataProcessing(...)` → filters data and computes `q_ddot`
3. Concatenates data across all files
4. Calls `SysIDDigit(..., step=2)` → fills unactuated joints, builds `W_ip`, `T`, `data`, `dataFull`

**Robot-specific parts (must be changed for a new robot):**
- The `datapaths` variable (your file format)
- The calls to `SysIDDigit` (replace with your own data extractor)
- The `bodypart` variable (Digit-specific)

**Outputs** (written to workspace):
| Variable | Size | Description |
|---|---|---|
| `W_ip` | `n*m x 10n` | Inertial parameter observation matrix |
| `T` | `n*m x 1` | Measured torque vector (stacked over all timesteps) |
| `data` | `n x 3m` | Processed `[q, q_dot, q_ddot]` for the identified limb |
| `dataFull` | `n_full x 3m` | Same but for all joints (needed by constraint Jacobian) |
| `J` | function handle | Constraint Jacobian `J(q)` returning `nc x n` matrix |
| `na_idx` | `1 x na` | Indices of actuated joints |
| `nu_idx` | `1 x nu` | Indices of unactuated joints |

---

### `SysIDDigit.m` — Digit-Specific Data Extractor

**Role:** Handles Digit's proprietary log format and kinematic model. **Replace this for a new robot.**

**Function signature:**
```matlab
[t_, pos, vel, torq_, J, na_idx, nu_idx, W_ip, T, data, dataFull] = ...
    SysIDDigit(datapaths, bodypart, step, q, qdot, qddot, torq, i, constraintVariant)
```

**Two-phase operation:**

**Step 1** (`step=1`): Raw data extraction
- Reads `.txt` log file (columns: q[1:20], qd[1:20], tau_obs[1:20], time)
- Deduplicates timestamps
- Returns raw `pos`, `vel`, `torq_` for the joints of the selected `bodypart`
- Returns `J` (constraint Jacobian function), `na_idx`, `nu_idx`

**Step 2** (`step=2`): Observation matrix construction
- Takes filtered/downsampled `q, qdot, qddot, torq` from `dataProcessing`
- Fills unactuated joint states using `model.fillUnactJoints(...)` (satisfies constraints)
- Builds inertial regressor matrices by calling `Yphi(model, q, qd, qdd)` at each timestep
- Returns `W_ip` (observation matrix), `T` (torque vector), `data`, `dataFull`

**Supported body parts for Digit:**
| bodypart | Joints | Actuated (`na_idx`) | Unactuated (`nu_idx`) |
|---|---|---|---|
| `"LeftLeg"` | Rows 1–6 in data | [1,2,3,4,15,16] | [5,6,7,19,20,21,22,27,28] |
| `"RightLeg"` | Rows 7–12 in data | [8,9,10,11,17,18] | [12,13,14,23,24,25,26,29,30] |
| `"LeftArm"` | Rows 13–16 | [] (all actuated) | [] |
| `"RightArm"` | Rows 17–20 | [] (all actuated) | [] |

---

### `dataProcessing.m` — Signal Processing

**Role:** General-purpose data conditioning. **Does not need to change for new robots.**

**Function signature:**
```matlab
[i, t, q, qdot, qddot, torq] = dataProcessing(time, pos, vel, torque, ...
    cfT, cfV, cfA, cutTimeBefore, cutTimeAfter, orderT, orderV, orderA, pointNumber)
```

**Processing pipeline:**
1. Clips time to `[cutTimeBefore, cutTimeAfter]` seconds
2. Computes `q_ddot` from `vel` using `centralDifference` (4th-order, with filter)
3. Applies 4th-order Butterworth low-pass filter to torque (at `cfT` Hz) and velocity (at `cfV` Hz)
4. Downsamples all signals by factor `ceil(n_points / pointNumber)`

**Key inputs:**
| Input | Description |
|---|---|
| `cfT`, `cfV`, `cfA` | Cutoff frequencies (Hz) for torque, velocity, accel filters |
| `orderT/V/A` | Butterworth filter orders (4 is standard) |
| `cutTimeBefore/After` | Time window to keep (seconds) |
| `pointNumber` | Target number of data points after downsampling |

---

### `centralDifference.m` — Numerical Differentiation

**Role:** Computes `dy/dt` using a 4th-order finite difference scheme with pre-filtering.
**Does not need to change for new robots.**

**Function signature:**
```matlab
dydt = centralDifference(y, dt, cfA, orderA)
```

**Scheme:**
- Interior points: `[-1/12, 2/3, 0, -2/3, 1/12]` (4th-order central difference)
- First 2 and last 2 points: 4th-order forward / backward difference
- Pre-filters `y` with a Butterworth filter at `cfA` Hz before differentiating

**Used by:** `dataProcessing.m`

---

### `QRDecomposition.m` — Parameter Regrouping

**Role:** Computes the base inertial parameters (identifiable subset) from the full observation matrix.
Implements the numerical regrouping algorithm from Gautier 1990.

**Function signature:**
```matlab
[Aid, Ad, Kd, beta, Ginv, dim_id, dim_d] = QRDecomposition(ObservationMatrix, InParam)
```

**What it computes:**
| Output | Description |
|---|---|
| `Aid` | `10n x b` selection matrix for independent (identifiable) parameters |
| `Ad` | `10n x d` selection matrix for dependent (unidentifiable) parameters |
| `Kd` | `b x d` regrouping transformation matrix |
| `beta` | Base parameter vector (what you actually identify) |
| `Ginv` | `10n x 10n` inverse map: `theta = Ginv * [beta; 0]` |
| `dim_id`, `dim_d` | Counts of identifiable and dependent parameters |

**When to use:** Set `regroup = true` in `SysIDMain` when your system has redundant inertial parameters
(almost always the case for legged robots). This reduces the number of optimization variables
from `10n` to `b << 10n`, making the problem much better conditioned.

**Used by:** `SysIDAlg.m` (when `regroup=true`), and internally by the algorithm variants.

---

### `SysIDAlg.m` — Algorithm Router

**Role:** Dispatches to the correct algorithm variant based on `AlgOptions`.
**You never call this directly** — `SysIDMain` calls it.

**Function signature:**
```matlab
[X, Wfull, alphanew, Ginv] = SysIDAlg(AlgOptions, na_idx, nu_idx, J, lb, ub, lba, uba, ...
                                        T, data, dataFull, W_ip, X0_1, X_ip, X_fm, options)
```

**Routing logic (in order of priority):**
```
SearchAll = true  AND  includeWeighting = true   →  SysIDAlgFull
SearchAll = true  AND  includeWeighting = false  →  SysIDAlgFullNoWeights
SearchFM  = true  AND  includeWeighting = true   →  SysIDAlgFMParamOnly
SearchFM  = true  AND  includeWeighting = false  →  SysIDAlgFMParamOnlyNoWeights
else              AND  includeWeighting = true   →  SysIDAlgInParamOnly
else              AND  includeWeighting = false  →  SysIDAlgInParamOnlyNoWeights
```

Also calls `QRDecomposition(W_ip, ...)` first if `regroup = true`.

### SysID Algorithm Variants

All six variants share the same core IRLS structure but differ in which parameters they identify
and whether they include GLS (Generalized Least Squares) noise normalization.

| File | Identifies | GLS Normalization | Use When |
|---|---|---|---|
| `SysIDAlgFull.m` | Inertial + Motor + Friction (+offset) | ✅ Yes | Default: full identification, noisy data |
| `SysIDAlgFullNoWeights.m` | Inertial + Motor + Friction (+offset) | ❌ No | Fast run, clean/simulated data |
| `SysIDAlgFMParamOnly.m` | Motor + Friction only | ✅ Yes | Inertial params known (e.g. from manufacturer) |
| `SysIDAlgFMParamOnlyNoWeights.m` | Motor + Friction only | ❌ No | Same, but faster without normalization |
| `SysIDAlgInParamOnly.m` | Inertial only | ✅ Yes | Friction/motor known; inertial params unknown |
| `SysIDAlgInParamOnlyNoWeights.m` | Inertial only | ❌ No | Same, but faster without normalization |

#### Internal algorithm structure (same for all variants with GLS)

```
[Outer loop: converge friction exponent alpha (only if SearchAlpha=true)]
  [Middle loop: converge IRLS weights D]
    [Inner loop: converge GLS covariance matrix Omega]
      1. Normalize W and T by sqrt(Omega)            % GLS whitening
      2. Apply IRLS weight matrix D                   % Outlier rejection
      3. Solve: min ||D*(Omega^-0.5)*W*x - D*(Omega^-0.5)*T||^2
               subject to: LMI(x) >= 0  (physical consistency)
                           lb <= x <= ub
         via fmincon + MultiStart
      4. Update Omega from residuals
    [end inner loop]
    Update D using T-class hard redescender:
      D_i = 0 if |residual_i| > k, else 1
  [end middle loop]
  [If SearchAlpha: solve second fmincon for alpha and friction params]
[end outer loop]
```

#### LMI Constraint
Each variant enforces that the **pseudo-inertia matrix** `L_j` for each link `j` is positive definite:
```
L_j = [ (tr(I)/2)*eye(3) - I    h  ]  >= 0   for each link j
      [        h'                m  ]
```
where `I` is the inertia tensor, `h = m*c` is the first moment of mass, and `m` is the mass.
This is encoded in the `lmiconDet` subfunction, which also provides the analytic gradient.

---

## Algorithm Option Reference

The `AlgOptions` cell array passed to `SysIDAlg` has the following indexed entries:

| Index | Variable | Type | Description |
|---|---|---|---|
| 1 | `k` | double | IRLS outlier threshold. Residuals > `k` get weight 0. Typical: 0.1–0.5 |
| 2 | `tol` | double | While-loop convergence threshold. Typical: 1e-4 |
| 3 | `MS1` | int | MultiStart restarts for main optimization. Higher = more thorough but slower |
| 4 | `MS2` | int | MultiStart restarts for friction exponent optimization |
| 5 | `regroup` | bool | Use QR-based parameter regrouping |
| 6 | `SearchAll` | bool | Identify all parameters (inertial + friction + motor) |
| 7 | `SearchFM` | bool | Identify friction + motor only (when `SearchAll=false`) |
| 8 | `SearchAlpha` | bool | Also identify friction exponent alpha |
| 9 | `includeOffset` | bool | Include constant torque offset beta in friction model |
| 10 | `includeFMDynamics` | bool | Include friction/motor contribution when doing inertial-only ID |
| 11 | `includeConstraints` | bool | System has kinematic constraints |
| 12 | `constraintVariant` | int | 1=general projection, 2=fully-actuated projection |
| 13 | `includeWeighting` | bool | Use GLS covariance normalization (recommended) |

---

## Parameter Vector Structure

The output `X` returned by `SysIDAlg` has the following layout.
For a system with `n` joints:

### Inertial parameters (first `10*n` entries):
For each link `j = 1, ..., n`:
```
X(10*(j-1)+1)  = Ixx_j   (inertia tensor element)
X(10*(j-1)+2)  = Ixy_j
X(10*(j-1)+3)  = Ixz_j
X(10*(j-1)+4)  = Iyy_j
X(10*(j-1)+5)  = Iyz_j
X(10*(j-1)+6)  = Izz_j
X(10*(j-1)+7)  = hx_j = m_j * cx_j  (first moment of mass, x)
X(10*(j-1)+8)  = hy_j = m_j * cy_j
X(10*(j-1)+9)  = hz_j = m_j * cz_j
X(10*(j-1)+10) = m_j   (mass)
```

### Motor parameters (next `n` entries):
```
X(10*n + j) = Im_j   (motor/transmission inertia for joint j)
```

### Friction parameters (next `2*n` or `3*n` entries):
For each joint `j = 1, ..., n` (with `includeOffset = false`):
```
X(11*n + 2*(j-1)+1) = Fc_j   (Coulomb friction coefficient)
X(11*n + 2*(j-1)+2) = Fv_j   (Viscous friction coefficient)
```
With `includeOffset = true`, add `beta_j` as the 3rd entry per joint.

### Friction model:
```
F_friction_j = Fc_j * sign(qdot_j) + Fv_j * sign(qdot_j) * |qdot_j|^alpha_j  [+ beta_j]
F_motor_j    = Im_j * qddot_j
```

---

## Adapting to a New Robot

To use this framework on a robot other than Digit:

1. **Replace `SysIDDigit.m`** with your own function having the same output signature:
   ```matlab
   [t_, pos, vel, torq_, J, na_idx, nu_idx, W_ip, T, data, dataFull] = MyRobotData(...)
   ```
   Your Step 1 should return raw `pos`, `vel`, `torq_`.  
   Your Step 2 should fill unactuated states, build `W_ip` via `Yphi(model, ...)`, and return `T`, `data`, `dataFull`.

2. **Update `SysIDData.m`:**
   - Change `datapaths` to match your file format
   - Replace calls to `SysIDDigit(...)` with calls to your new function
   - Adjust filter cutoffs (`cfT`, `cfV`, `cfA`) and time windows

3. **Set constraint info in `SysIDMain.m`:**
   - If unconstrained: `includeConstraints = false`, `na_idx = 1:n`, `nu_idx = []`
   - If constrained: provide your constraint Jacobian `J = @(q) myJacobian(q)` and fill `na_idx`, `nu_idx`

4. **Everything else** (`QRDecomposition`, `centralDifference`, `dataProcessing`, all `SysIDAlg*`) is **robot-agnostic** and does not need to change.

---

## Known Issues

> [!WARNING]
> **Bug in `SysIDAlgFullNoWeights.m`:** In both `AlgAlpha` and `AlgNoAlpha`, the objective function
> and residual computation reference a variable `W` that is never defined in that scope. It should be `Wfull`.
> This will cause a runtime error if you use `SearchAll = true` AND `includeWeighting = false`.
> The fix: replace `W` with `Wfull` in the objective function handles on those lines.

> [!NOTE]
> **Parallel computing:** `UseParallel = true` in `fmincon` options requires the MATLAB Parallel Computing Toolbox.
> If you do not have this toolbox, set it to `false` (optimization will still work, just slower).

> [!NOTE]
> **`SysIDAlgInParamOnly.m` line 183:** `Wfull = [W_ip Wfm]` will error if `includeFMDynamics = false`
> because `Wfm` is never computed in that code path. This edge case is not guarded.

---

## Citation

If you use this code in your research, please cite:

```bibtex
@article{zhang2024system,
  title={System Identification For Constrained Robots},
  author={Zhang, Bohao and Haugk, Daniel and Vasudevan, Ram},
  journal={arXiv preprint arXiv:2408.08830},
  year={2024}
}
```

Paper: https://arxiv.org/abs/2408.08830  
GitHub: https://github.com/roahmlab/ConstrainedSysID
