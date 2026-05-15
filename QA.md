# ConstrainedSysID — Q&A & Refactoring Notes

> **Purpose:** Running log of questions, answers, and planned modifications discussed during code review.
> Updated live during conversation. Use this as the reference doc before starting refactoring.

**Paper:** [arXiv:2408.08830](https://arxiv.org/abs/2408.08830) — *System Identification For Constrained Robots*
**Code:** `ConstrainedSysID/` folder
**Session started:** 2026-05-13

---

## How to Read This Doc

- **Q:** = Question asked during conversation
- **A:** = Answer (math ↔ code mapping, explanation)
- **MOD:** = Proposed modification or refactoring note
- **STATUS:** = `[ ]` not started / `[/]` in progress / `[x]` done

---

## Paper ↔ Code Quick Reference

| Paper Symbol | Code Variable | File | Description |
|---|---|---|---|
| θ ∈ ℝ^{14n} | `X` | output of `SysIDAlg` | Full parameter vector |
| θ_ip ∈ ℝ^{10n} | `X(1:10*n)` | all alg files | Inertial parameters |
| θ_f ∈ ℝ^{4n} | `X(10*n+1:end)` | all alg files | Friction+motor params |
| π (base params) | `X(1:b)` | when `regroup=true` | Identifiable inertial params |
| G^{-1} (regrouping map) | `Ginv` | `QRDecomposition.m` | θ = Ginv*[π; 0] |
| A_id (independent selector) | `Aid` | `QRDecomposition.m` | Selects identifiable cols |
| A_d (dependent selector) | `Ad` | `QRDecomposition.m` | Selects dependent cols |
| K_d (regrouping transform) | `Kd` | `QRDecomposition.m` | β = Aid'θ + Kd*Ad'θ |
| W(q,q̇,q̈) (full regressor) | `W_ip` | `SysIDData.m` | Inertial observation matrix |
| Y(q,q̇,q̈) (base regressor) | `W_ip*Aid` (= Wb) | algorithm files | Base observation matrix |
| J(q) (constraint Jacobian) | `J` (function handle) | `SysIDDigit.m` | J(q): nc×n matrix |
| G(q) (constraint transform) | `P = [I; -Ju\\Ja]` | algorithm files | Maps q̇_a → q̇ |
| K (null-space projector) | `K = eye(n)-J'*pinv(J')` | algorithm files | Variant 1 projector |
| Ω (covariance matrix) | `Onew` | algorithm files | GLS noise covariance |
| D (IRLS weight vector) | `Dnew` | algorithm files | Outlier rejection weights |
| k (weight threshold) | `k` | `SysIDMain.m` | T-class threshold |
| n (total joints) | `n = length(data(:,1))` | alg files | Total joint count |
| na (actuated joints) | `na = length(na_idx)` | alg files | Actuated joint count |
| nu (unactuated joints) | `length(nu_idx)` | `SysIDDigit.m` | Unactuated joint count |
| m (data points) | `m` | alg files | Number of timesteps |
| L_j ⪰ 0 (LMI constraint) | `lmiconDet(...)` | each alg file | Physical consistency check |
| Fc_j (Coulomb friction) | `X(11*n + 2*(j-1)+1)` | all alg files | Static friction coeff |
| Fv_j (viscous friction) | `X(11*n + 2*(j-1)+2)` | all alg files | Viscous friction coeff |
| Ia_j (motor inertia) | `X(10*n + j)` | all alg files | Transmission inertia |
| α_j (friction exponent) | `alphanew(j)` | all alg files | Friction velocity exponent |
| β_j (offset) | (only if includeOffset) | all alg files | Constant torque offset |

---

## Q&A Log

*(Questions and answers will be added here as the conversation progresses.)*

---

## Planned Modifications / Refactoring Notes

*(Modifications will be noted here as they are discussed.)*

### Known Bugs (pre-existing)

#### BUG-1: `SysIDAlgFullNoWeights.m` — undefined variable `W`
- **Location:** `AlgAlpha` (line ~204) and `AlgNoAlpha` (line ~299)
- **Symptom:** Runtime error when `SearchAll=true` AND `includeWeighting=false`
- **Fix:** Replace `W` with `Wfull` in objective function handles on those lines
- **STATUS:** `[ ]`

#### BUG-2: `SysIDAlgInParamOnly.m` — `Wfm` undefined when `includeFMDynamics=false`
- **Location:** Line 183: `Wfull = [W_ip Wfm]`
- **Symptom:** Error if `includeFMDynamics=false` since `Wfm` is never computed in that branch
- **Fix:** Guard with `if includeFMDynamics`, set `Wfm=[]` otherwise
- **STATUS:** `[ ]`

#### BUG-3: `SysIDAlgFull.m` — typo `inlcludeOffset` (with regroup branch)
- **Location:** `AlgAlpha`, line ~334
- **Symptom:** Error if `regroup=true` AND `SearchAlpha=true`
- **Fix:** Change `inlcludeOffset` to `includeOffset`
- **STATUS:** `[ ]`


---

### MOD-1: Adapt framework for Unitree Go1 quadruped
- **Scope:** Create `SysIDGo1.m`, modify `SysIDMain.m` and `SysIDData.m`
- **Key insight:** Go1 has open kinematic chains (3 actuated joints/leg, 0 passive), so ALL constraint code is disabled
- **Set:** `includeConstraints = false`, `na_idx = [1,2,3]`, `nu_idx = []`, `J = []`
- **Full plan:** See `implementation_plan.md`
- **STATUS:** `[ ]`

