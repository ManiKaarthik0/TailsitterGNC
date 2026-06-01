# Tailsitter GNC — Comprehensive Technical Review

**Reviewer role:** Senior GNC engineer / systems architect / code reviewer
**Scope:** Entire repository on branch `claude/tender-goodall-3WCZ7`
**Date:** 2026-06-01

> Every conclusion below is tied to concrete evidence in the repository
> (`file:line` references). This is an aerospace GNC review, not generic
> software feedback.

---

## 1. Executive Summary

The repository implements the **open-loop 6-DOF dynamics, a stability-augmentation
(SAS) inner loop, an IMU sensor model, and a 6-state attitude EKF** for a small
(2.5 kg) **"tailsitter"** UAV. In practice, what is modeled today is a
**fixed-wing forward-flight aircraft trimmed at 20 m/s** — there is no hover,
no VTOL, and no transition logic, despite the "tailsitter" name.

The work is at an **early-but-credible prototype stage**:

- The **16-state 6-DOF rigid-body model** (`dynamics/rigid_body.m`) is complete
  and structurally sound for cruise flight.
- A **linearization + pole-placement SAS** design exists for decoupled
  lateral/longitudinal axes (`sim/linearize_trim.m`).
- An **IMU model** (`dynamics/imu_simulate.m`) and a **6-state attitude EKF**
  (`dynamics/ekf_attitude.m`) are implemented and wired into a closed-loop
  Simulink model (`closed_loop_sim_est.slx`).

However, the GNC stack has **fundamental gaps and several concrete bugs** that
will block progress:

| Area | Status | Headline issue |
|---|---|---|
| Guidance | **Absent (0%)** | No outer loop, no reference/trajectory, no waypoint logic |
| Navigation/EKF | **Partial, with defects** | No gyro-bias state; yaw unobservable; one Jacobian sign error; discrete filter run at continuous sample time |
| IMU model | **Low fidelity** | Accelerometer = gravity tilt only (ignores kinematic/Coriolis accel); constant bias; no scale factor / misalignment / quantization |
| Control | **SAS only** | Stability augmentation, no tracking, no actuator/limit-aware design, sign of gains determined empirically |
| Dynamics | **Good (cruise only)** | No hover / propeller / transition aerodynamics |
| Simulation | **Fragile** | Hardcoded Windows paths; `randn` and a discrete EKF inside continuous variable-step `ode45`; build artifacts committed |

**Top three critical risks:** (1) running a discrete EKF and stochastic IMU noise
inside a continuous variable-step solver, (2) attitude estimator with no
gyro-bias state and an unobservable yaw channel, and (3) the project is branded
a tailsitter but contains none of the defining tailsitter physics/modes.

---

## 2. Repository Architecture

```
TailsitterGNC/
├── README.md                         # 2 lines — essentially empty
└── Tailsitter_GNC/
    ├── params/params.m               # mass, inertia, aero derivatives, layout
    ├── dynamics/
    │   ├── rigid_body.m              # 16-state 6-DOF EOM (core plant)
    │   ├── aero_forces.m             # forces/moments, stall blend, propwash
    │   ├── aero_angles.m             # alpha, beta from body velocity
    │   ├── rot_matrix.m              # ZXY body→world DCM
    │   ├── elevon_mix.m              # d1,d2 → delta_e, delta_a
    │   ├── imu_simulate.m            # gyro + accel sensor model
    │   ├── ekf_attitude.m           # 6-state attitude EKF
    │   ├── fixedwing_sfun.m         # L2 S-function wrapping rigid_body
    │   ├── imu_sfun.m               # L2 S-function wrapping imu_simulate
    │   └── ekf_sfun.m              # L2 S-function wrapping ekf_attitude
    ├── sim/
    │   ├── run_sim.m                 # open-loop trim validation (ode45)
    │   ├── linearize_trim.m         # numerical Jacobian + SAS pole placement
    │   ├── test_ekf.m              # standalone EKF unit test
    │   └── check_modes.m            # Simulink-based modal analysis (stub)
    ├── openloop_sim.slx              # plant + scopes only
    ├── closed_loop_sim_est.slx      # plant + IMU + EKF + SAS (full GNC loop)
    ├── openloop_sim/openloop_sim.slx # DUPLICATE of openloop model
    └── slprj/, *.slxc, *.asv,        # build/cache/backup artifacts (should be
        *.autosave, *.bak, *.zip      # .gitignore'd, not committed)
```

### Data-flow / dependency map (closed-loop EKF model)

Block inventory extracted from `closed_loop_sim_est.slx`
(`simulink/systems/system_root.xml`): three Level-2 MATLAB S-Functions
(`fixedwing_sfun`, `imu_sfun`, `ekf_sfun`), one `Integrator1`, gains
`K_lat`, `K_long`, `0.5`, `-0.5`, plus Mux/Demux/Sum/ToWorkspace.

```
                 ┌─────────────────────────────────────────────┐
                 │                                             │
   U (4×1) ──►(+)─► fixedwing_sfun ─► dX ─►[Integrator1]─► X (16×1) ─┬─► To Workspace
   rate cmd  ▲      (rigid_body)                                    │
             │                                                      │
             │   ┌──────────────── SAS feedback ───────────────┐    │
             │   │  K_lat·[vy,p,r,phi]   (→ delta_a, dT)        │    │
             └───┤  K_long·[vx,vz,theta,q] (→ delta_e rate)     │◄───┤ (TRUE state)
                 └──────────────────────────────────────────────┘    │
                                                                      │
                 X_true ─► imu_sfun ─► z_gyro(3), z_accel(3) ─► ekf_sfun ─► X_est(6)
                          (imu_simulate)                       (ekf_attitude)
```

**Critical architectural observation:** the SAS feedback in the model is driven
by the **true plant state X**, not by the **EKF estimate X_est**. The EKF output
(`X_est`, 6×1) is logged (`out.x_out2` in `sim/run_sim.m:307`) but, based on the
gain wiring (`K_lat`/`K_long` fed from the integrated state), the control loop is
**not actually closed on the estimator**. The estimator is currently a
*passenger*, not part of the control path. This must be resolved before any
navigation claim can be made.

---

## 3. Vehicle Dynamics Analysis

**Source:** `dynamics/rigid_body.m`, `dynamics/aero_forces.m`,
`dynamics/aero_angles.m`, `dynamics/rot_matrix.m`.

### Frames & conventions
- **Body frame** x-forward, y-right, z-down (standard aircraft body axes — confirmed
  by gravity projection `F_grav = mg[-sinθ; sinφcosθ; cosφcosθ]`, `rigid_body.m:36`).
- **World frame** NED (altitude printed as `-X(3)`, `run_sim.m:48`; initial
  `z = -100`).
- **Attitude parameterization:** Euler **ZXY** — `R = Rz(ψ)·Rx(φ)·Ry(θ)`
  (`rot_matrix.m:12`). This is a **non-standard sequence** (the aerospace default
  is ZYX yaw-pitch-roll). It is internally self-consistent (kinematics, gravity,
  and accel model all use ZXY), but it is unusual and undocumented — a likely
  source of future confusion and a singularity at **φ = ±90°** (note the
  `inv_cph = 1/sqrt(cos²φ + 1e-6)` guard, `rigid_body.m:72`). For a *tailsitter*,
  which deliberately operates near 90° pitch during hover, **any Euler
  parameterization is the wrong long-term choice** — quaternions are required.

### Force / moment build-up (`aero_forces.m`)
- Lift: linear `CL0 + CL_α·α` blended via a sigmoid (`σ`) into a flat-plate
  post-stall model `2·sign(α)·sin²α·cosα` (`aero_forces.m:21-26`). Reasonable
  for a high-α tailsitter wing.
- Drag: `CD0 + k_ind·CL²` (`:29`).
- Lateral/directional and pitch derivatives applied with standard non-dimensional
  rate terms `(b/2V)`, `(c/2V)` (`:32-41`).
- **Propwash:** `q_eff = q_inf + k_prop·T_total·T_pw + q_pw` (`:17`) adds a
  thrust-dependent and a **constant** dynamic-pressure floor `q_pw = 10 Pa`. The
  constant term is non-physical at V→0 and is clearly a forward-flight crutch.
- **Reaction yaw** from differential thrust via `k_rxn·(T1−T2)` in `Cn` (`:41`) —
  the only nod to multirotor-style control authority.

### Equations of motion (`rigid_body.m`)
- Translational (body frame, with Coriolis): correct cross-product form
  `dvx = Fx/m + r·vy − q·vz`, etc. (`:53-55`).
- Rotational: full Euler equations **including the Ixz cross-product** via
  `Γ = Ixx·Izz − Ixz²` (`:58-67`). Correctly implemented (currently `Ixz = 0`,
  `params.m:7`, so it reduces to diagonal).
- Actuators modeled as **first-order integrator states** with rate saturation
  (`:17-23, 83-86`): `T1,T2` (thrust) and `d1,d2` (elevon positions). Good — gives
  the controller rate commands and bandwidth limiting.

### Findings
- ✅ Cruise EOM are complete and dimensionally consistent.
- ⚠️ `alpha_dot ≈ wy` is an explicit placeholder (`:31-33`) feeding the
  `Cm_αdot` term; acceptable for trim, wrong for dynamic pitch response.
- ⚠️ Thrust moment arm disabled (`M_total = M_aero`, `:50`) — no pitch/no
  thrust-vectoring moment, even though a tailsitter's primary hover control IS
  thrust differential about an offset.
- ❌ **No hover / propeller-momentum / transition aerodynamics.** At V≈0 the
  aero model degenerates (the `1e-8` airspeed floor and constant `q_pw` mask it).
  This is the single largest dynamics gap relative to the "tailsitter" objective.

---

## 4. 16-State Model Analysis

State vector (`rigid_body.m:3`, `params.m:57`):

| idx | symbol | meaning | frame/units |
|----|--------|---------|-------------|
| 1–3 | x, y, z | position | NED [m] |
| 4 | ψ | yaw | [rad] (ZXY) |
| 5 | φ | roll | [rad] |
| 6 | θ | pitch | [rad] |
| 7–9 | vx, vy, vz | translational velocity | **body** [m/s] |
| 10–12 | wx, wy, wz (p,q,r) | angular rate | body [rad/s] |
| 13–14 | T1, T2 | thrust (per motor) | [N] (actuator state) |
| 15–16 | d1, d2 | elevon deflection | [rad] (actuator state) |

Input `U = [Ṫ1, Ṫ2, ḋ1, ḋ2]` (actuator **rates**, `params.m:58`).

**Assessment.** The 12 rigid-body states (position, attitude, body velocity, body
rate) are the textbook 6-DOF set. Appending the **4 actuator states** to make 16
is a sound choice: it lets the controller command rates and makes actuator
bandwidth part of the plant. The ordering quirk — Euler stored as
**[ψ, φ, θ]** rather than the conventional [φ, θ, ψ] — is consistent throughout
(EKF, IMU, kinematics) but is a documentation hazard and should be called out in
a header/spec.

**Concern:** velocity is expressed in the **body frame** while position is NED;
this is standard but means the EKF/navigation layer must carry the DCM correctly.
The current 6-state EKF does **not** estimate position or velocity at all
(Section 7), so there is presently **no full 16-state navigation filter** —
only attitude.

---

## 5. Navigation System Analysis

**Intended architecture (inferred):** loosely-coupled attitude estimation from
IMU, feeding (eventually) the control loop. Today only **attitude + body rate**
are estimated.

- **Estimator:** 6-state EKF, `X_est = [ψ, φ, θ, p, q, r]` (`ekf_attitude.m:1`).
- **Process model:** ZXY Euler kinematics for the angles; **rates modeled as
  constant** (random-walk, `dwx=dwy=dwz=0`, `:29`). This is a *total-state*
  formulation that treats the gyro as a **measurement**, not as a propagation
  input.
- **Measurement vector:** `z = [z_gyro(3); z_accel(3)]`, processed as **two
  sequential updates** (gyro then accel, `:57-95`).
  - Gyro update: `H_g = [0₃ , I₃]` — direct rate measurement (`:58`).
  - Accel update: nonlinear gravity/tilt model
    `[g·sinθ ; −g·sinφ·cosθ ; −g·cosφ·cosθ]` (`:76-78`).
- **Covariance:** `P_pred = F·P·Fᵀ + Q`; standard `(I−KH)P` update with `pinv(S)`,
  symmetrization, and a `1e-9·I` floor (`:55-99`).

### Navigation findings (detailed EKF review in Section 7)
- ❌ **No position/velocity estimation** → not a navigation filter, only attitude.
- ❌ **Yaw (ψ) is unobservable** from gyro + accelerometer. The accel measures
  only the gravity direction → constrains φ and θ but carries **zero information
  about ψ**. With no magnetometer/GPS, ψ is pure dead-reckoning of an integrated,
  biased gyro and will drift without bound. `H_a` correctly has a zero first
  column for ψ — confirming the structural unobservability.
- ❌ **No gyro-bias state** while the IMU injects a constant bias (Section 6) →
  guaranteed steady-state attitude error.
- **Performance expectation as written:** roll/pitch bounded to within accel-noise
  tilt accuracy (~a few deg, degraded by the bias-into-rate leak); yaw unbounded
  drift. `test_ekf.m` only exercises a benign φ-sinusoid with θ=const and reports
  RMS — it does **not** test yaw drift or bias, so the reported "good" RMS is
  not representative.

---

## 6. IMU Review (`dynamics/imu_simulate.m`)

### Models implemented
| Effect | Implemented? | Evidence |
|---|---|---|
| Gyro white noise | ✅ `σ=0.005 rad/s` | `:19, :26` |
| Gyro constant bias | ✅ `[0.003,−0.002,0.001]` | `:20, :26` |
| Accel white noise | ✅ `σ=0.02 m/s²` | `:23, :33` |
| Accel = gravity projection | ✅ (tilt only) | `:29-31` |
| Accel bias | ❌ | — |
| Bias drift / random walk | ❌ (bias is constant) | — |
| Scale factor errors | ❌ | — |
| Axis misalignment / cross-axis | ❌ | — |
| Temperature dependence | ❌ | — |
| Quantization / saturation | ❌ | — |
| g-sensitivity, run-to-run bias | ❌ | — |

### Physical / numerical correctness
- **Sign convention correct:** accelerometer outputs **specific force**
  (= −gravity in body for a quasi-static vehicle), matching the negative of the
  body-gravity vector used in the plant. Internally consistent with the EKF accel
  model. ✅
- ❌ **Major fidelity gap — accelerometer ignores kinematic acceleration.**
  True specific force is `f = R'(v̇_world) − g_body = (v̇_body + ω×v_body) − g_body`.
  The model returns **only** `−g_body`. During any maneuver (and even in the
  20 m/s trim with the Coriolis terms active) the simulated accel is wrong, and —
  more importantly — it *artificially validates* the EKF's "accel = tilt"
  assumption. In real flight the accel sees thrust/centripetal acceleration and
  the tilt assumption breaks; this model will give a falsely optimistic EKF.
- ❌ **Constant bias not estimated** anywhere → it propagates through gyro
  integration into attitude.
- ⚠️ **No `dt` use:** the `dt` argument is accepted but unused (`:1`), so noise is
  **not** sample-rate scaled. White-noise PSD should scale as `σ/√dt`; as written
  the discrete noise is rate-independent, which is incorrect when the block runs at
  a variable rate (Section 8).
- ⚠️ Sample-time assumption (100 Hz) is implicit only; the S-function hardcodes
  `dt=0.01` (`imu_sfun.m:22`) but declares `SampleTimes=[0 0]` (continuous).

### Recommendations
1. Add the **kinematic/Coriolis term** to the accel: pass `v̇_body, ω, v` and
   compute `f = v̇_body + ω×v − g_body`.
2. Augment with **accel bias + bias random walk**, **scale factor**, **3×3
   misalignment**, and optional **quantization/saturation** so the IMU matches a
   real MEMS datasheet (e.g., specify in °/hr bias instability, VRW/ARW).
3. Scale white noise by `1/√dt` and **run the block at a fixed discrete sample
   time** (e.g., `[0.01 0]`).
4. Make all parameters fields of `P` instead of magic numbers in the function body.

---

## 7. EKF Review (`dynamics/ekf_attitude.m`)

### State / model summary
- State `x = [ψ, φ, θ, p, q, r]ᵀ`.
- Process: angles via ZXY kinematics (`:24-26`), rates constant (`:29`).
- `F = ∂f/∂x` computed analytically (`:31-52`); `P⁻ = F P Fᵀ + Q` (`:55`).
- Sequential updates: gyro (linear `H_g=[0 I]`, `:58`) then accel
  (nonlinear `H_a`, `:81-87`).
- Gains via `pinv(S)`; covariance symmetrized + `1e-9` floor (`:63-99`).

### Mathematical correctness — issues found

1. **❌ Jacobian sign error in `F(3,3)` (∂θ̇/∂θ).**
   With `θ̇ = q + p·sinθ·tanφ − r·cosθ·tanφ`, the correct partial is
   `∂θ̇/∂θ = p·cosθ·tanφ + r·sinθ·tanφ`.
   The code (`ekf_attitude.m:47`) has
   `F(3,3) = dt·(p·cosθ·tanφ − r·sinθ·tanφ)` — **wrong sign on the `r` term**
   (`−` should be `+`; `d/dθ(−cosθ) = +sinθ`). Small in magnitude near trim but a
   genuine linearization error that degrades consistency and will matter at
   higher rates.
   *(For reference, the adjacent `F(3,2)` term carries a "fixed" comment, so the
   author was already debugging this Jacobian — this one slipped through.)*

2. **❌ No gyro-bias states.** The estimator cannot observe or remove the
   constant `[0.003,−0.002,0.001] rad/s` injected by the IMU. Standard practice is
   a 9-state filter `[attitude(3), gyro_bias(3), (accel_bias 3)]` or an
   error-state/MEKF formulation. As-is, bias leaks straight into the rate estimate
   (`H_g=[0 I]` makes `p,q,r` track the *biased* gyro) and integrates into attitude.

3. **❌ Yaw unobservable** (see Section 5). `H_a` column 1 is identically zero;
   gyro update only sees rates. Nothing bounds ψ. Needs a heading aid
   (magnetometer / GPS course / dual-antenna).

4. **⚠️ Total-state vs error-state design.** Using the gyro as a *measurement*
   while propagating rates as constant is unusual and statistically inferior to
   the standard **gyro-as-input** propagation (`θ̇ = f(att, ω_gyro − b)`). It
   couples measurement noise into the dynamics and inflates `Q` requirements.

5. **⚠️ Covariance update form.** `(I−KH)P` is used rather than the
   **Joseph form** `(I−KH)P(I−KH)ᵀ + KRKᵀ`. The `pinv`, symmetrization and
   `1e-9` floor are band-aids; Joseph form would be the principled fix and is
   cheap at 6 states.

6. **⚠️ Tuning.** `Q = diag([1e-4×3, 1e-5×3])`, `R_gyro = (0.005)²`,
   `R_accel = (0.02)²` (`ekf_sfun.m:53-55`) — `R` matches the IMU `σ`'s exactly
   (consistent), but `Q` is ad hoc and there is **no `Q` term for the unmodeled
   bias / kinematic accel**, so the filter is over-confident. Recommend NEES/NIS
   consistency checks.

7. **⚠️ `dt` and execution context.** `dt=0.01` is hardcoded in `ekf_sfun.m:58`
   but the block declares `SampleTimes=[0 0]` (continuous, `ekf_sfun.m:17`). A
   **discrete recursive filter must run on a fixed discrete sample time**; under a
   variable-step `ode45` it will fire at solver-chosen instants (and possibly
   multiple minor steps), all while assuming `dt=0.01`. This corrupts both
   prediction (`F`, `Q` scaled by the wrong `dt`) and the very meaning of the
   recursion. **This is the most serious EKF integration defect** (shared root
   cause with the IMU `randn` problem, Section 8).

### Verdict
The EKF is a *reasonable first attitude filter* and the analytic Jacobian work is
commendable, but it is **not yet a sound navigation component**: bias is
unmodeled, yaw is unobservable, one Jacobian term is wrong, and it is executed in
the wrong (continuous) timing context. None of these are fatal — all are standard
fixes — but they must be addressed before closing the loop on `X_est`.

---

## 8. Simulink Model Review

Models present (binary `.slx` inspected via the unpacked `system_root.xml`):

### 8.1 `openloop_sim.slx`
- **Purpose:** open-loop plant validation.
- **Structure:** `fixedwing_sfun` → `Integrator` → state, with Constants for the
  trim input, Demux, six Scopes (Angles, AngVel, LinVel, Positions, Thrust,
  Deflections), `To Workspace`.
- ✅ Clean and fit for trim checking. **Duplicated** verbatim under
  `openloop_sim/openloop_sim.slx` — remove the copy.

### 8.2 `closed_loop_sim_est.slx`
- **Purpose:** full GNC loop (plant + IMU + EKF + SAS).
- **Blocks:** `fixedwing_sfun`, `imu_sfun`, `ekf_sfun`, `Integrator1`,
  gains `K_lat`/`K_long`/`0.5`/`-0.5` (elevon mix), Mux/Demux/Sum,
  `To Workspace1`.
- **Solver:** `ode45`, variable-step, `MaxStep = 0.01`, `StopTime = 10`
  (`configSet0.xml`).
- **Findings:**
  - ❌ **Solver/sample-time mismatch.** All three S-functions declare
    `SampleTimes=[0 0]` (continuous) yet `imu_sfun`/`ekf_sfun` are inherently
    discrete (hardcoded `dt=0.01`, `randn`, recursive `Dwork` state). Running
    discrete stochastic logic under continuous variable-step `ode45` is
    incorrect: `randn` inside `Outputs` is re-drawn at every solver evaluation,
    producing a non-smooth ODE RHS that either forces the solver to tiny steps or
    yields ill-defined noise statistics. **Fix:** give IMU/EKF a discrete
    `SampleTime=[0.01 0]` (or run them in a discrete subsystem), keep only the
    plant continuous.
  - ❌ **Loop not closed on the estimator.** `K_lat`/`K_long` are driven from the
    integrated true state, while `X_est` feeds only `To Workspace1`
    (`out.x_out2`). The estimator is logged but not in the control path — the
    "EKF closed loop" is currently SAS-on-truth + EKF-on-the-side.
  - ⚠️ **`P` via `evalin('base','P')`** inside `fixedwing_sfun` (`:29`) and
    `alpha0` via `evalin('base','alpha0')` in `ekf_sfun.m:39` create hidden
    base-workspace coupling — fragile, non-reentrant, and breaks for code-gen /
    parallel runs. Use block parameters / a data dictionary.
  - ⚠️ **`closed_loop_sim.slx` (non-EKF) is missing** — only its `.slxc`,
    `.autosave`, `.bak`, and `.original` remain. The plotting code in
    `run_sim.m:206+` references `out.x_out1` (lateral SAS, no EKF) from a model
    that is not in the tree. Reproducibility is broken.

### 8.3 Best-practice / standards assessment
| Practice | Status |
|---|---|
| Fixed-step solver for embedded-bound GNC | ❌ variable-step `ode45` |
| Discrete sensors/filters at explicit rates | ❌ all `[0 0]` |
| No base-workspace `evalin` coupling | ❌ used in 2 S-funcs |
| Units/frames documented on signals | ❌ none |
| Bus objects for the 16-state vector | ❌ raw Mux/Demux |
| Build artifacts excluded from VCS | ❌ `slprj/`, `*.slxc`, `*.asv` committed |
| Single source of truth per model | ❌ duplicated openloop model |

---

## 9. Current Progress Assessment

| Subsystem | Maturity | Reasoning |
|---|---:|---|
| **Architecture** | **45%** | Clear plant/SAS/sensor/EKF separation and a closed-loop model exist, but loop isn't closed on the estimate, no guidance layer, undocumented frame conventions. |
| **Dynamics** | **70%** | Full 6-DOF cruise EOM with Coriolis, Euler+Ixz, actuator states, stall/propwash. Missing hover/transition/thrust-moment — the tailsitter-defining physics. |
| **Sensors (IMU)** | **35%** | Functional gyro+accel with noise+constant bias, but tilt-only accel, no scale/misalignment/drift/quantization, no `dt` scaling. |
| **Navigation/EKF** | **40%** | Working attitude EKF with analytic Jacobians, but no bias state, yaw unobservable, one Jacobian sign error, wrong timing context; no position/velocity filter. |
| **Guidance** | **0–5%** | No outer loop, reference model, trajectory generator, or waypoint logic anywhere in the repo. |
| **Control** | **35%** | Decoupled lateral/longitudinal SAS via pole placement exists and is checked for controllability/stability, but signs determined empirically (`check_modes`-style trial/negation), no tracking/integral action, no saturation-aware design, not validated on estimated state. |
| **Simulation** | **30%** | `ode45` trim sim and standalone EKF test run, but hardcoded Windows paths, solver/sample-time defects, missing model file, committed build artifacts. |

**Overall: ~35% — early prototype.** Solid dynamics core; navigation and control
are partial; guidance is absent; simulation infrastructure is fragile.

---

## 10. Technical Gaps

### High priority (blocking)
| # | Gap | Why | Depends on | Effort |
|---|---|---|---|---|
| H1 | Run IMU & EKF at a **discrete fixed sample time** (and use a fixed-step solver for the loop) | Current continuous `randn`/recursive EKF under `ode45` is mathematically invalid | — | S (hours) |
| H2 | **Close the control loop on `X_est`**, not truth | Otherwise navigation is untested and meaningless | H1 | S |
| H3 | Add **gyro-bias states** to the EKF (≥9-state, ideally error-state/MEKF) | Constant IMU bias otherwise corrupts attitude | H1 | M |
| H4 | Provide a **heading aid** (mag/GPS) or explicitly bound the unobservable yaw | ψ drifts without bound | H3 | M |
| H5 | Fix **`F(3,3)` Jacobian sign** | Linearization correctness/consistency | — | XS |
| H6 | Decide **attitude representation** (quaternion) before any hover/transition work | Euler singularity at the tailsitter's primary operating point | — | M |

### Medium priority
| # | Gap | Why | Depends on | Effort |
|---|---|---|---|---|
| M1 | **Higher-fidelity IMU** (kinematic accel, scale, misalignment, bias RW, quantization) | Realistic nav validation | H1 | M |
| M2 | **Full nav EKF** (position/velocity, GPS/baro fusion) | Only attitude is estimated today | H3,H4 | L |
| M3 | **Guidance layer** (waypoint / path-following / reference generator) | Currently absent | H2 | L |
| M4 | **Joseph-form covariance** + NEES/NIS consistency tooling | Numerical robustness & tuning evidence | H3 | S |
| M5 | Replace `evalin('base',…)` with **block params / data dictionary**; bus-ify the 16-state | Reproducibility, code-gen readiness | — | S |
| M6 | **Portable paths / project setup** (remove `/Personal Projects/...` hardcodes; add a `startup.m`/project file) | Repo doesn't run outside the author's machine | — | XS |

### Low priority (future)
- L1 Tailsitter **hover + transition** aerodynamics and control (eventually the
  headline feature). *(Large; depends on H6, M3.)*
- L2 Monte-Carlo / dispersion harness; SIL/HIL prep.
- L3 Repo hygiene: `.gitignore` for `slprj/ *.slxc *.asv *.autosave *.bak`,
  remove duplicate model, expand README.
- L4 Actuator/effector fault and limit modeling.

---

## 11. Critical Issues (ranked)

1. **Discrete logic in a continuous variable-step solver** (IMU `randn` + recursive
   EKF at `SampleTimes=[0 0]`, `dt` hardcoded). Invalidates every closed-loop EKF
   result produced so far. *(H1)*
2. **Control loop closed on truth, not estimate** — the EKF is decorative in the
   current model. *(H2)*
3. **Estimator cannot observe gyro bias or yaw** → guaranteed steady-state attitude
   error + unbounded heading drift. *(H3, H4)*
4. **"Tailsitter" with no tailsitter physics/modes** and an **Euler
   parameterization that is singular at hover** — a structural mismatch with the
   stated objective. *(H6, L1)*
5. **`F(3,3)` Jacobian sign error.** *(H5)*
6. **Non-reproducible simulation:** hardcoded Windows paths, a referenced model
   file (`closed_loop_sim.slx`) missing from the tree, build artifacts committed.

---

## 12. Recommended Next Steps (immediate, in order)

1. **Stabilize the simulation harness** (H1, M5, M6): fixed-step solver, discrete
   `[0.01 0]` IMU/EKF, kill `evalin`, fix paths, restore/commit the missing model.
2. **Fix the EKF correctness items** (H5 sign, M4 Joseph form) and add a NEES/NIS
   consistency script extending `test_ekf.m` to include yaw and bias.
3. **Augment the EKF with gyro-bias states** (H3) and add a heading measurement or
   document yaw as dead-reckoned (H4).
4. **Close the SAS loop on `X_est`** (H2) and quantify the truth-vs-estimate
   control performance gap.
5. **Decide on quaternion attitude** (H6) before investing in hover/transition.

---

## 13. Development Roadmap

| Stage | Objectives | Deliverables | Validation criteria | Risks |
|---|---|---|---|---|
| **0. Harness fix** | Correct timing/solver, portability | Fixed-step model, discrete sensors, `startup.m`, `.gitignore` | Deterministic re-run; no `evalin`; CI runs `run_sim`/`test_ekf` | Hidden model-state bugs surfaced by fixed step |
| **1. Validate 6-DOF dynamics** | Confirm cruise trim & modes | Trim residual ≈0, eigen/mode table vs hand calc | `||dX||≈0` at trim; phugoid/SP/dutch-roll frequencies physical | Aero-derivative provenance unknown |
| **2. Validate IMU** | Realistic, sample-correct sensor | Upgraded `imu_simulate` + spec sheet (ARW/VRW, bias) | Allan-variance reproduces specified stability; noise scales with rate | Over/under-modeling |
| **3. Complete EKF** | Bias-aware, observable, consistent | 9-state (or MEKF) filter, Joseph form, NEES/NIS | NEES within χ² bounds; bias converges; yaw bounded w/ aid | Tuning effort; observability of bias requires motion |
| **4. Sensor-fusion validation** | Add GPS/baro/mag → full nav | 15/16-state nav EKF | Position/velocity error within sensor bounds in MC | Frame/timing alignment bugs |
| **5. Guidance** | Reference/trajectory generation | Waypoint + path-follow (e.g., L1/carrot) module | Track error < tolerance on canonical paths | Coupling with SAS bandwidth |
| **6. Control law** | Tracking + integral, sat-aware | Outer loops on top of SAS, anti-windup | Step/track response meets spec; robust to ±X% derivative error | Sign/scaling errors (already seen in SAS) |
| **7. Closed-loop on estimate** | Full GNC on `X_est` | Integrated `closed_loop_sim_est` | Truth-vs-est performance gap quantified & acceptable | Estimator-in-the-loop instability |
| **8. Tailsitter hover/transition** | VTOL physics + mode logic | Quaternion model, hover aero, transition scheduler | Stable hover, successful transition envelope | Largest technical risk; needs H6 |
| **9. Monte-Carlo** | Statistical robustness | Dispersion harness, pass/fail metrics | ≥N runs within spec; no divergence | Compute; metric definition |
| **10. HIL prep** | Real-time/embedded readiness | Fixed-step code-gen-ready models, I/O bus | Runs at rate; no `evalin`/continuous-noise | Code-gen incompatibilities |

---

## 14. Final Engineering Assessment

### Strengths
- **Sound cruise dynamics core:** full 6-DOF body-frame EOM with Coriolis, Euler
  equations including `Ixz`, actuator states with rate limits, and a thoughtful
  stall/propwash aero blend.
- **Real GNC literacy on display:** numerical-Jacobian linearization,
  controllability checks, and pole-placement SAS for decoupled axes; an EKF with
  hand-derived analytic Jacobians and basic covariance conditioning.
- **Internal consistency** of the unusual ZXY/`[ψ,φ,θ]` convention across plant,
  IMU, and EKF.

### Weaknesses
- Guidance entirely absent; control is SAS-only with empirically-signed gains and
  no tracking/integral/anti-windup.
- IMU and EKF are low-fidelity and run in the wrong timing context; bias and yaw
  are unhandled.
- Simulation is non-portable and partially non-reproducible (missing model,
  hardcoded paths, committed build artifacts, base-workspace coupling).

### Critical risks
- **Timing/solver defect** silently invalidates closed-loop EKF results.
- **Estimator not in the loop** → unverified navigation.
- **Euler-at-hover** singularity + **missing tailsitter physics** mean the current
  model cannot reach the project's namesake mission without an attitude
  re-architecture.

### Recommended refactors
- **Code:** parameterize all magic numbers into `P`; return a struct from
  `params.m` instead of base-workspace scatter; eliminate `evalin`; add a project
  `startup.m` and relative paths; split `dynamics/` (plant) from `nav/` and
  `control/`.
- **Model:** fixed-step solver; discrete sensor/filter sample times; bus objects
  for the 16-state; a data dictionary; one canonical model (drop the duplicate).
- **Estimation:** migrate to a **quaternion error-state (MEKF)** with gyro/accel
  bias states and pluggable aiding (GPS/baro/mag); adopt Joseph-form covariance
  and standing NEES/NIS consistency tests.
- **Simulation:** add a Monte-Carlo dispersion harness and truth-vs-estimate
  scoring before any HIL work.

**Bottom line:** a promising, technically literate **fixed-wing cruise GNC
prototype (~35% complete)** mislabeled as a tailsitter. Prioritize fixing the
simulation timing, closing the loop on the estimate, and hardening the EKF
(bias + yaw) before adding guidance — and treat the tailsitter hover/transition
capability (with a quaternion re-architecture) as the major program milestone it
actually is.
