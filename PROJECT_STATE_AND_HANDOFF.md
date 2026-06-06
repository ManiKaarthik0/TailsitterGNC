# Tailsitter / Fixed-Wing GNC — Project State & Handoff

> **Purpose of this file.** A complete, self-contained snapshot of the project so work can be **continued in a fresh chat or by another engineer** without losing context. It records the system, what is **done**, what is **in progress**, the exact technical state of the estimator work, the design decisions and *why*, the roadmap, reusable code, and an honest status for CV / interview prep.
>
> **Branch:** `claude/tender-goodall-3WCZ7`  ·  **Latest commit at time of writing:** `e4c44dd`
> **Base:** `main` (merge-base `b0212bf`).

---

## 0. One-paragraph status

A **GNC stack for a small (2.5 kg) fixed-wing aircraft** cruising at 20 m/s, modelled in MATLAB/Simulink with full **16-state, 6-DOF** nonlinear dynamics. The **inner-loop controller (Stability Augmentation System, SAS) is COMPLETE and verified** — it stabilises the open-loop-unstable aircraft and holds trim. The **attitude estimator (IMU model + Extended Kalman Filter) is IN PROGRESS**: the test/validation harness is built, the EKF process Jacobian has been numerically verified and a two-part bug fixed, and the next step (gyro-bias estimation) is fully specified. Guidance and the full navigation filter are future work.

---

## 1. System overview

| Item | Value | Source |
|---|---|---|
| Vehicle | Fixed-wing aircraft (named "tailsitter"; only cruise modelled) | `dynamics/`, `params/params.m` |
| Mass / span / wing area | 2.5 kg / 1.0 m / 0.26 m² | `params/params.m` |
| Trim condition | V₀ = 20 m/s, α₀ ≈ 6.55° (0.114 rad) | `sim/linearize_trim.m` |
| Attitude convention | Euler **ZXY**, `R = Rz(ψ)·Rx(φ)·Ry(θ)` | `dynamics/rot_matrix.m` |
| Frames | Body (x-fwd, y-right, z-down); world NED | `dynamics/rigid_body.m` |
| Solver (sim) | `ode45`, MaxStep 0.01 | `.slx` config |

**State vector (16):**
```
X = [ x y z | ψ φ θ | vx vy vz | wx wy wz | T1 T2 | d1 d2 ]
     pos(NED)  Euler    body vel   body rate  thrust  elevons
```
**Input (4) — actuator RATES:** `U = [T1dot, T2dot, d1dot, d2dot]`.

### File map
| File | Role | Status vs `main` |
|---|---|---|
| `params/params.m` | mass, inertia, aero coefficients, actuator limits | unchanged |
| `dynamics/rigid_body.m` | 6-DOF EOM; **actuator rate limits** (elevon 1 rad/s, thrust 10 N/s) | unchanged |
| `dynamics/aero_forces.m` | forces/moments, stall blend, propwash; source of `Cl0` | unchanged |
| `dynamics/aero_angles.m`, `rot_matrix.m`, `elevon_mix.m` | α/β, DCM, elevon mixing | unchanged |
| `dynamics/imu_simulate.m` | IMU model (gyro+accel, constant bias, tilt-only accel) | unchanged (EKF phase) |
| `dynamics/ekf_attitude.m` | 6-state attitude EKF | **edited locally — F(3,3) fix; confirm committed** |
| `dynamics/fixedwing_sfun.m`, `imu_sfun.m`, `ekf_sfun.m` | Simulink wrappers | unchanged |
| `sim/linearize_trim.m` | **trim + linearise + LQR SAS design + ground-truth sim** | **rewritten (SAS work)** |
| `sim/run_sim.m`, `test_ekf.m`, `check_modes.m` | open-loop sim, EKF unit test, modal analysis | mostly unchanged |
| `closed_loop_sim.slx` | plant + SAS ("SAS working") | committed |
| `closed_loop_sim_est.slx` | plant + IMU + EKF + SAS | committed (EKF raw) |
| Docs | `GNC_TECHNICAL_REVIEW.md`, `SYSTEM_DEEP_DIVE_*.md`, `The_Theory_Behind.md` | added |

---

## 2. PHASE A — Stability Augmentation System (✅ COMPLETE)

### What it does
The open-loop aircraft is **unstable**: a fast growing **dutch-roll** (`+0.265 ± 6.82j`) and a slow **spiral** (`+0.132`). The SAS is full-state feedback `u = −K·(x − x_trim)` that drags every closed-loop pole into the left half-plane.

### Design pipeline (in `sim/linearize_trim.m`)
1. **Trim** at V₀=20: solve longitudinal (lift/drag/pitch) **and** lateral (cancel `Cl0` roll moment).
2. **Assert the trim is an equilibrium**: `norm(rigid_body(X_trim,0)(4:16)) < 0.5`.
3. **Linearise** by finite differences → `A` (16×16), `B` (16×4).
4. **Augment** each channel with the actuator as a state (because the real input is a *rate*).
5. **LQR** gains (not pole placement — see lesson below).
6. **Sign self-tests** (assert restoring response).
7. **Ground-truth `ode45` closed-loop sim** (`sas_cmd`) to validate on the nonlinear plant.

### Key numbers
- Lateral design model: 6 states `[vy, p, r, φ, δa, δT]`; `Q_lat = diag([1 1 1 5 0.1 0.1])`, `R_lat = 10·I` → `K_lat` (2×6).
- Longitudinal design model: 5 states `[vx, vz, θ, q, δe]`; `Q_long = diag([1 1 5 1 0.1])`, `R_long = 10` → `K_long` (1×5).
- Closed-loop poles: long `{−19.6, −7.7±5.6j, −0.7±0.9j}`; lat `{−2.3, −9.3, −10.0±11.1j, −6.2±10.3j}` — all stable.
- Lateral trim aileron `δa_trim = −Cl0/Cl_delta ≈ −0.0048 rad`.
- Performance: from a 5° disturbance, settles in ~2 s, **peak elevon rate ≈ 0.07 rad/s** (limit 1.0).

### The four bugs found & fixed (the SAS debugging story)
| # | Bug | Symptom | Fix |
|---|---|---|---|
| 1 | Gains designed for actuator **position** but applied as **rate** (an ignored integrator) | "stable" design tumbled | augment actuator as a design **state**; design poles now = implemented poles |
| 2 | Trim not an equilibrium — uncancelled `Cl0` → **+7.14 rad/s²** roll | rolled off at t=0 | `da_trim = −Cl0/Cl_delta`, asymmetric elevon; add equilibrium assert |
| 3 | No lateral trim feed-forward → controller drove `δa→0`, re-creating the moment | stable ~5 s then diverged | subtract `X_lat_trim = [0;0;0;0;da_trim;0]` |
| 4 | `place()` non-unique (MIMO) + demanded ~20 rad/s vs 1 rad/s limit → **saturation** | diverging oscillation; MATLAB/Octave disagreed | switch to **LQR** with effort penalty `R=10` |

---

## 3. PHASE B — IMU + EKF Attitude Estimator (🟡 IN PROGRESS)

### Current estimator architecture (as in `ekf_attitude.m` today)
- **6-state, total-state form:** `x = [ψ, φ, θ, wx, wy, wz]`.
- **Predict:** attitude via ZXY kinematics; rates modelled as constant (random walk).
- **Update 1 (gyro):** treats gyro as a *measurement* of rates, `H_g = [0₃, I₃]`.
- **Update 2 (accel):** nonlinear gravity-tilt model `[g·sinθ; −g·sinφ·cosθ; −g·cosφ·cosθ]`.
- **Covariance:** `(I−KH)P` with `pinv(S)`, symmetrise, `+1e-9·I` floor.

### IMU model (`imu_simulate.m`)
- gyro = true rate + **constant bias `[0.003, −0.002, 0.001]`** + white noise (σ=0.005).
- accel = **gravity tilt only** (ignores kinematic acceleration) + white noise (σ=0.02).

### What has been DONE in this phase (this session)
1. **Validation harness built** (the right way):
   - Truth generated by **integrating the real closed-loop flight** (`ode45` of `rigid_body` + `sas_cmd`) on a **fixed 100 Hz grid** (`tgrid = 0:dt:T`) → attitude & rates are automatically consistent.
   - Truth → `imu_simulate` → `ekf_attitude`, recursive (state fed back each step).
   - Three diagnostics: **estimate-vs-truth**, **error-vs-±3σ** (from `sqrt(diag(P))`), and **NEES** (`eᵀP⁻¹e`).
2. **Numerical Jacobian verification tool** (`numjac`) — central differences, the same trick used to build `A`. Compares analytic `F` to a finite-difference `F` at a *rich* test point (nonzero rates AND angles).
3. **Bug found & fixed in the EKF process Jacobian `F(3,3)`** — *two* errors in one line:
   - missing **identity term** (`F(3,3)=dt*(...)` overwrote the `eye(6)` diagonal `1`), and
   - **wrong sign** on the `wz` term.
   - **Corrected:** `F(3,3) = 1 + dt*(wx*cth*tph + wz*sth*tph);` — verified to ~`1e-9`.
4. **Consistency analysis:** full-run NEES `7.18 → 6.91`; **steady-state (t>1) NEES = 4.96**, decoded as *the observable states being consistent* while the **unobservable yaw inflates its own variance and drags the aggregate below 6**.

### Diagnosed remaining issues (evidence from the plots)
- **Gyro bias leaks into rates:** `wx` error sits ≈ −0.003, `wy` ≈ +0.002 — exactly `−[bias]`. The filter has no bias state.
- **Yaw unobservable:** `ψ` estimate drifts (0.054 vs truth 0.044); its ±3σ bounds **widen without bound** (vs roll/pitch bounds which *plateau*). Accelerometer can't see heading.
- **Init transient:** NEES spikes ~318 at t=0 (filter starts at zero attitude while truth is tilted ~12°) — initial overconfidence.
- **Latent (catalogued, not yet fixed):** accelerometer is tilt-only (ignores kinematic accel); `(I−KH)P` not Joseph form; IMU/EKF Simulink blocks declare continuous `SampleTimes=[0 0]` with `randn` (must be discrete `[0.01 0]`).

---

## 4. NEXT STEPS (the remaining roadmap)

### ▶ Step 2 — Gyro-bias estimation (the immediate next task, fully specified)
**Restructure to the standard "gyro-as-input" attitude EKF** (stays 6 states by swapping rate-states for bias-states):
- **State:** `x = [ψ, φ, θ, b_x, b_y, b_z]`.
- **Predict:** `omega_hat = z_gyro − b`; propagate attitude with `omega_hat`; bias is a **random walk** (`b_next = b`, growth via `Q`).
- **Jacobian `F` (block form):**
  ```
  F = [ A_att   −B_att ;      A_att  = I + dt·(∂kin/∂att)   (your existing, verified 3×3 att block)
        0         I    ]      B_att  = dt·(∂kin/∂omega)     (your existing F(1:3,4:6) rate block)
  ```
  Key insight: because `omega_hat = z_gyro − b`, **`∂att/∂bias = −B_att`** — reuse the rate block, negated. **Verify with `numjac`.**
- **Measurement:** accel only, `H = [H_att | 0₃]` (gyro update removed).
- **`Q`:** attitude process noise ≈ gyro noise; small bias random-walk noise (tuning knob).
- **Init:** `b_hat = 0`, moderate `P_bias`.
- **Controller** still gets rates as `omega_hat = z_gyro − b_hat` (bias-corrected).
- **Observability caveat:** `b_x, b_y` strongly observable (→ converge to truth); `b_z` weakly observable (yaw still needs a magnetometer); observability improves with maneuvering.
- **Watch:** `b_hat` converges to `[0.003,−0.002,0.001]`; `wx/wy` error offsets vanish; yaw drift shrinks; NEES → clean ~6 on observable subspace.

### ▶ Step 3 — Yaw aiding (magnetometer / heading)
Add a heading pseudo-measurement `z_ψ = ψ + noise`, `H = [1 0 …]`. Makes yaw (and `b_z`) observable → its ±3σ bounds **plateau** instead of growing.

### ▶ Step 4 — Numerical robustness & tuning
- **Joseph-form covariance** `P = (I−KH)P(I−KH)ᵀ + KRKᵀ` (drop the `pinv`/`1e-9` patches).
- Tune `Q`/`R` by **NEES/NIS** consistency, not by eye.
- Optionally raise IMU fidelity (kinematic accel term, bias drift, scale/misalignment).

### ▶ Step 5 — Simulink integration
- Give `imu_sfun`/`ekf_sfun` a **discrete sample time `[0.01 0]`** (currently continuous `[0 0]` with `randn` — invalid under variable-step `ode45`).
- **Close the SAS loop on `X_est`** (currently flies on the true state) and quantify the truth-vs-estimate performance gap.

---

## 5. Key engineering decisions & rationale (the "why")

| Decision | Why |
|---|---|
| Augment actuator as a design state | the real input is a *rate*; otherwise design poles ≠ implemented poles |
| LQR over `place()` | MIMO `place` is non-unique and ignores effort; LQR is unique, reproducible, effort-penalised → respects the 1 rad/s actuator limit |
| Trim the lateral axis too | `Cl0 ≠ 0` gives a constant roll moment; linearisation is only valid at a true equilibrium |
| Verify Jacobians numerically | human eyes catch sign errors but miss *missing terms* (the identity bug); a `numjac` check catches both |
| Gyro-as-input (Step 2) | standard attitude-EKF form; makes gyro bias estimable and gives bias-corrected rates |
| Truth-by-integration in the harness | attitude & rates stay physically consistent (the old `test_ekf.m` prescribed them independently) |

---

## 6. Lessons learned (CV / interview talking points — all genuinely demonstrated)

- **A linearly-stable design can diverge in reality** due to actuator rate saturation; always check commanded effort against physical limits. *(Diagnosed via simulation; fixed with LQR effort weighting.)*
- **MIMO pole placement is non-unique** — different tools gave different gains and different stability; LQR removes the ambiguity.
- **Linearise only about a true equilibrium** — found a 7.14 rad/s² uncommanded roll from an untrimmed `Cl0`; added an automatic equilibrium assertion.
- **Estimator consistency ≠ tracking accuracy** — used **NEES** and **error-vs-±3σ** to expose overconfidence and to recognise that an **unobservable state (yaw) contaminates aggregate NEES**.
- **Numerical Jacobian verification** caught a two-part bug (missing identity + sign) that eyeballing missed; the *magnitude* of the mismatch diagnosed the *type* of error.
- **Observability**: accelerometer+gyro can't see yaw; gyro bias becomes estimable only through attitude coupling and is excited by maneuvering.

> **Honest status for CV:** the **SAS is complete and validated**; the **EKF/IMU is in active development** (harness + Jacobian verification done; gyro-bias and magnetometer-aiding next). Describe it as "designed and validated an LQR stability-augmentation system; **building** an EKF attitude estimator with IMU simulation, including Jacobian verification and NEES-based consistency analysis." That phrasing is accurate, specific, and interview-defensible.

---

## 7. Reusable code (carry into the next chat)

**Numerical Jacobian check:**
```matlab
function J = numjac(fun, x, h)
    n = numel(x); J = zeros(n);
    for j = 1:n
        xp=x; xp(j)=xp(j)+h;  xm=x; xm(j)=xm(j)-h;
        J(:,j) = (fun(xp) - fun(xm)) / (2*h);
    end
end
```

**SAS control law (ground-truth sim):**
```matlab
function U = sas_cmd(X, K_long, K_lat, X_long_trim, X_lat_trim)
    de=(X(15)+X(16))/2; da=(X(15)-X(16))/2; dT=(X(13)-X(14))/2;
    de_rate = -K_long * ([X(7);X(9);X(6);X(11);de] - X_long_trim);
    ul      = -K_lat  * ([X(8);X(10);X(12);X(5);da;dT] - X_lat_trim);
    U = [0.5*ul(2); -0.5*ul(2); de_rate+ul(1); de_rate-ul(1)];
end
```

**EKF harness skeleton:**
```matlab
dt=0.01; T=15; tgrid=0:dt:T; N=numel(tgrid);
X0p=X_trim; X0p(5)+=0.15; X0p(6)+=0.10;                 % perturb so there's motion
[~,Xtrue]=ode45(@(t,X) rigid_body(t,X, sas_cmd(X,K_long,K_lat,X_long_trim,X_lat_trim),P), tgrid, X0p);
x_est=[0;0;X_trim(6);0;0;0];  P=diag([0.1 0.1 0.1 0.01 0.01 0.01]);
Q=diag([1e-4 1e-4 1e-4 1e-5 1e-5 1e-5]); R_gyro=(0.005^2)*eye(3); R_accel=(0.02^2)*eye(3);
for k=1:N
    Xk=Xtrue(k,:).';
    [zg,za]=imu_simulate(Xk,dt);
    [x_est,P]=ekf_attitude(x_est,P,zg,za,dt,Q,R_gyro,R_accel);   % FEED BACK
    truth(:,k)=Xk([4 5 6 10 11 12]); est(:,k)=x_est; sig(:,k)=sqrt(diag(P));
    e=truth(:,k)-x_est; nees(k)=e.'*(P\e);
end
% steady-state consistency: mean(nees(tgrid>1)) should approach the observable-state count
```

---

## 8. Open issues / known bugs still present

- [ ] `ekf_attitude.m` `F(3,3)` fix — applied locally this session; **confirm it is committed**.
- [ ] No gyro-bias state (Step 2).
- [ ] Yaw unobservable — no magnetometer (Step 3).
- [ ] `(I−KH)P` not Joseph form (Step 4).
- [ ] Accelerometer tilt-only (ignores kinematic acceleration).
- [ ] IMU/EKF Simulink blocks at continuous `[0 0]` sample time with `randn` (Step 5).
- [ ] SAS loop closed on **true** state, not `X_est` (Step 5).
- [ ] Hardcoded absolute `addpath('/FixedwingGNC/...')` in `linearize_trim.m` — non-portable.
- [ ] Build artifacts (`slprj/`, `*.slxc`, `*.autosave`) committed — should be `.gitignore`d.

---

## 9. How to resume in a new chat

Paste this file as context and say: *"Continue the GNC project from this state. Next task: Step 2 — implement gyro-bias estimation in the EKF using the gyro-as-input restructure described in §4."* Everything needed (architecture, the `F` block structure, the reusable harness, the diagnostics, and the watch-list) is above.

---

*Snapshot generated for chat handoff. SAS: complete & validated. EKF/IMU: harness + Jacobian verification done; gyro-bias estimation is the next step.*
