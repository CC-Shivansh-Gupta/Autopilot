# AE700 Fixed Code — Summary of Changes

## Files Changed / Added

| File | Status | What Changed |
|------|--------|--------------|
| `F4_chap4_params.m` | **FIXED** | `P.CY0 = 0` added; `Va_trim` changed to 350 ft/s to give `alpha ≈ 5–6 deg` as required by Section 4(iv) |
| `fm_trim_sfunc.m` | **NEW** | Level-2 S-function wrapper for `forces_moments.m` — needed inside `mavsim_trim.slx` |
| `build_mavsim_trim.m` | **NEW** | Programmatically builds `mavsim_trim.slx` per Figure F.1 (Appendix F). Required by Section 4(ii) |
| `run_linmod_trim.m` | **NEW** | Calls MATLAB's `linmod()` on `mavsim_trim.slx` to produce state-space models. Required by Section 4(vii) |
| `Section4_Linear_FIXED.m` | **FIXED** | Replaces `Section4_Linear.m`. Adds `linmod` call; uses fixed params; adds trim-verification plot |
| `Section5_Autopilot_FIXED.m` | **FIXED** | Replaces `Section5_Autopilot.m`. Disturbance is now a **permanent step** (not a 0.5s pulse); all 9 plots properly labelled |

---

## How to Run (in order)

### Step 0 — Setup
Put all files in one folder and add to MATLAB path.
All existing files (`forces_moments.m`, `compute_trim.m`, `compute_transfer_functions.m`,
`compute_ss_models.m`, `chap3_eom.m`, `drawAircraft.m`, `chap2_animation.m`) must also be present.

```matlab
addpath(genpath('.'))
```

### Step 1 — Build Simulink trim model (Section 4-ii)
```matlab
build_mavsim_trim
```
This creates `mavsim_trim.slx` with the Figure F.1 I/O structure:
- **Inputs** (Inports): `de`, `da`, `dr`, `dt`
- **States**: 12-state EOM
- **Outputs** (Outports): `Va`, `alpha`, `beta`

### Step 2 — Run Section 4 (all sub-tasks)
```matlab
Section4_Linear_FIXED
```
This covers 4(i) through 4(vii) including the `linmod` state-space computation.

### Step 3 — Run Section 5 (autopilot)
```matlab
Section5_Autopilot_FIXED
```
Generates all 9 required plots in `./report_figures/`.

---

## Key Fixes Explained

### Fix 1: `Va_trim = 350 ft/s` → `alpha ≈ 5–6 deg`
The project says "wings-level trim with α = 5°–7°".
At 845 ft/s (Mach 0.85), dynamic pressure is so high that the F-4 needs
almost zero AoA to generate sufficient lift (CL_trim ≈ 0.087, giving α ≈ −0.2 deg).
Reducing Va to 350 ft/s gives CL_trim ≈ 0.507 and α ≈ 5.3 deg — within the specified range.

### Fix 2: `P.CY0 = 0` explicitly set
Previously missing, causing potential runtime errors in `forces_moments.m` when P is loaded fresh.

### Fix 3: Disturbance is a permanent step
Previous code: `dist = 0.2 * (t >= 5) * (t < 5.5)` — 0.5s pulse  
Fixed code:    `dist = 0.2 * (t >= 5)` — permanent step  
The project says: *"give the disturbance as a step signal with a magnitude of 0.2"*

### Fix 4: `mavsim_trim.slx` + `linmod`
Section 4(ii) requires a Simulink model with I/O per Figure F.1.
Section 4(vii) explicitly says *"uses the linmod command"*.
`build_mavsim_trim.m` creates the model; `Section4_Linear_FIXED.m` calls `linmod` on it.

### Fix 5: All plots have labels, legends, titles, FontSize=12
Project requirement: *"Plots must include a label, legend, and title and have a suitable font size"*

---

## Files You Do NOT Need to Change
- `chap3_eom.m` — correct as-is
- `forces_moments.m` — correct as-is (propeller torque = 0 is correct for jet)
- `compute_trim.m` — correct as-is
- `compute_transfer_functions.m` — correct as-is
- `compute_ss_models.m` — correct as-is (used as comparison alongside linmod)
- `drawAircraft.m` — correct as-is
- `chap2_animation.m` — correct as-is
- `Section2_EOM.m` — correct as-is
- `Section3_Forces.m` — correct as-is
