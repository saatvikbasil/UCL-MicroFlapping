# Matlab Simulink

Electromechanical simulation, design-space exploration, and wing optimisation for the direct-drive FWMAV. Implements the coupled quasi-steady aerodynamic and motor dynamics model of Park & Abolfathi (2024), extended with Coulomb friction, resonant spring tuning, and a multi-start `fmincon` optimiser.

---

## Files

| File | Description |
|---|---|
| `FWMAV_Setup.m` | Load all motor, wing, spring, and solver parameters |
| `FWMAV_Run.m` | One-line pipeline: setup → simulate → analyse |
| `FWMAV_Analyze.m` | Post-process a Simulink run; prints metrics and plots six time-series panels |
| `FWMAV_SimulateInner.m` | Fast RK4 integrator used by the optimiser (no Simulink required) |
| `FWMAV_Optimize.m` | Multi-start `fmincon` optimisation over wing length and drive frequency |
| `FWMAV_DesignSpace.m` | Parallel sweep over (wing length, frequency) grid; produces heatmaps |
| `FWMAV_FrequencySweep.m` | Frequency sweep with experimental stroke overlay |
| `ForceSensor_Testing.m` | Process raw LabVIEW CSV from the PCB 208C force transducer |
| `FWMAV_DirectDrive.slx` | Simulink block diagram (reference model) |

---

## Quick Start

```matlab
% Single simulation
FWMAV_Run

% Optimise wing length and frequency for maximum lift
FWMAV_Optimize

% Map the design space
FWMAV_DesignSpace
```

---

## Model Summary

The simulation integrates two coupled ODEs:

**Mechanical**
```
J_total · β̈ = Kt·I − bm·β̇ − Cw·β̇|β̇| − KDD·β − τc·tanh(β̇/ωs)
```

**Electrical** (quasi-static inner loop)
```
I = (V(t) − Kb·β̇) / Ra
```

Aerodynamic lift and drag torque are computed by blade-element integration along the span using the Sane–Dickinson empirical coefficients. The spring stiffness `KDD` is always set to satisfy the resonance condition `KDD = J_total · (2πf)²`.

---

## Optimisation

`FWMAV_Optimize` runs 20 multi-start `fmincon` (interior-point) calls over the design vector `[Le, f]`. Constraints cap peak current at 1.5 A, mean power at 2 W, and peak stroke at 120°. The best feasible result is reported with a full performance breakdown.

---

## Dependencies

- MATLAB R2020a or later
- Simulink (for `FWMAV_Run` and `FWMAV_FrequencySweep`)
- Optimization Toolbox (for `FWMAV_Optimize` and `FWMAV_DesignSpace`)
- Parallel Computing Toolbox (optional; accelerates `FWMAV_DesignSpace`)

---

## Key Parameters (`FWMAV_Setup.m`)

| Parameter | Symbol | Value |
|---|---|---|
| Terminal resistance | Ra | 7.8 Ω |
| Gear ratio | N | 26.5 |
| Wing length | Le | 70.6 mm |
| Aspect ratio | AR | 2.059 |
| Drive voltage | V₀ | 4.5 V |
| Spring stiffness | KDD | 1.409 mNm/rad (default) |

---

## Reference

Park, H. & Abolfathi, A. (2024). *Direct Drive Design and Operational Performance Analysis of an Insect-Sized Flapping-Wing Micro Air Vehicle.*
