# FWMAV Optimization Code Documentation

Complete guide to the Flapping Wing Micro Aerial Vehicle (FWMAV) direct-drive motor selection and optimization codebase.

---

## 📋 Overview

This repository contains MATLAB code for designing and optimizing direct-drive FWMAVs based on the Park & Abolfathi (2024) model. The code performs motor selection, wing parameter optimization, dynamic simulation, and performance analysis.

**Key Features:**
- Motor parameter validation and optimization
- Wing geometry optimization (length, frequency)
- Spring stiffness calculation for resonant operation
- Full aerodynamic modeling with blade element theory
- Simulink-based dynamic simulation
- Performance metrics (lift, power, efficiency)

---

## 📋 Software Requrements
1. MATLAB (R2020a or later recommended)
2. Simulink
3. Optimization Toolbox
4. Global Optimization Toolbox (for Motor_Selection_fmincon.m only)

## 🚀 Quick Start

### Basic Pipeline (Recommended for Beginners)
```matlab
% 1. Set up parameters
FWMAV_Parameters

% 2. Run simulation
out = sim('FWMAV_DirectDrive.slx');

% 3. Analyze results
Analyze_Results
```

Or use the automated pipeline:
```matlab
pipeline
```

### Motor Selection & Optimization
```matlab
% For comprehensive motor selection with your motor specs:
Optimize_DirectDrive

% For advanced optimization with MultiStart:
Motor_Selection_fmincon
```

---

## 📁 File Descriptions

### 1. **pipeline.m**
**Purpose:** Automated workflow for quick parameter → simulation → analysis

**What it does:**
- Loads FWMAV parameters
- Runs Simulink simulation
- Analyzes results and generates plots

**Usage:**
```matlab
pipeline
```

**Best for:** Quick iterations after modifying parameters

---

### 2. **FWMAV_Parameters.m**
**Purpose:** Initialize all motor, wing, and simulation parameters

**Parameters defined:**
- **Motor specs:** Ra, La, Kt, Kb, Jm, bm
- **Wing geometry:** Le, Lc, Lb, aspect ratio
- **Material properties:** CFRP density, membrane thickness
- **Operating conditions:** frequency, voltage amplitude
- **Spring stiffness:** KDD (calculated for resonance)

**Key calculations:**
- Wing moment of inertia (Jw)
- Total system inertia (Jm + Jw)
- Resonant spring stiffness: `KDD = J_total * (2πf)²`

**Usage:**
```matlab
FWMAV_Parameters  % Run before simulation
```

**Outputs to workspace:**
- All parameters needed for Simulink model
- Displays summary of loaded parameters

**Customization:**
Edit the motor parameters section to match your motor datasheet:
```matlab
Ra = 9;                      % Your motor's resistance
Kt = 0.01;                   % Your motor's torque constant
% ... etc
```

---

### 3. **Analyze_Results.m**
**Purpose:** Post-process Simulink simulation results and calculate performance metrics

**What it analyzes:**
1. **Steady-state extraction:** Last complete cycle
2. **Performance metrics:**
   - Average lift (mN)
   - Input electrical power (W)
   - Aerodynamic power (W)
   - System efficiency (%)
   - Peak stroke angle (deg)
   - Peak current (mA)

**Key equations:**
- **Input Power (Eq. 20):** `Pe = mean(|V × I|)`
- **Aerodynamic Power (Eq. 22):** `Pa = mean(|drag_torque × ω|)`
- **Efficiency (Eq. 23):** `η = Pa/Pe × 100%`

**Usage:**
```matlab
% After running Simulink simulation:
Analyze_Results
```

**Requirements:**
- Must have `out` (SimulationOutput) or `ans` in workspace
- Run `FWMAV_Parameters` first to have frequency available

**Outputs:**
- Console display of all metrics
- Comparison table with paper results
- 6-panel figure with time histories and phase portrait

**Critical feature:** Correctly calculates drag **torque** (not just drag force) for accurate aerodynamic power estimation

---

### 4. **Motor_Selection_fmincon.m**
**Purpose:** Advanced optimization using MATLAB's MultiStart for robust global optimization

**Algorithm:** Systematic multi-start fmincon with SQP solver

**Optimization variables:**
- `x(1)`: Wing leading edge length, Le [m]
- `x(2)`: Flapping frequency [Hz]

**Objective:** Maximize average lift

**Constraints:**
- Power < 1.5W (scales with voltage²)
- Efficiency between 12-50%
- Physical bounds on Le and frequency

**Key features:**
1. **Systematic starting points:** Grid of Le × frequency combinations
2. **Voltage-adaptive bounds:** Different ranges for 3V vs 5V operation
3. **All solutions tracked:** Displays top 10 local minima
4. **Fast simulation:** Reduced to 6 cycles for speed
5. **Convergence checking:** Validates steady-state by comparing consecutive cycles

**Usage:**
```matlab
% Edit voltage at line 31:
V_amplitude = 5;  % or 3V

% Run optimization:
Motor_Selection_fmincon
```

**Typical runtime:** 3-5 minutes for ~25 starting points

**Outputs:**
- Optimal design parameters (Le, freq, KDD)
- Performance metrics (lift, power, efficiency)
- All local minima ranked by lift
- 3-panel visualization of design space
- Saved .mat file with results

**Comparison with paper:**
Automatically compares to Park & Abolfathi (2024) results if V=3V or V=5V

---

### 5. **Optimize_DirectDrive.m**
**Purpose:** Complete all-in-one motor selection tool with grid search

**Best for:** 
- Testing new motor specifications
- Understanding design space
- Quick motor selection

**Algorithm:** Two-stage grid search
1. **Coarse search:** 15×15 grid across full range
2. **Refined search:** 20×20 grid near paper's optimal region

**Motor parameter entry:**
Edit lines 15-20 with your motor specs:
```matlab
motor_Ra = 3.035;      % From datasheet: terminal resistance
motor_Kt = 0.00893;    % Calculate: τ_stall / I_stall
motor_Kb = 0.0097;     % Calculate: (V - I_no-load×Ra) / ω_no-load
motor_La = 0.005;      % Estimate ~5mH for small DC motors
motor_Jm = 5e-7;       % Estimate from motor size
motor_bm = 5e-6;       % Estimate ~5e-6 for small motors
```

**Search ranges:**
- Le: 50-80 mm (wing leading edge)
- Frequency: 18-30 Hz (flapping frequency)

**Constraints:**
- Input power < 1.5W
- Efficiency > 15%

**Usage:**
```matlab
Optimize_DirectDrive
```

**Typical runtime:** 5-10 minutes

**Outputs:**
1. **Console:**
   - Optimal design summary
   - Comparison with paper (if 5V)
   - Coarse vs. refined results
   
2. **Visualizations:**
   - 3D surfaces (lift, efficiency, power)
   - Performance trade-offs
   - Frequency sweeps
   - Wing size sweeps

3. **Saved file:** `motor_selection_results.mat`

**Helper functions included:**
- `calculate_wing_inertia()`: Equation 12 from paper
- `run_simulation()`: Full dynamic simulation
- `get_aero()`: Blade element aerodynamics

---

### 6. **old_fmincon.m**
**Purpose:** Earlier version of fmincon optimization (kept for reference)

**Differences from Motor_Selection_fmincon.m:**
- Uses 3 hand-picked initial guesses instead of systematic grid
- Longer simulations (10 cycles vs 6)
- More iterations (300 vs 150)
- Includes verification test at paper's values

**Usage:**
```matlab
old_fmincon
```

**When to use:**
- Reference for algorithm development
- Comparing optimization strategies
- Validation purposes

**Note:** `Motor_Selection_fmincon.m` is generally preferred for production use

---

### 7. **create_fwmav_simulink_model.m**
**Purpose:** Programmatically create Simulink model structure

**What it creates:**
- Input voltage source (sine wave)
- Current dynamics (motor electrical circuit)
- Motor torque generation
- Aerodynamic damping
- Spring torque
- Angular acceleration calculation
- State integrators (current, β_dot, β)
- Scopes for visualization

**Usage:**
```matlab
create_fwmav_simulink_model
```

**Outputs:**
- `FWMAV_DirectDrive.slx` Simulink model
- Opens model in Simulink editor

**Note:** Manual configuration still needed for:
- MATLAB Function block (aerodynamics)
- Proper signal routing
- Parameter connections

**Current status:** Basic structure only - the existing `FWMAV_DirectDrive.slx` is more complete

---

## 🔬 Technical Details

### System Dynamics

**Electrical subsystem:**
```
di/dt = (V(t) - Ra·i - Kb·β_dot) / La
```

**Mechanical subsystem:**
```
J_total·β_ddot = Kt·i - bm·β_dot - Cw·β_dot·|β_dot| - KDD·β
```

**Aerodynamics (blade element theory):**
- Angle of attack: `α = π/2 - Calpha·|β_dot|`
- Lift coefficient: `CL = 0.225 + 1.58·sin(2.13α - 0.1257)`
- Drag coefficient: `CD = 1.92 - 1.55·cos(2.04α - 0.1714)`
- Integration from wing base to tip

### Key Design Parameters

| Parameter | Symbol | Typical Range | Unit |
|-----------|--------|---------------|------|
| Wing length | Le | 50-85 | mm |
| Root chord | Lc | Le/3 | mm |
| Frequency | f | 18-40 | Hz |
| Voltage | V | 3-6 | V |
| Spring stiffness | KDD | 5-20 | mNm/rad |

### Performance Metrics

**Target performance (5V, paper):**
- Lift: 32.35 mN
- Efficiency: 29.87%
- Power: 0.877 W
- Stroke angle: ~30°

---

## 🛠️ Workflow Guide

### For New Motor Selection:
1. **Gather motor specs** from datasheet (Ra, Kt, Kb, etc.)
2. **Edit `Optimize_DirectDrive.m`** with your motor parameters
3. **Run optimization:** `Optimize_DirectDrive`
4. **Note optimal design:** Le, frequency, KDD
5. **Validate in Simulink:**
   - Update `FWMAV_Parameters.m` with optimal values
   - Run `pipeline`
6. **Analyze results:** Check if performance meets requirements

### For Parameter Tuning:
1. **Modify `FWMAV_Parameters.m`**
2. **Run `pipeline`** to see effect
3. **Iterate** until satisfied

### For Detailed Optimization:
1. **Use `Motor_Selection_fmincon`** for rigorous search
2. **Check all local minima** for alternative designs
3. **Validate convergence** in plots

---

## 📊 Validation

All codes validated against:
- **Park & Abolfathi (2024)** - "Direct Drive Design and Operational Performance Analysis of an Insect-Sized Flapping-Wing Micro Air Vehicle"

**Known validation points:**
- 5V: Le=68.46mm, f=23.89Hz, Lift=32.35mN, η=29.87%
- 3V: Le=60.49mm, f=35.67Hz, Lift=12.31mN, η=33.74%

---

## ⚠️ Common Issues & Fixes

### "Run the Simulink simulation first!"
**Fix:** Run `FWMAV_Parameters`, then `sim('FWMAV_DirectDrive')` before `Analyze_Results`

### Unrealistic efficiency (>100% or negative)
**Fix:** Check that:
- Motor parameters are in correct units
- Spring stiffness is reasonable (5-20 mNm/rad)
- Simulation has converged (run more cycles)

### Optimization returns poor results
**Fix:**
- Check motor parameter estimates (especially La, Jm, bm)
- Verify power constraint isn't too restrictive
- Try different voltage levels

### Constraint violations
**Fix:**
- Increase power limit if motor can handle it
- Relax efficiency bounds (12-50% is reasonable)
- Expand search bounds for Le and frequency

---

## 📚 References

Park, H., & Abolfathi, A. (2024). Direct Drive Design and Operational Performance Analysis of an Insect-Sized Flapping-Wing Micro Air Vehicle.

**Key equations implemented:**
- Eq. 8-9: Lift and drag coefficients
- Eq. 10: Angle of attack
- Eq. 12: Wing moment of inertia
- Eq. 20: Input electrical power
- Eq. 22: Aerodynamic power
- Eq. 23: System efficiency

---

## 💡 Tips

1. **Motor parameter estimation:** If datasheet incomplete, estimate:
   - La ≈ 5 mH (typical for small DC motors)
   - Jm ≈ 5×10⁻⁷ kg·m² (scale with motor size)
   - bm ≈ 5×10⁻⁶ Ns/rad (usually small)

2. **Kt and Kb calculation:**
```matlab
   Kt = Torque_stall / Current_stall
   Kb = (V_rated - Current_noload * Ra) / (Speed_noload * 2*pi/60)
```

3. **Optimization speed:** Reduce simulation cycles in optimization codes if needed (line ~200 in helper functions)

4. **Visualization:** All optimization codes generate plots automatically

5. **Batch processing:** Modify `Optimize_DirectDrive.m` to loop over multiple motor specs

---

## 🔄 Code Relationships
```
FWMAV_Parameters.m ──→ FWMAV_DirectDrive.slx ──→ Analyze_Results.m
                              ↑
                              │
                         pipeline.m (combines all)

Motor_Selection_fmincon.m ──→ Optimal parameters
                                    ↓
Optimize_DirectDrive.m ──────→ FWMAV_Parameters.m

create_fwmav_simulink_model.m ──→ FWMAV_DirectDrive.slx (initial)
```

---
