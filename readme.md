# UCL MicroFlapping

**MECH0073 Capstone Group Design Project — UCL Mechanical Engineering, 2025/26**

A direct-drive flapping-wing micro aerial vehicle (FWMAV) designed for complex disaster-relief environments. The system combines bio-inspired resonant wing kinematics with a brushed DC motor to achieve efficient, low-mass flapping flight at the sub-5 g scale.

---

## Repository Structure

```
UCL-MicroFlapping/
├── Arduino/            # Embedded firmware for motor drive and frequency sweep
├── Matlab Simulink/    # Electromechanical model, optimisation, and analysis
└── Python/             # High-speed camera processing and lift-force analysis
```

---

## System Overview

The prototype uses two Precision Microdrives 206-10K brushed DC motors driven by an Adafruit TB6612FNG dual H-bridge, commanded by an Arduino Nano Every. A torsion spring tuned to the resonance condition cancels inertial torque each half-stroke, minimising motor power. Wings are fabricated from 12 µm Mylar membrane with 0.5 mm CFRP veins and a 1 mm CFRP leading-edge rod.

| Parameter | Value |
|---|---|
| Total mass | 4.92 g |
| Wingspan | < 10 cm |
| Motor | PMD 206-10K (1.26 g) |
| Drive voltage | 4.5 V sinusoidal |
| Target frequency | ~8–10 Hz |
| Spring stiffness range tested | 1.41–8.11 Nmm/rad |

---

## Quick Start

**Simulation**
```matlab
cd "Matlab Simulink"
FWMAV_Setup        % load parameters
FWMAV_Run          % simulate and plot
```

**Frequency sweep (hardware)**

Flash `Arduino/Wing_Flappingv3.ino` to the Arduino Nano Every, connect the TB6612FNG driver, and power on. The sweep runs automatically from 2 Hz to 50 Hz.

**Lift-force data processing**
```bash
cd Python
python Frequency-Lift.py
```

---

## Dependencies

| Tool | Version |
|---|---|
| MATLAB | R2020a or later |
| Simulink | included with MATLAB |
| Optimization Toolbox | required for `FWMAV_Optimize.m` |
| Python | 3.9+ |
| Python packages | numpy, pandas, scipy, matplotlib, opencv-python |
| Arduino IDE | 2.x |

---

## Reference

Park, H. & Abolfathi, A. (2024). *Direct Drive Design and Operational Performance Analysis of an Insect-Sized Flapping-Wing Micro Air Vehicle.*

---

## Authors

Saatvik Basil, Ruby Duck, Joshua Evans, George Katochianos, Youssef Metias, Oskar Peterson

Supervised by Dr Ali Abolfathi, UCL Department of Mechanical Engineering.
