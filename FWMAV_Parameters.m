%% FWMAV_Parameters.m
% Run this script before running the Simulink model

clear; clc;

%% Motor Parameters (Table 1 from paper)
Ra = 9;                      % Armature resistance [Ohm]
La = 0.029;                  % Armature inductance [H]
Kt = 0.01;                   % Torque constant [Nm/A]
Kb = 0.01;                   % Back-EMF constant [V/(rad/s)]
Jm = 1.5e-7;                 % Motor inertia [kg·m²]
bm = 3.6e-6;                 % Mechanical damping [Ns/rad]

%% Physical Constants
rho = 1.225;                 % Air density [kg/m³]
Calpha = 5e-3;               % Passive rotation coefficient

%% Wing Material Properties
rho_wf = 1900;               % CFRP density [kg/m³]
rho_mb = 1100;               % Polyimide density [kg/m³]
Awf = 1e-6;                  % Wing frame cross-section [m²]
t_mb = 25e-6;                % Membrane thickness [m]
aspect_ratio = 3;            % Le/Lc
Lb = 0.010;                  % Wing base length [m]
wb = 0.005;                  % Wing base width [m]
hb = 0.001;                  % Wing base height [m]
Awb = hb * wb;               % Wing base cross-section [m²]

%% Optimized Parameters (from Table 3 - 5V case)
Le = 0.06846;                % Leading edge length [m]
Lc = Le / aspect_ratio;      % Root chord [m]
freq = 23.89;                % Frequency [Hz]
KDD = 0.00865;               % Spring stiffness [Nm/rad]


%% Calculate Wing Inertia (Eq. 12)
r_bar = 0.6 * Le;            % Approximate aerodynamic center

Jw = (1/3) * Awb * Lb^3 * rho_wf + ...
     (1/3) * Awf * Le * rho_wf * (Le + Lb/2)^2 + ...
     (1/3) * Awf * Lc * rho_wf * (Lb/2)^2 + ...
     (pi/4) * Lc * Le * t_mb * rho_mb * r_bar^2;

%% Total Inertia
J_total = Jm + Jw;
KDD = J_total * (2*pi*freq)^2;  % Recalculate
fprintf('Corrected KDD: %.2f mNm/rad\n', KDD*1000);

%% Input Signal Parameters
V_amplitude = 5;             % Input voltage amplitude [V]
omega = 2*pi*freq;           % Angular frequency [rad/s]

%% Simulation Parameters
dt = 1e-5;                   % Fixed step size [s]
t_stop = 10/freq;             % Simulate 5 cycles [s]

%% Display Parameters
fprintf('=== FWMAV Parameters Loaded ===\n');
fprintf('Wing Leading Edge: %.2f mm\n', Le*1000);
fprintf('Wing Root Chord: %.2f mm\n', Lc*1000);
fprintf('Frequency: %.2f Hz\n', freq);
fprintf('Wing Inertia: %.2e kg·m²\n', Jw);
fprintf('Total Inertia: %.2e kg·m²\n', J_total);
fprintf('Spring Stiffness: %.2f mNm/rad\n', KDD*1000);
fprintf('==============================\n');