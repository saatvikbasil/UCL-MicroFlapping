% FWMAV_Setup  Load all parameters for the direct-drive FWMAV simulation.
% Defines motor, gearhead, wing, spring, drive, and aerodynamic parameters
% in the base workspace so that the Simulink model can read them.

clear; clc;

%% Motor (NEW: small-DC test motor)
ratedVoltage    = 3;        % V
noLoadCurrent   = 0.070;      % A at rated voltage
Ra              = 9;        % Ohm, terminal resistance
La              = 29e-3;     % H, armature inductance
Jrotor          = 1.5e-7;      % kg*m^2, rotor inertia
motorMass       = 1.2e-3;    % kg, motor + gearhead
shaftDiameter   = 1.5e-3;       % m
noLoadRpmRaw    = 2000;        % motor-shaft no-load speed
stallCurrentDs  = 0.330;      % A, datasheet momentary stall


omegaNoLoadRaw  = noLoadRpmRaw * 2*pi/60;
KbRaw           = 0.010;
KtRaw           = KbRaw;
bmRaw           = 3.6e-6;

%% Gearhead (built-in planetary)
gearRatio       = 1;
gearEfficiency  = 1;       % torque-conversion efficiency

%% Reflected motor + gearhead constants (output-shaft frame)
Kb              = KbRaw * gearRatio;
Kt              = KtRaw * gearRatio * gearEfficiency;
Jm              = Jrotor * gearRatio^2;
bm              = bmRaw  * gearRatio^2;

%% Coulomb friction (motor + gearhead, output shaft)
tauCoulomb      = 0.00;      % Nm  (tune from experiment)
omegaSmooth     = 1.0;        % rad/s, smoothing scale for tanh sign approximation

%% Wing geometry
wingLength      = 0.06049;            % Le, span of one wing [m]
aspectRatio     = 3;
rootChord       = wingLength / aspectRatio;   % Lc [m]
hubLength       = 0.01;             % Lb, hub/root width [m]
frameArea       = 1e-6;               % m^2, CFRP frame cross-section
membraneThk     = 25e-6;              % m
rhoCfrp         = 1900;               % kg/m^3
rhoMembrane     = 1100;               % kg/m^3 
beamWidth       = 0.005;              % wb [m]
beamHeight      = 0.001;              % hb [m]

%% Wing inertia (about flap axis)
nElements       = 50;
rGrid           = linspace(hubLength/2, wingLength + hubLength/2, nElements);
dr              = rGrid(2) - rGrid(1);
chordGrid       = rootChord * sqrt(max(0, 1 - ((rGrid - hubLength/2) / wingLength).^2));

rEffective      = sum(rGrid .* (rGrid.^2 .* chordGrid) * dr) / ...
                  sum(rGrid.^2 .* chordGrid * dr);

beamArea        = beamHeight * beamWidth;
Jbeam           = (1/3) * rhoCfrp * beamArea * hubLength^3;
JleadingEdge    = (1/3) * rhoCfrp * frameArea * wingLength * (wingLength + hubLength/2)^2;
JrootChord      = (1/3) * rhoCfrp * frameArea * rootChord * (hubLength/2)^2;
Jmembrane       = rhoMembrane * membraneThk * (pi/4 * rootChord * wingLength) * rEffective^2;
Jw              = Jbeam + JleadingEdge + JrootChord + Jmembrane;

%% Added-mass virtual inertia (Sane-Dickinson 2002)
jAddedMass      = (pi/4) * 1.225 * sum(chordGrid.^2 .* rGrid.^2) * dr;

Jtotal          = Jm + Jw + jAddedMass;

%% Wing mass budget
mBeam       = rhoCfrp * beamArea * hubLength;
mLeadEdge   = rhoCfrp * frameArea * wingLength;
mRootChord  = rhoCfrp * frameArea * rootChord;
mMembrane   = rhoMembrane * membraneThk * (pi/4 * rootChord * wingLength);
mWingOne    = mBeam + mLeadEdge + mRootChord + mMembrane;
mWingTotal  = mWingOne;          
mTotal      = motorMass + mWingTotal;
weightTotal = mTotal * 9.81;         % N

%% Aerodynamics
airDensity      = 1.225;      % kg/m^3
alphaCoeff      = 5e-3;       % Cα, passive wing rotation coefficient

%% Drive and spring
kSpring         = 0.01580;           % Nm/rad
driveFreq       = sqrt(kSpring / Jtotal) / (2*pi);            % Hz
driveOmega      = 2*pi*driveFreq;
driveVoltage    = ratedVoltage;                % V, sine amplitude

%% Solver
numCycles       = 40;
tStop           = numCycles / driveFreq;
solverStep      = 1e-7;

%% Quick natural-frequency report
naturalFreq = sqrt(kSpring / Jtotal) / (2*pi);
fprintf('Parameters loaded.\n');
fprintf('  Natural frequency: %.2f Hz\n', naturalFreq);
fprintf('  Drive frequency:   %.2f Hz\n', driveFreq);
fprintf('  Q (mech only):     %.2f\n', sqrt(Jtotal*kSpring)/bm);

fprintf('  Wing mass (both):  %.3f g\n', mWingTotal*1e3);
fprintf('  Total mass:        %.3f g\n', mTotal*1e3);
fprintf('  Weight to beat:    %.4f N  (%.2f mN)\n', weightTotal, weightTotal*1e3);