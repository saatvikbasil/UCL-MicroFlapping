% FWMAV_Setup  Load all parameters for the direct-drive FWMAV simulation.
clear; clc;

%% Motor 
ratedVoltage    = 4.5;        % V
noLoadCurrent   = 0.04;      % A at rated voltage
Ra              = 7.8;        % Ohm, terminal resistance
La              = 1.5e-4;     % H, armature inductance
Jrotor          = 5e-10;      % kg*m^2, rotor inertia
motorMass       = 1.26e-3;    % kg, motor + gearhead
shaftDiameter   = 2e-3;       % m
noLoadRpmRaw    = 1820 * 26.5;        % motor-shaft no-load speed
stallCurrentDs  = 0.623;      % A, datasheet momentary stall


omegaNoLoadRaw  = noLoadRpmRaw * 2*pi/60;
KbRaw           = (ratedVoltage - noLoadCurrent * Ra) / omegaNoLoadRaw;
KtRaw           = KbRaw;
bmRaw           = KtRaw * noLoadCurrent / omegaNoLoadRaw;

%% Gearhead (built-in planetary)
gearRatio       = 26.5;
gearEfficiency  = 0.5;       % torque-conversion efficiency

%% Reflected motor + gearhead constants (output-shaft frame)
Kb              = KbRaw * gearRatio;
Kt              = KtRaw * gearRatio * gearEfficiency;
Jm              = Jrotor * gearRatio^2;
bm              = bmRaw  * gearRatio^2;

%% Coulomb friction
tauCoulomb      = 0.002;      % Nm  (tune from experiment)
omegaSmooth     = 1.0;       

%% Wing geometry
wingLength      = 0.07059;            % Le, span of one wing [m]
aspectRatio     = 2.0591;
rootChord       = wingLength / aspectRatio;   % Lc [m]
hubLength       = 0.0031;             % Lb, hub/root width [m]
frameArea       = 1e-6;               % m^2, CFRP frame cross-section
membraneThk     = 50e-6;              % m
rhoCfrp         = 1400;               % kg/m^3
rhoMembrane     = 2000;               % kg/m^3
beamWidth       = 0.005;              % wb [m]
beamHeight      = 0.001;              % hb [m]

%% Aerodynamics
airDensity      = 1.225;      % kg/m^3
alphaCoeff      = 5e-3;       % Cα, passive wing rotation coefficient (empirical)

%% Wing inertia (about flap axis)
nElements       = 50;
rGrid           = linspace(hubLength/2, wingLength + hubLength/2, nElements);
dr              = rGrid(2) - rGrid(1);
chordGrid       = rootChord * sqrt(max(0, 1 - ((rGrid - hubLength/2) / wingLength).^2));

beamArea        = beamHeight * beamWidth;
Jbeam           = (1/3) * rhoCfrp * beamArea * hubLength^3;
JleadingEdge    = (1/3) * rhoCfrp * frameArea * wingLength * (wingLength + hubLength/2)^2;
JrootChord      = (1/3) * rhoCfrp * frameArea * rootChord * (hubLength/2)^2;
Jmembrane  = rhoMembrane * membraneThk * sum(chordGrid .* rGrid.^2 * dr);
Jw              = Jbeam + JleadingEdge + JrootChord + Jmembrane;

%% Added-mass virtual inertia (Sane-Dickinson 2002)
jAddedMass      = (pi/4) * airDensity * sum(chordGrid.^2 .* rGrid.^2) * dr;

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

%% Drive and spring
kSpring         = 0.001409;           % Nm/rad
driveFreq       = sqrt(kSpring / Jtotal) / (2*pi);            % Hz
driveOmega      = 2*pi*driveFreq;
driveVoltage    = ratedVoltage;                % V, sine amplitude

%% Solver
numCycles       = 25;
tStop           = numCycles / driveFreq;
solverStep      = 1e-5;

%% Quick natural-frequency report
naturalFreq = sqrt(kSpring / Jtotal) / (2*pi);
fprintf('Parameters loaded.\n');
fprintf('  Natural frequency: %.2f Hz\n', naturalFreq);
fprintf('  Drive frequency:   %.2f Hz\n', driveFreq);
fprintf('  Q (mech only):     %.2f\n', sqrt(Jtotal*kSpring)/bm);


fprintf('  Wing mass:  %.3f g\n', mWingTotal*1e3);
fprintf('  Total mass:        %.3f g\n', mTotal*1e3);
fprintf('  Weight to beat:    %.4f N  (%.2f mN)\n', weightTotal, weightTotal*1e3);
