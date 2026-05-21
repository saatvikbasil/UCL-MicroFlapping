    function result = FWMAV_SimulateInner(designLw, designFreq, designKspring, params)
% FWMAV_SimulateInner  Fast custom RK4 simulation matching the Simulink model.
% Used by FWMAV_Optimize for fast inner-loop evaluation. Returns mean lift,
% mean power, peak current, peak stroke and convergence flag.
%
% Inputs:
%   designLw       wing length [m]
%   designFreq     drive frequency [Hz]
%   designKspring  spring stiffness [Nm/rad]
%   params         struct with motor + wing + aero + drive parameters
%                  (see FWMAV_Setup for required fields)

driveOmega = 2*pi*designFreq;
nCycles    = params.numCycles;
nSteps     = round(nCycles / designFreq / params.solverStep);
dt         = 1 / (designFreq * round(1/(designFreq*params.solverStep)));

%% Wing geometry and inertia (recompute since Lw varies)
rootChord  = designLw / params.aspectRatio;
hubLength  = params.hubLength;

rGrid      = linspace(hubLength/2, designLw + hubLength/2, params.nElements);
dr         = rGrid(2) - rGrid(1);
chordGrid  = rootChord * sqrt(max(0, 1 - ((rGrid - hubLength/2) / designLw).^2));

rEffective = sum(rGrid .* (rGrid.^2 .* chordGrid) * dr) / ...
             sum(rGrid.^2 .* chordGrid * dr);

beamArea     = params.beamHeight * params.beamWidth;
Jbeam        = (1/3) * params.rhoCfrp * beamArea * hubLength^3;
JleadingEdge = (1/3) * params.rhoCfrp * params.frameArea * designLw * (designLw + hubLength/2)^2;
JrootChord   = (1/3) * params.rhoCfrp * params.frameArea * rootChord * (hubLength/2)^2;
Jmembrane    = params.rhoMembrane * params.membraneThk * (pi/4 * rootChord * designLw) * rEffective^2;
Jw           = Jbeam + JleadingEdge + JrootChord + Jmembrane;
jAddedMass   = (pi/4) * params.airDensity * sum(chordGrid.^2 .* rGrid.^2) * dr;
Jtotal       = params.Jm + Jw + jAddedMass;

%% Wing mass (needed for net lift)
beamArea_s    = params.beamHeight * params.beamWidth;
mBeam_s       = params.rhoCfrp * beamArea_s * hubLength;
mLeadEdge_s   = params.rhoCfrp * params.frameArea * designLw;
mRootChord_s  = params.rhoCfrp * params.frameArea * rootChord;
mMembrane_s   = params.rhoMembrane * params.membraneThk * (pi/4 * rootChord * designLw);
mWingTotal_s  = (mBeam_s + mLeadEdge_s + mRootChord_s + mMembrane_s);
mTotal_s      = params.motorMass + mWingTotal_s;
weightTotal_s = mTotal_s * 9.81;

%% Storage
t       = (0:nSteps) * dt;
beta    = zeros(1, nSteps+1);
betaDot = zeros(1, nSteps+1);
current = zeros(1, nSteps+1);
lift    = zeros(1, nSteps+1);
voltage = params.driveVoltage * sin(driveOmega * t);

%% RK4 integration of the 3-state system [beta, betaDot, current]
state = [0; 0];
for k = 1:nSteps
    v_k   = voltage(k);
    v_kh  = params.driveVoltage * sin(driveOmega * (t(k) + dt/2));
    v_kp1 = voltage(k+1);

    f1 = stateDeriv(state,              v_k,   designKspring, Jtotal, designLw, rootChord, hubLength, params);
    f2 = stateDeriv(state + dt/2 * f1,  v_kh,  designKspring, Jtotal, designLw, rootChord, hubLength, params);
    f3 = stateDeriv(state + dt/2 * f2,  v_kh,  designKspring, Jtotal, designLw, rootChord, hubLength, params);
    f4 = stateDeriv(state + dt   * f3,  v_kp1, designKspring, Jtotal, designLw, rootChord, hubLength, params);

    state = state + dt/6 * (f1 + 2*f2 + 2*f3 + f4);
    beta(k+1)    = state(1);
    betaDot(k+1) = state(2);
    % Recompute current algebraically for logging
    current(k+1) = (voltage(k+1) - params.Kb * state(2)) / params.Ra;
    lift(k+1)    = aeroLift(state(2), designLw, rootChord, hubLength, params);
end

%% Steady-state metrics (last 5 cycles)
ssWindow = t >= t(end) - 5/designFreq;
strokePeak    = max(abs(beta(ssWindow)));
betaDotPeak   = max(abs(betaDot(ssWindow)));
currentPeak   = max(abs(current(ssWindow)));
currentRms    = rms(current(ssWindow));
liftMean      = mean(abs(lift(ssWindow)));
powerMean     = mean(voltage(ssWindow) .* current(ssWindow));

%% Convergence: compare last cycle to second-last cycle peak amplitudes
lastCycleStart   = t >= t(end) - 1/designFreq;
prevCycleStart   = t >= t(end) - 2/designFreq & t < t(end) - 1/designFreq;
lastPeak = max(abs(beta(lastCycleStart)));
prevPeak = max(abs(beta(prevCycleStart)));
converged = abs(lastPeak - prevPeak) / max(lastPeak, eps) < 0.02;

result = struct( ...
    'strokePeakDeg', strokePeak * 180/pi, ...
    'betaDotPeak',   betaDotPeak, ...
    'currentPeak',   currentPeak, ...
    'currentRms',    currentRms, ...
    'liftMean',      liftMean, ...
    'liftMeanMn',    liftMean * 1e3, ...
    'powerMean',     powerMean, ...
    'liftPerWatt',   liftMean * 1e3 / max(powerMean, eps), ...
    'efficiency',    nan, ...
    'Jtotal',        Jtotal, ...
    'naturalFreq',   sqrt(designKspring/Jtotal)/(2*pi), ...
    'mWingTotal',   mWingTotal_s, ...
    'mTotal',       mTotal_s, ...
    'weightTotal',  weightTotal_s, ...
    'netLift',      liftMean - weightTotal_s, ...
    'netLiftMn',    (liftMean - weightTotal_s) * 1e3, ...
    'liftToWeight', liftMean / max(weightTotal_s, eps), ...
    'converged',     converged );

end

%% ----------------------------------------------------------------------
function dx = stateDeriv(x, v, kSpring, Jtotal, Lw, Lc, Lb, p)
% State: [beta; betaDot] only — current is algebraic
beta    = x(1);
betaDot = x(2);

current      = (v - p.Kb * betaDot) / p.Ra;   % quasi-static: La*di/dt ≈ 0
dragTorque   = aeroDragTorque(betaDot, Lw, Lc, Lb, p);
tauC         = p.tauCoulomb * tanh(betaDot / p.omegaSmooth);
betaDdot     = (p.Kt * current - p.bm * betaDot - kSpring * beta - dragTorque - tauC) / Jtotal;

dx = [betaDot; betaDdot];
end

%% ----------------------------------------------------------------------
function tau = aeroDragTorque(betaDot, Lw, Lc, Lb, p)
alpha = max(pi/4, pi/2 - p.alphaCoeff * abs(betaDot));
cd    = 1.92 - 1.55 * cos(2.04 * alpha - 0.1714);
cdRot = 5.0;

rRoot = Lb / 2;
rTip  = Lw + rRoot;
nEl   = 50;
dr    = (rTip - rRoot) / (nEl - 1);
halfRho = 0.5 * p.airDensity;

dragTrans = 0;
dragRotSum = 0;
for k = 1:nEl
    r = rRoot + (k-1) * dr;
    chord = Lc * sqrt(max(0, 1 - ((r - rRoot)/Lw)^2));
    dragTrans  = dragTrans  + halfRho * (r * betaDot)^2 * chord * dr * cd * r;
    dragRotSum = dragRotSum + cdRot * halfRho * chord^4 * dr / 8;
end
if betaDot < 0
    dragTrans = -dragTrans;
end
tau = dragTrans + dragRotSum * betaDot * abs(betaDot);
end

%% ----------------------------------------------------------------------
function L = aeroLift(betaDot, Lw, Lc, Lb, p)
alpha = max(pi/4, pi/2 - p.alphaCoeff * abs(betaDot));
cl    = 0.225 + 1.58 * sin(2.13 * alpha - 0.1257);

rRoot = Lb / 2;
rTip  = Lw + rRoot;
nEl   = 50;
dr    = (rTip - rRoot) / (nEl - 1);
halfRho = 0.5 * p.airDensity;

L = 0;
for k = 1:nEl
    r = rRoot + (k-1) * dr;
    chord = Lc * sqrt(max(0, 1 - ((r - rRoot)/Lw)^2));
    L = L + halfRho * (r * betaDot)^2 * chord * dr * cl;
end
end
