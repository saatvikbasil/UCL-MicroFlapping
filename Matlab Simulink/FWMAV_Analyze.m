% FWMAV_Analyze  Analyse a single Simulink run and plot results.
% Run FWMAV_Setup, then sim('FWMAV_DirectDrive.slx'), then this script.

if ~exist('out', 'var')
    error('No simulation output found. Run the Simulink model first.');
end

%% Pull signals from the simulation output
t        = out.tout(:);
voltage  = out.voltage.signals.values(:);
current  = out.current.signals.values(:);
beta     = out.beta.signals.values(:);
betaDot  = out.betadot.signals.values(:);
lift     = out.lift.signals.values(:);

%% Restrict to the last few cycles for steady-state metrics
nSteady = min(5, numCycles - 2);
tSteady = t(end) - nSteady / driveFreq;
ss      = t >= tSteady;

tWindow       = t(ss) - tSteady;
voltageWindow = voltage(ss);
currentWindow = current(ss);
betaWindow    = beta(ss);
betaDotWindow = betaDot(ss);
liftWindow    = lift(ss);

%% Electrical metrics
peakCurrent  = max(abs(currentWindow));
rmsCurrent   = rms(currentWindow);
peakVoltage  = max(abs(voltageWindow));
rmsVoltage   = rms(voltageWindow);

instElecPower   = voltageWindow .* currentWindow;
meanElecPower   = mean(instElecPower);
resistiveLoss   = rmsCurrent^2 * Ra;

%% Mechanical metrics
peakStrokeDeg  = max(abs(betaWindow)) * 180/pi;
totalStrokeDeg = (max(betaWindow) - min(betaWindow)) * 180/pi;
peakBetaDot    = max(abs(betaDotWindow));

%% Aerodynamic metrics
meanLift = mean(abs(liftWindow));
peakLift = max(abs(liftWindow));

if exist('mWingTotal','var') && exist('motorMass','var')
    mTotalSys   = motorMass + mWingTotal;
    weightSys   = mTotalSys * 9.81;
    netLift     = meanLift - weightSys;
    fprintf('Weight:     %.2f mN  (motor %.1f g + wings %.1f g)\n', ...
        weightSys*1e3, motorMass*1e3, mWingTotal*1e3);
    if netLift > 0, flyStr = 'POSITIVE - can fly'; else, flyStr = 'NEGATIVE - cannot fly'; end
    fprintf('Net lift:   %.2f mN  (%s)\n', netLift*1e3, flyStr);
else
    fprintf('(Run FWMAV_Setup with wing mass block to see net lift)\n');
end

% Drag-power: instantaneous = drag torque * angular velocity, averaged
dragTorqueWindow = -(Kt * currentWindow ...
                     - bm * betaDotWindow ...
                     - kSpring * betaWindow ...
                     - tauCoulomb * sign(betaDotWindow) ...
                     - Jtotal * gradient(betaDotWindow, tWindow));
% (Reconstructed from the torque balance; equals what the aero block computed)

dragPower = mean(abs(dragTorqueWindow .* betaDotWindow));

% Ideal hover induced power (actuator-disk theory)
diskRadius   = wingLength + hubLength/2;
diskArea     = pi * diskRadius^2;
inducedPower = (meanLift^1.5) / sqrt(2 * airDensity * diskArea);

%% Effective angle of attack and aerodynamic coefficients over the cycle
alphaWindow = max(pi/4, pi/2 - alphaCoeff * abs(betaDotWindow));
clWindow    = 0.225 + 1.58 * sin(2.13 * alphaWindow - 0.1257);
cdWindow    = 1.92  - 1.55 * cos(2.04 * alphaWindow - 0.1714);

%% Efficiency summary
liftPerWatt = (meanLift * 1e3) / max(meanElecPower, eps);   % mN/W
figureOfMerit = inducedPower / max(meanElecPower, eps);
motorEfficiency = (meanElecPower - resistiveLoss) / max(meanElecPower, eps);

%% Print summary
fprintf('\n--- Simulation summary ---\n');
fprintf('Drive:      %.1f V at %.2f Hz\n', driveVoltage, driveFreq);
fprintf('Stroke:     +/- %.1f deg  (%.1f peak-to-peak)\n', peakStrokeDeg, totalStrokeDeg);
fprintf('Velocity:   %.1f rad/s peak\n', peakBetaDot);
fprintf('Current:    %.3f A peak, %.3f A rms\n', peakCurrent, rmsCurrent);
fprintf('Power in:   %.3f W (mean V*I)\n', meanElecPower);
fprintf('  I^2*R:    %.3f W (%.1f%%)\n', resistiveLoss, 100*resistiveLoss/meanElecPower);
fprintf('  Drag:     %.3f W (%.1f%%)\n', dragPower, 100*dragPower/meanElecPower);
fprintf('  Induced:  %.3f W (%.1f%%)\n', inducedPower, 100*inducedPower/meanElecPower);
fprintf('Lift:       %.2f mN avg, %.2f mN peak\n', meanLift*1e3, peakLift*1e3);
fprintf('L/P:        %.1f mN/W\n', liftPerWatt);
fprintf('FM:         %.3f\n', figureOfMerit);
fprintf('Motor eff:  %.1f%%\n', 100*motorEfficiency);
fprintf('Alpha:      %.1f to %.1f deg over cycle\n', ...
    min(alphaWindow)*180/pi, max(alphaWindow)*180/pi);

%% Time-series plots (3 cycles)
threeCycles = 3 / driveFreq;
plotMask    = tWindow <= threeCycles;
figure('Position', [100 100 1300 800]);

subplot(3,2,1);
plot(tWindow(plotMask)*1e3, voltageWindow(plotMask));
xlabel('time [ms]'); ylabel('voltage [V]'); title('Input voltage'); grid on;

subplot(3,2,2);
plot(tWindow(plotMask)*1e3, currentWindow(plotMask));
xlabel('time [ms]'); ylabel('current [A]');
title(sprintf('Motor current (peak %.2f A, rms %.2f A)', peakCurrent, rmsCurrent));
grid on;

subplot(3,2,3);
plot(tWindow(plotMask)*1e3, betaWindow(plotMask)*180/pi);
xlabel('time [ms]'); ylabel('stroke [deg]');
title(sprintf('Stroke angle (+/- %.1f deg)', peakStrokeDeg));
grid on;

subplot(3,2,4);
plot(tWindow(plotMask)*1e3, betaDotWindow(plotMask));
xlabel('time [ms]'); ylabel('omega [rad/s]');
title(sprintf('Angular velocity (peak %.0f rad/s)', peakBetaDot));
grid on;

subplot(3,2,5);
plot(tWindow(plotMask)*1e3, liftWindow(plotMask)*1e3);
hold on; yline(meanLift*1e3, '--');
xlabel('time [ms]'); ylabel('lift [mN]');
title(sprintf('Lift (avg %.2f mN)', meanLift*1e3));
grid on;

subplot(3,2,6);
plot(tWindow(plotMask)*1e3, alphaWindow(plotMask)*180/pi, 'DisplayName', '\alpha');
hold on;
plot(tWindow(plotMask)*1e3, clWindow(plotMask)*30, 'DisplayName', 'C_L \times 30');
plot(tWindow(plotMask)*1e3, cdWindow(plotMask)*30, 'DisplayName', 'C_D \times 30');
xlabel('time [ms]'); ylabel('alpha [deg]   /   coefficient \times 30');
title('Angle of attack and aero coefficients'); legend('show'); grid on;

sgtitle(sprintf('FWMAV results: %.1f V, %.1f Hz, %.1f mN/W', ...
    driveVoltage, driveFreq, liftPerWatt));
