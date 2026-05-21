% FWMAV_Optimize  Find the wing length, drive frequency, and spring stiffness
% that maximise mean lift, subject to power and current limits.
%
% This script tries fmincon (Optimization Toolbox) first. If unavailable, it
% falls back to a multi-start fminsearch with penalty constraints (base
% MATLAB only). The inner simulation is a custom RK4 integrator
% (FWMAV_SimulateInner) that mirrors the Simulink physics for speed.

clear; clc;
FWMAV_Setup_RSPro_motor;

%% Pack parameters into a struct for the inner simulator
% Use a coarser solver step inside the optimiser for speed; the final answer
% is re-evaluated with the production (1e-5) step.
params = struct( ...
    'Ra', Ra, 'La', La, 'Kt', Kt, 'Kb', Kb, 'Jm', Jm, 'bm', bm, ...
    'tauCoulomb', tauCoulomb, 'omegaSmooth', omegaSmooth, ...
    'aspectRatio', aspectRatio, 'hubLength', hubLength, ...
    'frameArea', frameArea, 'membraneThk', membraneThk, ...
    'rhoCfrp', rhoCfrp, 'rhoMembrane', rhoMembrane, ...
    'beamWidth', beamWidth, 'beamHeight', beamHeight, ...
    'motorMass', motorMass, ...         
    'nElements', nElements, ...
    'airDensity', airDensity, 'alphaCoeff', alphaCoeff, ...
    'driveVoltage', driveVoltage, ...
    'numCycles', 15, 'solverStep', 1e-5);

paramsFinal = params;
paramsFinal.numCycles  = numCycles;
paramsFinal.solverStep = 1e-5;

%% Design variables: x = [wingLength, driveFreq, kSpring]
lb = [0.030, 4.0,  0.5e-3];
ub = [0.100, 25.0, 30e-3];

constraints = struct( ...
    'currentPeakMax', 1.5, ...   % A
    'powerMean',      2, ...    % W
    'strokePeakMax',  pi*(120/180), ...   % rad
    'minLift',        0);

%% Sanity check
x0 = [wingLength, driveFreq, kSpring];
fprintf('Sanity check at default x0:\n');
result0 = FWMAV_SimulateInner(x0(1), x0(2), x0(3), paramsFinal);
fprintf('  Lw=%.1f mm, freq=%.1f Hz, k=%.4f Nm/rad\n', x0(1)*1e3, x0(2), x0(3));
fprintf('  Net lift %.2f mN, Power %.3f W, I_pk %.3f A, stroke +/-%.1f deg\n\n', ...
    result0.netLiftMn, result0.powerMean, result0.currentPeak, result0.strokePeakDeg);

%% Generate Latin-hypercube starts (custom, no Statistics Toolbox required)
nStarts = 19;
rng(42);
startPoints = generateLhs(nStarts, 3, lb, ub);
startPoints = [x0; startPoints];          % include default x0
nStarts = size(startPoints, 1);

%% Multi-start optimisation loop
hasOptimToolbox = license('test', 'Optimization_Toolbox') && exist('fmincon','file') > 0;
if hasOptimToolbox
    fprintf('Optimization Toolbox detected, using fmincon (interior-point).\n');
else
    fprintf('Optimization Toolbox not available, using fminsearch + penalty fallback.\n');
end
fprintf('Running %d starts...\n\n', nStarts);

results(nStarts) = struct('x',[],'fval',[],'feasible',[],'flag',[],'sim',[]);
tStart = tic;

for k = 1:nStarts
    sp = startPoints(k,:);
    fprintf('Start %d/%d: x0=[%.1f mm, %.1f Hz, %.1f mNm/rad]\n', ...
        k, nStarts, sp(1)*1e3, sp(2), sp(3)*1e3);

    if hasOptimToolbox
        opts = optimoptions('fmincon', ...
            'Algorithm',                'interior-point', ...
            'Display',                  'off', ...
            'MaxIterations',            80, ...
            'MaxFunctionEvaluations',   400, ...
            'OptimalityTolerance',      1e-4, ...
            'StepTolerance',            1e-6, ...
            'ConstraintTolerance',      1e-5, ...
            'FiniteDifferenceType',     'central', ...
            'FiniteDifferenceStepSize', 1e-3, ...
            'ScaleProblem',             true);
        try
            [x, ~, flag] = fmincon( ...
                @(x) -FWMAV_SimulateInner(x(1),x(2),x(3),params).netLiftMn, ...
                sp, [], [], [], [], lb, ub, ...
                @(x) constraintFun(x, params, constraints), opts);
        catch ME
            fprintf('  fmincon error: %s\n', ME.message);
            x = sp; flag = -99;
        end
    else
        penalised = @(x) penalisedObjective(x, params, constraints, lb, ub);
        opts = optimset('Display','off', 'MaxIter', 200, 'MaxFunEvals', 600, ...
            'TolX', 1e-5, 'TolFun', 1e-4);
        [x, ~, flag] = fminsearch(penalised, sp, opts);
        x = max(lb, min(ub, x));
    end

    sim = FWMAV_SimulateInner(x(1), x(2), x(3), paramsFinal);
    feasible = sim.currentPeak     <= constraints.currentPeakMax + 1e-3 && ...
               sim.powerMean       <= constraints.powerMean        + 1e-3 && ...
               sim.strokePeakDeg   <= constraints.strokePeakMax*180/pi + 0.5 && ...
               sim.liftMean        >= constraints.minLift;

    results(k).x        = x;
    results(k).fval     = -sim.netLiftMn;
    results(k).feasible = feasible;
    results(k).flag     = flag;
    results(k).sim      = sim;

    fprintf('  -> Lw=%.2f mm, f=%.2f Hz, k=%.4f Nm/rad | lift=%.2f mN | feasible=%d\n\n', ...
        x(1)*1e3, x(2), x(3), sim.netLiftMn, feasible);
end

elapsed = toc(tStart);

%% Pick best feasible
feasMask = [results.feasible];
fvals    = [results.fval];
if any(feasMask)
    candidates = find(feasMask);
    [~, jj]    = min(fvals(candidates));
    bestIdx    = candidates(jj);
else
    fprintf('WARNING: no feasible result. Reporting best by objective.\n');
    [~, bestIdx] = min(fvals);
end

xOpt    = results(bestIdx).x;
simBest = results(bestIdx).sim;

%% Report
fprintf('=== Optimisation complete (%.1f s, %d starts) ===\n', elapsed, nStarts);
fprintf('Best design (start %d):\n', bestIdx);
fprintf('  wingLength   = %.2f mm\n', xOpt(1)*1e3);
fprintf('  driveFreq    = %.2f Hz\n', xOpt(2));
fprintf('  kSpring      = %.4f Nm/rad\n', xOpt(3));
fprintf('  k / (J*omega^2) = %.3f\n\n', xOpt(3) / (simBest.Jtotal * (2*pi*xOpt(2))^2));
fprintf('Performance:\n');
fprintf('  Gross lift     %.2f mN\n', simBest.liftMeanMn);
fprintf('  System weight  %.2f mN  (motor %.1f g + wing %.1f g)\n', simBest.weightTotal*1e3, params.motorMass*1e3, simBest.mWingTotal*1e3);
fprintf('  Net lift       %.2f mN\n', simBest.netLiftMn);
fprintf('  Lift / weight  %.3f\n',    simBest.liftToWeight);
fprintf('  Mean power     %.3f W\n', simBest.powerMean);
fprintf('  Lift / power   %.1f mN/W\n', simBest.liftPerWatt);
fprintf('  Peak current   %.3f A\n', simBest.currentPeak);
fprintf('  RMS current    %.3f A\n', simBest.currentRms);
fprintf('  Peak stroke    +/-%.1f deg\n', simBest.strokePeakDeg);
fprintf('  Natural freq   %.2f Hz\n', simBest.naturalFreq);

save('FWMAV_OptimizeResult.mat', 'xOpt', 'simBest', 'results', 'lb', 'ub', 'params');

%% Plot all local optima
xs = arrayfun(@(r) r.x(1)*1e3, results);
ys = arrayfun(@(r) r.x(2),     results);
ks = arrayfun(@(r) r.x(3)*1e3, results);
fs = -[results.fval];

figure('Position', [100 100 1000 450]);
subplot(1,2,1);
scatter(xs(~feasMask), ys(~feasMask), 80, [0.7 0.7 0.7], 'filled'); hold on;
scatter(xs( feasMask), ys( feasMask), 80, fs(feasMask), 'filled');
plot(xOpt(1)*1e3, xOpt(2), 'r*', 'MarkerSize', 20, 'LineWidth', 2);
colorbar; xlabel('wing length [mm]'); ylabel('drive frequency [Hz]');
title('Local optima (red star = best, grey = infeasible)'); grid on;

subplot(1,2,2);
scatter(ys(~feasMask), ks(~feasMask), 80, [0.7 0.7 0.7], 'filled'); hold on;
scatter(ys( feasMask), ks( feasMask), 80, fs(feasMask), 'filled');
plot(xOpt(2), xOpt(3)*1e3, 'r*', 'MarkerSize', 20, 'LineWidth', 2);
colorbar; xlabel('drive frequency [Hz]'); ylabel('kSpring [mNm/rad]');
title('Spring vs frequency'); grid on;

sgtitle('FWMAV optimisation: local optima from multi-start');
saveas(gcf, 'FWMAV_OptimizeResult.png');
fprintf('Saved FWMAV_OptimizeResult.png\n');

%% --------------------------------------------------------------------
function [c, ceq] = constraintFun(x, params, lim)
    sim = FWMAV_SimulateInner(x(1), x(2), x(3), params);
    c = [
        sim.currentPeak - lim.currentPeakMax;
        sim.powerMean    - lim.powerMean;
        sim.strokePeakDeg*pi/180 - lim.strokePeakMax;
        lim.minLift - sim.netLift ];
    ceq = [];
end

%% --------------------------------------------------------------------
function f = penalisedObjective(x, params, lim, lb, ub)
    bx = max(lb, min(ub, x));
    boundPenalty = 1e3 * sum(max(0, x - ub).^2 + max(0, lb - x).^2);
    sim = FWMAV_SimulateInner(bx(1), bx(2), bx(3), params);
    cv = [
        sim.currentPeak - lim.currentPeakMax;
        sim.powerMean    - lim.powerMean;
        sim.strokePeakDeg*pi/180 - lim.strokePeakMax;
        lim.minLift - sim.netLift ];
    constraintPenalty = 1e3 * sum(max(0, cv).^2);
    f = -sim.netLiftMn + boundPenalty + constraintPenalty;
end

%% --------------------------------------------------------------------
function pts = generateLhs(n, dim, lb, ub)
    pts = zeros(n, dim);
    for d = 1:dim
        bins = (0:n-1)' / n + rand(n,1) / n;
        bins = bins(randperm(n));
        pts(:,d) = lb(d) + bins * (ub(d) - lb(d));
    end
end
