% FWMAV_DesignSpace  Map performance over (wing length, drive frequency).
% Produces contour heatmaps of mean lift, mean power, peak current, and
% peak stroke for a fixed (resonant) spring stiffness, plus a Pareto plot
% of lift versus power. Designed for the final-year report.

clear; clc;
FWMAV_Setup;

params = struct( ...
    'Ra', Ra, 'La', La, 'Kt', Kt, 'Kb', Kb, 'Jm', Jm, 'bm', bm, ...
    'tauCoulomb', tauCoulomb, 'omegaSmooth', omegaSmooth, ...
    'aspectRatio', aspectRatio, 'hubLength', hubLength, ...
    'frameArea', frameArea, 'membraneThk', membraneThk, ...
    'rhoCfrp', rhoCfrp, 'rhoMembrane', rhoMembrane, ...
    'beamWidth', beamWidth, 'beamHeight', beamHeight, ...
    'nElements', nElements, ...
    'airDensity', airDensity, 'alphaCoeff', alphaCoeff, ...
    'motorMass', motorMass, 'driveVoltage', driveVoltage, ...
    'numCycles', numCycles, 'solverStep', 1e-5);

%% Grid
nLw   = 18;
nFreq = 18;
lwGrid   = linspace(0.040, 0.090, nLw);
freqGrid = linspace(6, 22, nFreq);
[LW, FQ] = meshgrid(lwGrid, freqGrid);

liftMap    = zeros(size(LW));
powerMap   = zeros(size(LW));
currentMap = zeros(size(LW));
strokeMap  = zeros(size(LW));
liftPerWMap = zeros(size(LW));

%% Sweep with kSpring set to resonance for each (Lw, freq) pair
nTotal = numel(LW);
fprintf('Evaluating %d design points...\n', nTotal);
tStart = tic;

useParallel = ~isempty(ver('parallel'));
if useParallel && isempty(gcp('nocreate'))
    parpool('Processes', min(8, feature('numcores')));
end

paramCell = cell(nTotal, 1);
for k = 1:nTotal
    paramCell{k} = params;
end

LwFlat   = LW(:);
FqFlat   = FQ(:);
liftFlat   = zeros(nTotal,1);
powerFlat  = zeros(nTotal,1);
currentFlat= zeros(nTotal,1);
strokeFlat = zeros(nTotal,1);
ratioFlat  = zeros(nTotal,1);

parfor k = 1:nTotal
    Lw   = LwFlat(k);
    fq   = FqFlat(k);
    p    = paramCell{k};
    Lc   = Lw / p.aspectRatio;
    rGrid = linspace(p.hubLength/2, Lw + p.hubLength/2, p.nElements);
    dr    = rGrid(2) - rGrid(1);
    chord = Lc * sqrt(max(0, 1 - ((rGrid - p.hubLength/2)/Lw).^2));
    rEff  = sum(rGrid .* (rGrid.^2 .* chord) * dr) / sum(rGrid.^2 .* chord * dr);
    Jb    = (1/3) * p.rhoCfrp * (p.beamHeight*p.beamWidth) * p.hubLength^3;
    Jle   = (1/3) * p.rhoCfrp * p.frameArea * Lw * (Lw + p.hubLength/2)^2;
    Jrc   = (1/3) * p.rhoCfrp * p.frameArea * Lc * (p.hubLength/2)^2;
    Jmb   = p.rhoMembrane * p.membraneThk * (pi/4 * Lc * Lw) * rEff^2;
    Jaero = (pi/4) * p.airDensity * sum(chord.^2 .* rGrid.^2) * dr;
    Jt    = p.Jm + Jb + Jle + Jrc + Jmb + Jaero;
    kS    = Jt * (2*pi*fq)^2;

    res = FWMAV_SimulateInner(Lw, fq, kS, p);
    liftFlat(k)   = res.liftMeanMn;
    powerFlat(k)  = res.powerMean;
    currentFlat(k)= res.currentPeak;
    strokeFlat(k) = res.strokePeakDeg;
    ratioFlat(k)  = res.liftPerWatt;
end

liftMap     = reshape(liftFlat, size(LW));
powerMap    = reshape(powerFlat, size(LW));
currentMap  = reshape(currentFlat, size(LW));
strokeMap   = reshape(strokeFlat, size(LW));
liftPerWMap = reshape(ratioFlat, size(LW));

elapsed = toc(tStart);
fprintf('Done in %.1f s.\n', elapsed);

%% Heatmaps
figure('Position', [80 80 1400 850]);

subplot(2,3,1);
contourf(LW*1e3, FQ, liftMap, 18, 'LineStyle','none');
colormap(gca, parula); cb = colorbar; cb.Label.String = 'mN';
xlabel('wing length [mm]'); ylabel('drive frequency [Hz]');
title('Mean lift'); hold on;
contour(LW*1e3, FQ, currentMap, [0.55 0.55], 'r-', 'LineWidth', 2);
contour(LW*1e3, FQ, powerMap,   [1.5 1.5],   'k-', 'LineWidth', 2);

subplot(2,3,2);
contourf(LW*1e3, FQ, powerMap, 18, 'LineStyle','none');
colormap(gca, hot); cb = colorbar; cb.Label.String = 'W';
xlabel('wing length [mm]'); ylabel('drive frequency [Hz]');
title('Mean electrical power');

subplot(2,3,3);
contourf(LW*1e3, FQ, currentMap, 18, 'LineStyle','none');
colormap(gca, hot); cb = colorbar; cb.Label.String = 'A';
xlabel('wing length [mm]'); ylabel('drive frequency [Hz]');
title('Peak current'); hold on;
contour(LW*1e3, FQ, currentMap, [0.55 0.55], 'r-', 'LineWidth', 2);

subplot(2,3,4);
contourf(LW*1e3, FQ, strokeMap, 18, 'LineStyle','none');
colormap(gca, parula); cb = colorbar; cb.Label.String = 'deg';
xlabel('wing length [mm]'); ylabel('drive frequency [Hz]');
title('Peak stroke');

subplot(2,3,5);
contourf(LW*1e3, FQ, liftPerWMap, 18, 'LineStyle','none');
colormap(gca, parula); cb = colorbar; cb.Label.String = 'mN/W';
xlabel('wing length [mm]'); ylabel('drive frequency [Hz]');
title('Lift / power ratio');

%% Pareto: lift vs power, with feasible region marked
subplot(2,3,6);
feasible = currentMap <= 0.55 & powerMap <= 1.5 & strokeMap <= 90;
scatter(powerMap(:), liftMap(:), 25, currentMap(:), 'filled'); hold on;
scatter(powerMap(feasible), liftMap(feasible), 50, 'g', 'LineWidth', 1);
xlabel('mean power [W]'); ylabel('mean lift [mN]');
title('Performance scatter (green = feasible)');
cb = colorbar; cb.Label.String = 'I peak [A]';
grid on;

sgtitle(sprintf('FWMAV design space (V_{drive} = %.1f V, k tuned to resonance per point)', driveVoltage));

saveas(gcf, 'FWMAV_DesignSpace.png');
fprintf('Saved FWMAV_DesignSpace.png\n');

%% Spring detuning sensitivity at the best operating point
[~, idxBest] = max(liftMap(feasible));
feasIdx = find(feasible);
bestIdx = feasIdx(idxBest);
LwBest = LW(bestIdx);
FqBest = FQ(bestIdx);

fprintf('\nBest feasible point: Lw=%.1f mm, freq=%.1f Hz, lift=%.2f mN\n', ...
    LwBest*1e3, FqBest, liftMap(bestIdx));

% Compute Jtotal for this best point
Lc = LwBest / params.aspectRatio;
rGrid = linspace(params.hubLength/2, LwBest + params.hubLength/2, params.nElements);
dr_   = rGrid(2) - rGrid(1);
chord = Lc * sqrt(max(0, 1 - ((rGrid - params.hubLength/2)/LwBest).^2));
rEff  = sum(rGrid .* (rGrid.^2 .* chord) * dr_) / sum(rGrid.^2 .* chord * dr_);
Jt = params.Jm + ...
     (1/3) * params.rhoCfrp * (params.beamHeight*params.beamWidth) * params.hubLength^3 + ...
     (1/3) * params.rhoCfrp * params.frameArea * LwBest * (LwBest + params.hubLength/2)^2 + ...
     (1/3) * params.rhoCfrp * params.frameArea * Lc * (params.hubLength/2)^2 + ...
     params.rhoMembrane * params.membraneThk * (pi/4 * Lc * LwBest) * rEff^2 + ...
     (pi/4) * params.airDensity * sum(chord.^2 .* rGrid.^2) * dr_;

kResonance = Jt * (2*pi*FqBest)^2;
ratios = linspace(0.5, 1.5, 11);
liftDetune   = zeros(size(ratios));
strokeDetune = zeros(size(ratios));
for k = 1:numel(ratios)
    res = FWMAV_SimulateInner(LwBest, FqBest, ratios(k)*kResonance, params);
    liftDetune(k)   = res.liftMeanMn;
    strokeDetune(k) = res.strokePeakDeg;
end

figure('Position',[200 200 900 450]);
subplot(1,2,1); plot(ratios, liftDetune, '-o', 'LineWidth',1.5);
xline(1, '--k'); xlabel('k_{spring} / k_{resonance}'); ylabel('lift [mN]');
title('Spring detuning'); grid on;
subplot(1,2,2); plot(ratios, strokeDetune, '-o', 'LineWidth',1.5);
xline(1, '--k'); xlabel('k_{spring} / k_{resonance}'); ylabel('stroke [deg]');
title('Stroke vs spring detuning'); grid on;
sgtitle(sprintf('Spring sensitivity at L_w=%.1f mm, f=%.1f Hz', LwBest*1e3, FqBest));
saveas(gcf, 'FWMAV_SpringSensitivity.png');
fprintf('Saved FWMAV_SpringSensitivity.png\n');

save('FWMAV_DesignSpace.mat', 'LW','FQ','liftMap','powerMap','currentMap','strokeMap','liftPerWMap');
