% FWMAV_FrequencySweep  Sweep drive frequency, compare to experimental stroke.
% Identifies the system's resonance peak and overlays measured data from
% the high-speed-camera test (wing + spring configuration).

FWMAV_Setup_RSPro_motor;

%% Frequencies to test (Hz)
sweepFreqs = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 17, 20, 23, 25];
nFreq      = numel(sweepFreqs);

%% Experimental peak stroke at the same drive voltage (wing + spring)
% Read off the high-speed-camera plot. Approximate.
expFreqs  = [7.5, 10, 15, 20, 25];
expStroke = [70,  45, 15, 10,  5];

%% Storage
peakStrokeDeg = zeros(1, nFreq);
peakCurrentA  = zeros(1, nFreq);
meanLiftMn    = zeros(1, nFreq);
meanPowerW    = zeros(1, nFreq);

%% Run the sweep
fprintf('Sweeping %d frequencies from %.1f to %.1f Hz...\n', ...
    nFreq, sweepFreqs(1), sweepFreqs(end));

for k = 1:nFreq
    driveFreq  = sweepFreqs(k);
    driveOmega = 2*pi*driveFreq;
    tStop      = numCycles / driveFreq;

    out = sim('FWMAV_DirectDrive.slx', ...
        'StopTime', num2str(tStop), ...
        'SrcWorkspace', 'current');

    t  = out.tout(:);
    ss = t >= t(end) - min(5, numCycles - 2)/driveFreq;

    beta    = out.beta.signals.values(:);
    current = out.current.signals.values(:);
    lift    = out.lift.signals.values(:);
    voltage = out.voltage.signals.values(:);

    peakStrokeDeg(k) = max(abs(beta(ss))) * 180/pi;
    peakCurrentA(k)  = max(abs(current(ss)));
    meanLiftMn(k)    = mean(abs(lift(ss))) * 1e3;
    meanPowerW(k)    = mean(voltage(ss) .* current(ss));

    fprintf('  %.2f Hz: stroke %.1f deg, I_peak %.2f A, lift %.1f mN\n', ...
        driveFreq, peakStrokeDeg(k), peakCurrentA(k), meanLiftMn(k));
end

%% Identify model resonance
[strokeMax, idxRes] = max(peakStrokeDeg);
freqRes = sweepFreqs(idxRes);
fprintf('\nModel peak stroke %.1f deg at %.2f Hz.\n', strokeMax, freqRes);

%% Plot
figure('Position', [100 100 1100 700]);

subplot(2,2,1);
plot(sweepFreqs, peakStrokeDeg, '-o', 'LineWidth', 1.5, 'DisplayName', 'Model');
hold on;
plot(expFreqs, expStroke, 's', 'MarkerSize', 8, 'LineWidth', 1.5, ...
     'DisplayName', 'Experiment');
xline(freqRes, '--', sprintf('Model peak %.1f Hz', freqRes), ...
      'LabelVerticalAlignment', 'bottom');
xlabel('drive frequency [Hz]');
ylabel('peak stroke [deg]');
title('Stroke vs frequency');
legend('Location', 'best');
grid on;

subplot(2,2,2);
plot(sweepFreqs, peakCurrentA, '-o', 'LineWidth', 1.5);
xlabel('drive frequency [Hz]');
ylabel('peak current [A]');
title('Current vs frequency');
grid on;

subplot(2,2,3);
plot(sweepFreqs, meanLiftMn, '-o', 'LineWidth', 1.5);
xlabel('drive frequency [Hz]');
ylabel('mean lift [mN]');
title('Lift vs frequency');
grid on;

subplot(2,2,4);
plot(sweepFreqs, meanPowerW, '-o', 'LineWidth', 1.5);
xlabel('drive frequency [Hz]');
ylabel('mean electrical power [W]');
title('Power vs frequency');
grid on;

sgtitle(sprintf('Frequency sweep at %.1f V (Q estimate: %.2f)', ...
    driveVoltage, sqrt(Jtotal*kSpring)/bm));