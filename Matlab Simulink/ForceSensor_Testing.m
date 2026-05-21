%% Code for processing data from LabView (applicable for PCB Piezotronics 208C01 sensor)
data = readmatrix("test.csv");

t = data(:,1);
f = data(:,2) .* 1000;

baseline = 0;
f_zeroed = f - baseline;

%Plotting the test data
figure;
plot(t, f_zeroed, '-r');
xlabel('Time (s)');
ylabel('Force (mN)');
title('Impulse Graph');
grid on;

% Save processed data to CSV with timestamped filename
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
output_file = sprintf('test_%s.csv', timestamp);

writematrix([t, f_zeroed], output_file);

% Thresholds
pos_thresh = 0;
neg_thresh = 0;
above_pos = f_zeroed > pos_thresh;
below_neg = f_zeroed < neg_thresh;

% Event detection
d_pos = diff([0; above_pos; 0]);
start_pos = find(d_pos == 1);
end_pos   = find(d_pos == -1) - 1;

d_neg = diff([0; below_neg; 0]);
start_neg = find(d_neg == 1);
end_neg   = find(d_neg == -1) - 1;

n = length(t);

% --- Vectorized impulse computation ---
function [impulses, avg_forces] = compute_impulses(starts, ends, t, f, n)
    N = min(length(starts), length(ends));
    starts = starts(1:N);
    ends   = ends(1:N);

    valid = ends >= starts;
    impulses   = zeros(N, 1);
    avg_forces = zeros(N, 1);

    for i = find(valid)'
        idx = starts(i):ends(i);
        if max(idx) > n || length(idx) < 2, continue; end
        if any(~isfinite(t(idx))) || any(~isfinite(f(idx))), continue; end
        impulses(i)   = trapz(t(idx), f(idx));
        dur = t(ends(i)) - t(starts(i));
        if dur ~= 0
            avg_forces(i) = impulses(i) / dur;
        end
    end
end

[impulse_pos, avg_force_pos] = compute_impulses(start_pos, end_pos, t, f_zeroed, n);
[impulse_neg, avg_force_neg] = compute_impulses(start_neg, end_neg, t, f_zeroed, n);

% --- Batched area shading (much faster than looping area()) ---
hold on;
yline(pos_thresh, '--r');
yline(neg_thresh, '--r');

function shade_regions(starts, ends, t, f, color)
    N = min(length(starts), length(ends));
    if N == 0, return; end
    px = []; py = [];
    for i = 1:N
        idx = starts(i):ends(i);
        if isempty(idx), continue; end
        xi = t(idx)';
        yi = f(idx)';
        % Close the polygon: go along the data, then back along zero baseline
        px = [px, xi, fliplr(xi), xi(1), NaN];
        py = [py, yi, zeros(1, length(xi)), yi(1), NaN];
    end
    patch(px, py, color, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

shade_regions(start_pos, end_pos, t, f_zeroed, 'red');
shade_regions(start_neg, end_neg, t, f_zeroed, 'blue');

% Output table
maxLen = max([length(impulse_pos), length(impulse_neg), ...
              length(avg_force_pos), length(avg_force_neg)]);

impulse_pos(end+1:maxLen)   = NaN;
impulse_neg(end+1:maxLen)   = NaN;
avg_force_pos(end+1:maxLen) = NaN;
avg_force_neg(end+1:maxLen) = NaN;

T = table(impulse_pos, impulse_neg, avg_force_pos, avg_force_neg);
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
outputFile = fullfile(pwd, ['test_processed_' timestamp '.xlsx']);
writetable(T, outputFile);