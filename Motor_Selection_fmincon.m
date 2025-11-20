%% Motor_Selection_MultiStart.m
% FWMAV Optimization using MultiStart for robust optimization
% Based on Park & Abolfathi (2024) Direct Drive Model

clear; clc; close all;
rng(0); % Fix random seed for reproducibility

fprintf('\n');
fprintf('=============================================\n');
fprintf('  FWMAV Motor Selection - MultiStart\n');
fprintf('=============================================\n\n');

%% Check toolboxes
if ~license('test', 'Optimization_Toolbox')
    error('Optimization Toolbox not found!');
end

if ~license('test', 'GADS_Toolbox')
    error('Global Optimization Toolbox not found!');
end

%% ==================== MOTOR PARAMETERS ====================
motor.Ra = 9;               % Armature resistance [Ohm]
motor.La = 0.029;           % Armature inductance [H]
motor.Kt = 0.01;            % Torque constant [Nm/A]
motor.Kb = 0.01;            % Back-EMF constant [V/(rad/s)]
motor.Jm = 1.5e-7;          % Motor inertia [kg·m²]
motor.bm = 3.6e-6;          % Mechanical damping [Ns/rad]

V_amplitude = 5;            % Input voltage [V]

fprintf('Motor Specifications:\n');
fprintf('  Resistance:    %.1f Ω\n', motor.Ra);
fprintf('  Torque const:  %.2f mNm/A\n', motor.Kt*1000);
fprintf('  Voltage:       %.1f V\n\n', V_amplitude);

%% ==================== MATERIAL PROPERTIES ====================
materials.rho_air = 1.225;       % Air density [kg/m³]
materials.Calpha = 5e-3;         % Passive rotation coefficient
materials.rho_wf = 1900;         % CFRP density [kg/m³]
materials.rho_mb = 1100;         % Polyimide density [kg/m³]
materials.Awf = 1e-6;            % Frame cross-section [m²]
materials.t_mb = 25e-6;          % Membrane thickness [m]
materials.aspect_ratio = 3;      % Le/Lc ratio
materials.Lb = 0.010;            % Wing base length [m]
materials.Awb = 0.001 * 0.005;   % Wing base area [m²]

%% ==================== POWER LIMIT ====================
P_max = 1.5 * (V_amplitude/6)^2;  % Power limit scales with V²
fprintf('Power constraint: %.2f W (scaled for %.1fV)\n\n', P_max, V_amplitude);

%% ==================== OPTIMIZATION SETUP ====================
fprintf('Setting up MultiStart optimization...\n\n');

% Bounds based on voltage
if V_amplitude <= 3
    lb = [0.040, 28];   % 3V: higher frequency
    ub = [0.075, 40];
    freq_range = [28, 32, 36];  % Test around 35 Hz (paper value)
    Le_range = linspace(0.050, 0.070, 5);
elseif V_amplitude <= 5
    lb = [0.050, 18];   % 5V: lower frequency  
    ub = [0.085, 32];
    freq_range = [20, 24, 28];  % Test around 24 Hz (paper value)
    Le_range = linspace(0.055, 0.075, 5);
else
    lb = [0.030, 15];
    ub = [0.100, 40];
    freq_range = linspace(lb(2), ub(2), 5);
    Le_range = linspace(lb(1), ub(1), 5);
end

% Create systematic starting points
n_starts = length(Le_range) * length(freq_range);
start_points = zeros(n_starts, 2);
idx = 1;
for i = 1:length(Le_range)
    for j = 1:length(freq_range)
        start_points(idx, :) = [Le_range(i), freq_range(j)];
        idx = idx + 1;
    end
end

fprintf('Created %d systematic starting points\n', n_starts);
fprintf('  Le range: %.1f - %.1f mm\n', Le_range(1)*1000, Le_range(end)*1000);
fprintf('  Freq range: %.1f - %.1f Hz\n\n', freq_range(1), freq_range(end));

% Local solver options (FASTER simulation)
local_options = optimoptions('fmincon', ...
    'Display', 'off', ...
    'Algorithm', 'sqp', ...
    'MaxIterations', 150, ...
    'MaxFunctionEvaluations', 400, ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-10, ...
    'ConstraintTolerance', 1e-6, ...
    'FiniteDifferenceStepSize', 1e-5, ...
    'FiniteDifferenceType', 'central', ...
    'UseParallel', false);

% Objective and constraints
objective = @(x) objective_function(x, V_amplitude, motor, materials);
nonlcon = @(x) constraint_function(x, V_amplitude, motor, materials, P_max);

% Create optimization problem
problem = createOptimProblem('fmincon', ...
    'objective', objective, ...
    'x0', start_points(1,:), ...  % Will be overridden by MultiStart
    'lb', lb, ...
    'ub', ub, ...
    'nonlcon', nonlcon, ...
    'options', local_options);

% Create CustomStartPointSet with our systematic points
startpts = CustomStartPointSet(start_points);

% MultiStart settings
ms = MultiStart(...
    'Display', 'iter', ...
    'UseParallel', false, ...
    'StartPointsToRun', 'all');  % Run ALL starting points

fprintf('Starting MultiStart optimization...\n');
fprintf('Testing all %d starting points systematically.\n', n_starts);
fprintf('Expected time: 3-5 minutes.\n\n');

%% ==================== RUN OPTIMIZATION ====================
tic;
[x_opt, fval, exitflag, output, solutions] = run(ms, problem, startpts);
elapsed_time = toc;

fprintf('\n✓ MultiStart complete in %.1f minutes!\n', elapsed_time/60);
fprintf('  Ran %d optimizations\n', length(solutions));
fprintf('  Function evaluations: %d\n', output.funcCount);

%% ==================== EXTRACT OPTIMAL DESIGN ====================
Le_opt = x_opt(1);
freq_opt = x_opt(2);

% Calculate optimal spring stiffness
Lc_opt = Le_opt / materials.aspect_ratio;
Jw_opt = calculate_wing_inertia(Le_opt, Lc_opt, materials);
J_total_opt = motor.Jm + Jw_opt;
KDD_opt = J_total_opt * (2*pi*freq_opt)^2;

% Get performance at optimum
[lift_opt, Pe_opt, Pa_opt, eff_opt, beta_max, converged] = evaluate_design(...
    Le_opt, freq_opt, V_amplitude, motor, materials);

%% ==================== DISPLAY ALL LOCAL MINIMA ====================
fprintf('\n=============================================\n');
fprintf('         ALL LOCAL MINIMA FOUND\n');
fprintf('=============================================\n');

% Extract and sort all solutions
all_solutions = [];
for i = 1:length(solutions)
    if solutions(i).Exitflag > 0  % Only converged solutions
        Le_i = solutions(i).X(1);
        freq_i = solutions(i).X(2);
        lift_i = -solutions(i).Fval;
        
        % Calculate KDD
        Lc_i = Le_i / materials.aspect_ratio;
        Jw_i = calculate_wing_inertia(Le_i, Lc_i, materials);
        J_total_i = motor.Jm + Jw_i;
        KDD_i = J_total_i * (2*pi*freq_i)^2;
        
        all_solutions = [all_solutions; Le_i*1000, freq_i, lift_i*1000, KDD_i*1000];
    end
end

% Sort by lift (descending)
[~, sort_idx] = sort(all_solutions(:,3), 'descend');
all_solutions = all_solutions(sort_idx, :);

% Display top solutions
n_display = min(10, size(all_solutions, 1));
fprintf('Top %d solutions (sorted by lift):\n', n_display);
fprintf('---------------------------------------------------\n');
fprintf(' Rank  Le[mm]  Freq[Hz]  Lift[mN]  KDD[mNm/rad]\n');
fprintf('---------------------------------------------------\n');
for i = 1:n_display
    fprintf('  %2d   %5.1f    %5.1f    %6.2f      %6.2f\n', ...
        i, all_solutions(i,1), all_solutions(i,2), all_solutions(i,3), all_solutions(i,4));
end
fprintf('=============================================\n\n');

%% ==================== DISPLAY OPTIMAL RESULTS ====================
fprintf('=============================================\n');
fprintf('         GLOBAL OPTIMAL DESIGN\n');
fprintf('=============================================\n');
fprintf('WING DESIGN:\n');
fprintf('  Leading Edge:    %.2f mm\n', Le_opt*1000);
fprintf('  Root Chord:      %.2f mm\n', Lc_opt*1000);
fprintf('  Aspect Ratio:    %.1f\n', materials.aspect_ratio);
fprintf('\nOPERATION:\n');
fprintf('  Frequency:       %.2f Hz\n', freq_opt);
fprintf('  Spring Stiffness: %.2f mNm/rad\n', KDD_opt*1000);
fprintf('  Wing Inertia:    %.2e kg·m²\n', Jw_opt);
fprintf('  Max Stroke:      %.1f deg\n', beta_max);
fprintf('\nPERFORMANCE:\n');
fprintf('  Average Lift:    %.2f mN\n', lift_opt*1000);
fprintf('  Input Power:     %.3f W (limit: %.2f W)\n', Pe_opt, P_max);
fprintf('  Aero Power:      %.3f W\n', Pa_opt);
fprintf('  Efficiency:      %.2f %%\n', eff_opt);
fprintf('  Converged:       %s\n', bool2str(converged));
fprintf('\nOPTIMIZATION INFO:\n');
fprintf('  Exit flag:       %d\n', exitflag);
fprintf('  Total solutions: %d\n', size(all_solutions, 1));
fprintf('=============================================\n\n');

%% ==================== COMPARISON TO PAPER ====================
if V_amplitude == 5
    fprintf('COMPARISON TO PAPER (5V, Direct Drive):\n');
    fprintf('  Paper Le:        68.46 mm (yours: %.2f mm, %.1f%% diff)\n', ...
        Le_opt*1000, abs(Le_opt*1000 - 68.46)/68.46*100);
    fprintf('  Paper freq:      23.89 Hz (yours: %.2f Hz, %.1f%% diff)\n', ...
        freq_opt, abs(freq_opt - 23.89)/23.89*100);
    fprintf('  Paper KDD:       8.65 mNm/rad (yours: %.2f mNm/rad, %.1f%% diff)\n', ...
        KDD_opt*1000, abs(KDD_opt*1000 - 8.65)/8.65*100);
    fprintf('  Paper lift:      32.35 mN (yours: %.2f mN, %.1f%% diff)\n', ...
        lift_opt*1000, abs(lift_opt*1000 - 32.35)/32.35*100);
    fprintf('  Paper eff:       29.87 %% (yours: %.2f %%, %.1f%% diff)\n\n', ...
        eff_opt, abs(eff_opt - 29.87)/29.87*100);
elseif V_amplitude == 3
    fprintf('COMPARISON TO PAPER (3V, Direct Drive):\n');
    fprintf('  Paper Le:        60.49 mm (yours: %.2f mm, %.1f%% diff)\n', ...
        Le_opt*1000, abs(Le_opt*1000 - 60.49)/60.49*100);
    fprintf('  Paper freq:      35.67 Hz (yours: %.2f Hz, %.1f%% diff)\n', ...
        freq_opt, abs(freq_opt - 35.67)/35.67*100);
    fprintf('  Paper KDD:       15.80 mNm/rad (yours: %.2f mNm/rad, %.1f%% diff)\n', ...
        KDD_opt*1000, abs(KDD_opt*1000 - 15.80)/15.80*100);
    fprintf('  Paper lift:      12.31 mN (yours: %.2f mN, %.1f%% diff)\n', ...
        lift_opt*1000, abs(lift_opt*1000 - 12.31)/12.31*100);
    fprintf('  Paper eff:       33.74 %% (yours: %.2f %%, %.1f%% diff)\n\n', ...
        eff_opt, abs(eff_opt - 33.74)/33.74*100);
end

%% ==================== VISUALIZATION ====================
fprintf('Generating visualization...\n');

figure('Position', [100 100 1600 600]);

% Plot 1: All solutions scatter
subplot(1,3,1);
scatter(all_solutions(:,1), all_solutions(:,2), 100, all_solutions(:,3), 'filled');
hold on;
plot(Le_opt*1000, freq_opt, 'r*', 'MarkerSize', 30, 'LineWidth', 4);
xlabel('Wing Length [mm]');
ylabel('Frequency [Hz]');
title('All Local Minima Found');
colorbar;
ylabel(colorbar, 'Lift [mN]');
grid on;
legend('Local minima', 'Global optimum', 'Location', 'best');

% Plot 2: Lift vs Frequency
subplot(1,3,2);
scatter(all_solutions(:,2), all_solutions(:,3), 100, all_solutions(:,1), 'filled');
hold on;
plot(freq_opt, lift_opt*1000, 'r*', 'MarkerSize', 30, 'LineWidth', 4);
xlabel('Frequency [Hz]');
ylabel('Lift [mN]');
title('Lift vs Frequency');
colorbar;
ylabel(colorbar, 'Wing Length [mm]');
grid on;

% Plot 3: Lift vs Wing Length
subplot(1,3,3);
scatter(all_solutions(:,1), all_solutions(:,3), 100, all_solutions(:,2), 'filled');
hold on;
plot(Le_opt*1000, lift_opt*1000, 'r*', 'MarkerSize', 30, 'LineWidth', 4);
xlabel('Wing Length [mm]');
ylabel('Lift [mN]');
title('Lift vs Wing Length');
colorbar;
ylabel(colorbar, 'Frequency [Hz]');
grid on;

sgtitle(sprintf('MultiStart Results: %d Solutions Found', size(all_solutions,1)), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% ==================== SAVE RESULTS ====================
save('motor_selection_multistart.mat', 'x_opt', 'Le_opt', 'freq_opt', 'KDD_opt', ...
    'lift_opt', 'Pe_opt', 'Pa_opt', 'eff_opt', 'motor', 'materials', ...
    'output', 'solutions', 'all_solutions');

fprintf('✓ Results saved to motor_selection_multistart.mat\n\n');

fprintf('=============================================\n');
fprintf('  MultiStart Optimization Complete!\n');
fprintf('=============================================\n');

%% ==================== HELPER FUNCTIONS ====================

function cost = objective_function(x, V_amp, motor, materials)
    Le = x(1);
    freq = x(2);
    
    try
        [lift, ~, ~, ~, ~, converged] = evaluate_design(Le, freq, V_amp, motor, materials);
        
        if ~converged
            cost = 1e6;
        else
            cost = -lift;
        end
        
    catch
        cost = 1e6;
    end
end

function [c, ceq] = constraint_function(x, V_amp, motor, materials, P_max)
    Le = x(1);
    freq = x(2);
    
    try
        [~, Pe, ~, eff, ~, ~] = evaluate_design(Le, freq, V_amp, motor, materials);
        
        c = [
            Pe - P_max;
            12 - eff;
            eff - 50;
        ];
        
        ceq = [];
        
    catch
        c = [1e6; 1e6; 1e6];
        ceq = [];
    end
end

function [lift_avg, Pe, Pa, eff, beta_max, converged] = evaluate_design(...
    Le, freq, V_in, motor, materials)
    
    Lc = Le / materials.aspect_ratio;
    Jw = calculate_wing_inertia(Le, Lc, materials);
    J_total = motor.Jm + Jw;
    KDD = J_total * (2*pi*freq)^2;
    
    dt = 1e-5;
    t_cycles = 6;  % REDUCED from 10 for speed
    T_period = 1/freq;
    time = 0:dt:(t_cycles*T_period);
    
    beta = zeros(size(time));
    beta_dot = zeros(size(time));
    current = zeros(size(time));
    
    for i = 1:length(time)-1
        v = V_in * sin(2*pi*freq*time(i));
        [Cw, ~, ~] = compute_aerodynamics(beta(i), beta_dot(i), Le, Lc, materials);
        di_dt = (v - motor.Ra*current(i) - motor.Kb*beta_dot(i)) / motor.La;
        Tm = motor.Kt * current(i);
        damping = motor.bm*beta_dot(i) + Cw*beta_dot(i)*abs(beta_dot(i));
        spring = KDD*beta(i);
        beta_ddot = (Tm - damping - spring) / J_total;
        current(i+1) = current(i) + di_dt * dt;
        beta_dot(i+1) = beta_dot(i) + beta_ddot * dt;
        beta(i+1) = beta(i) + beta_dot(i) * dt;
    end
    
    idx_cycle1 = find(time >= (t_cycles-2)*T_period & time < (t_cycles-1)*T_period);
    idx_cycle2 = find(time >= (t_cycles-1)*T_period);
    beta_range1 = max(beta(idx_cycle1)) - min(beta(idx_cycle1));
    beta_range2 = max(beta(idx_cycle2)) - min(beta(idx_cycle2));
    converged = abs(beta_range2 - beta_range1) / beta_range1 < 0.05;
    
    idx_start = find(time >= (t_cycles-1)*T_period, 1);
    time_ss = time(idx_start:end);
    beta_ss = beta(idx_start:end);
    beta_dot_ss = beta_dot(idx_start:end);
    current_ss = current(idx_start:end);
    
    beta_max = max(abs(beta_ss)) * 180/pi;
    
    lift_inst = zeros(size(time_ss));
    drag_torque_inst = zeros(size(time_ss));
    
    for i = 1:length(time_ss)
        [~, L, Dt] = compute_aerodynamics(beta_ss(i), beta_dot_ss(i), Le, Lc, materials);
        lift_inst(i) = L;
        drag_torque_inst(i) = Dt;
    end
    
    lift_avg = mean(abs(lift_inst));
    v_ss = V_in * sin(2*pi*freq*time_ss);
    Pe = mean(abs(v_ss .* current_ss));
    Pa = mean(abs(drag_torque_inst .* beta_dot_ss));
    eff = (Pa / Pe) * 100;
end

function Jw = calculate_wing_inertia(Le, Lc, materials)
    Lb = materials.Lb;
    Awb = materials.Awb;
    Awf = materials.Awf;
    t_mb = materials.t_mb;
    rho_wf = materials.rho_wf;
    rho_mb = materials.rho_mb;
    
    n = 50;  % REDUCED from 100 for speed
    r = linspace(Lb/2, Le + Lb/2, n);
    dr = r(2) - r(1);
    weight = zeros(size(r));
    
    for i = 1:n
        c = Lc * sqrt(max(0, 1 - ((r(i) - Lb/2) / Le)^2));
        weight(i) = r(i)^2 * c;
    end
    
    r_bar = sum(r .* weight * dr) / sum(weight * dr);
    
    Jw = (1/3) * Awb * Lb^3 * rho_wf + ...
         (1/3) * Awf * Le * rho_wf * (Le + Lb/2)^2 + ...
         (1/3) * Awf * Lc * rho_wf * (Lb/2)^2 + ...
         (pi/4) * Lc * Le * t_mb * rho_mb * r_bar^2;
end

function [Cw, lift, drag_torque] = compute_aerodynamics(~, beta_dot, Le, Lc, materials)
    Lb = materials.Lb;
    rho = materials.rho_air;
    Calpha = materials.Calpha;
    
    alpha = pi/2 - Calpha * abs(beta_dot);
    alpha = max(alpha, pi/4);
    
    CL = 0.225 + 1.58 * sin(2.13*alpha - 0.1257);
    CD = 1.92 - 1.55 * cos(2.04*alpha - 0.1714);
    
    n = 50;  % REDUCED from 100 for speed
    r = linspace(Lb/2, Le + Lb/2, n);
    dr = r(2) - r(1);
    
    lift_total = 0;
    drag_torque_total = 0;
    Cw_sum = 0;
    
    for i = 1:n
        c = Lc * sqrt(max(0, 1 - ((r(i) - Lb/2) / Le)^2));
        A = c * dr;
        Vr = abs(r(i) * beta_dot);
        L = 0.5 * rho * Vr^2 * A * CL;
        D = 0.5 * rho * Vr * abs(Vr) * A * CD;
        lift_total = lift_total + L;
        drag_torque_total = drag_torque_total + D * r(i);
        Cw_sum = Cw_sum + 0.5 * rho * r(i)^3 * A;
    end
    
    Cw = CD * Cw_sum;
    lift = lift_total;
    drag_torque = drag_torque_total;
end

function s = bool2str(b)
    if b
        s = 'YES';
    else
        s = 'NO';
    end
end