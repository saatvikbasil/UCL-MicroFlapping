%% Motor_Selection_Improved.m
% Improved FWMAV Optimization with better convergence
% Based on Park & Abolfathi (2024) Direct Drive Model

clear; clc; close all;
rng(0); % Fix random seed for reproducibility

fprintf('\n');
fprintf('=============================================\n');
fprintf('  FWMAV Motor Selection - Improved Version\n');
fprintf('=============================================\n\n');

%% Check Optimization Toolbox
if ~license('test', 'Optimization_Toolbox')
    error('Optimization Toolbox not found!');
end

%% ==================== MOTOR PARAMETERS ====================
motor.Ra = 9;               % Armature resistance [Ohm]
motor.La = 0.029;           % Armature inductance [H]
motor.Kt = 0.01;            % Torque constant [Nm/A]
motor.Kb = 0.01;            % Back-EMF constant [V/(rad/s)]
motor.Jm = 1.5e-7;          % Motor inertia [kg·m²]
motor.bm = 3.6e-6;          % Mechanical damping [Ns/rad]

V_amplitude = 3;            % Input voltage [V]

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
fprintf('Setting up optimization...\n\n');

% Design variables: [Le, freq]
% Paper results at 5V: Le=68.46mm, freq=23.89Hz, KDD=8.65 mNm/rad

% Try multiple initial guesses to avoid local minima
initial_guesses = [
    0.065, 28;   % Close to paper result
    0.060, 30;   % Alternative
    0.055, 35;   % Lower frequency
];

lb = [0.030, 17];                % Lower bounds [m, Hz]
ub = [0.100, 40];                % Upper bounds [m, Hz]

% Optimization options - improved for stability
options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'sqp', ...
    'MaxIterations', 300, ...
    'MaxFunctionEvaluations', 2000, ...
    'OptimalityTolerance', 1e-4, ...
    'StepTolerance', 1e-6, ...
    'ConstraintTolerance', 1e-5, ...
    'FiniteDifferenceStepSize', 1e-4, ...    % Increased from 1e-6
    'FiniteDifferenceType', 'central', ...   % More accurate
    'UseParallel', false, ...
    'ScaleProblem', true);                   % Auto-scale variables

% Store results from all initial guesses
results = struct('x_opt', {}, 'fval', {}, 'exitflag', {}, 'output', {});

fprintf('Testing %d initial guesses...\n', size(initial_guesses, 1));

for i = 1:size(initial_guesses, 1)
    x0 = initial_guesses(i, :);
    
    fprintf('\n--- Initial Guess #%d: Le=%.1fmm, freq=%.1fHz ---\n', ...
        i, x0(1)*1000, x0(2));
    
    % Objective and constraints
    objective = @(x) objective_function(x, V_amplitude, motor, materials);
    nonlcon = @(x) constraint_function(x, V_amplitude, motor, materials, P_max);
    
    % Run optimization
    tic;
    [x_opt, fval, exitflag, output] = fmincon(objective, x0, [], [], [], [], ...
                                              lb, ub, nonlcon, options);
    elapsed = toc;
    
    % Calculate KDD for this result
    Le_i = x_opt(1);
    freq_i = x_opt(2);
    Lc_i = Le_i / materials.aspect_ratio;
    Jw_i = calculate_wing_inertia(Le_i, Lc_i, materials);
    J_total_i = motor.Jm + Jw_i;
    KDD_i = J_total_i * (2*pi*freq_i)^2;
    
    % Store results
    results(i).x_opt = x_opt;
    results(i).fval = fval;
    results(i).exitflag = exitflag;
    results(i).output = output;
    results(i).elapsed = elapsed;
    results(i).x0 = x0;
    results(i).KDD = KDD_i;  % Store KDD too
    
    fprintf('Result: Le=%.2fmm, freq=%.2fHz, lift=%.2fmN, KDD=%.2f mNm/rad (%.1fs)\n', ...
        x_opt(1)*1000, x_opt(2), -fval*1000, KDD_i*1000, elapsed);
end

%% ==================== SELECT BEST RESULT ====================
fprintf('\n=============================================\n');
fprintf('         COMPARING ALL RESULTS\n');
fprintf('=============================================\n');

best_idx = 1;
best_lift = -results(1).fval;

for i = 1:length(results)
    lift_i = -results(i).fval;
    Le_i = results(i).x_opt(1);
    freq_i = results(i).x_opt(2);
    
    fprintf('#%d: Le=%.2fmm, freq=%.2fHz, lift=%.2fmN, flag=%d\n', ...
        i, Le_i*1000, freq_i, lift_i*1000, results(i).exitflag);
    
    if lift_i > best_lift && results(i).exitflag > 0
        best_lift = lift_i;
        best_idx = i;
    end
end

fprintf('\n✓ Best result: Initial guess #%d\n', best_idx);

% Use best result
x_opt = results(best_idx).x_opt;
output = results(best_idx).output;
elapsed_time = results(best_idx).elapsed;

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

%% ==================== DISPLAY RESULTS ====================
fprintf('\n=============================================\n');
fprintf('         OPTIMAL DESIGN (Best Result)\n');
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
fprintf('\nCOMPARISON TO PAPER (5V, Direct Drive):\n');
fprintf('  Paper Le:        68.46 mm (yours: %.2f mm)\n', Le_opt*1000);
fprintf('  Paper freq:      23.89 Hz (yours: %.2f Hz)\n', freq_opt);
fprintf('  Paper KDD:       8.65 mNm/rad (yours: %.2f mNm/rad)\n', KDD_opt*1000);
fprintf('  Paper lift:      32.35 mN (yours: %.2f mN)\n', lift_opt*1000);
fprintf('  Paper eff:       29.87 %% (yours: %.2f %%)\n', eff_opt);
fprintf('=============================================\n\n');

%% ==================== VERIFICATION ====================
fprintf('Running verification checks...\n');

% Test at paper's reported values for comparison
Le_paper = 0.06846;
freq_paper = 23.89;
[lift_paper, Pe_paper, Pa_paper, eff_paper, ~, ~] = evaluate_design(...
    Le_paper, freq_paper, V_amplitude, motor, materials);

fprintf('Paper design (Le=68.46mm, freq=23.89Hz):\n');
fprintf('  Lift: %.2f mN, Eff: %.2f %%\n', lift_paper*1000, eff_paper);
fprintf('  Power: %.3f W, Aero: %.3f W\n\n', Pe_paper, Pa_paper);

%% ==================== SAVE RESULTS ====================
save('motor_selection_improved.mat', 'x_opt', 'Le_opt', 'freq_opt', 'KDD_opt', ...
    'lift_opt', 'Pe_opt', 'Pa_opt', 'eff_opt', 'motor', 'materials', ...
    'results', 'best_idx');

fprintf('✓ Results saved!\n\n');

%% ==================== HELPER FUNCTIONS ====================

function cost = objective_function(x, V_amp, motor, materials)
    % Objective: Maximize lift (minimize negative lift)
    Le = x(1);
    freq = x(2);
    
    try
        [lift, ~, ~, ~, ~, converged] = evaluate_design(Le, freq, V_amp, motor, materials);
        
        if ~converged
            % Penalty for non-converged solutions
            cost = 1e6;
        else
            % Minimize negative lift
            cost = -lift;
        end
        
    catch ME
        % If simulation fails, return large penalty
        warning('Simulation failed: %s', ME.message);
        cost = 1e6;
    end
end

function [c, ceq] = constraint_function(x, V_amp, motor, materials, P_max)
    % Constraints
    Le = x(1);
    freq = x(2);
    
    try
        [~, Pe, ~, eff, ~, ~] = evaluate_design(Le, freq, V_amp, motor, materials);
        
        % Inequality constraints: c(x) <= 0
        c = [
            Pe - P_max;         % Power must be < P_max
            12 - eff;           % Efficiency must be > 12% (relaxed from 15%)
            eff - 50;           % Efficiency must be < 50% (relaxed from 45%)
        ];
        
        ceq = [];  % No equality constraints
        
    catch ME
        % If simulation fails, violate constraints heavily
        warning('Constraint evaluation failed: %s', ME.message);
        c = [10; 10; 10];
        ceq = [];
    end
end

function [lift_avg, Pe, Pa, eff, beta_max, converged] = evaluate_design(...
    Le, freq, V_in, motor, materials)
    % Full simulation with convergence check
    
    % Calculate wing properties
    Lc = Le / materials.aspect_ratio;
    Jw = calculate_wing_inertia(Le, Lc, materials);
    J_total = motor.Jm + Jw;
    
    % Spring stiffness for resonance
    KDD = J_total * (2*pi*freq)^2;
    
    % Simulation parameters - increased for better convergence
    dt = 1e-5;                      % Time step [s]
    t_cycles = 10;                   % More cycles for convergence
    T_period = 1/freq;
    time = 0:dt:(t_cycles*T_period);
    
    % Initialize state variables
    beta = zeros(size(time));
    beta_dot = zeros(size(time));
    current = zeros(size(time));
    
    % Simulation loop
    for i = 1:length(time)-1
        % Input voltage
        v = V_in * sin(2*pi*freq*time(i));
        
        % Aerodynamic forces
        [Cw, ~, ~] = compute_aerodynamics(beta(i), beta_dot(i), Le, Lc, materials);
        
        % Electrical dynamics
        di_dt = (v - motor.Ra*current(i) - motor.Kb*beta_dot(i)) / motor.La;
        
        % Mechanical dynamics
        Tm = motor.Kt * current(i);
        damping = motor.bm*beta_dot(i) + Cw*beta_dot(i)*abs(beta_dot(i));
        spring = KDD*beta(i);
        beta_ddot = (Tm - damping - spring) / J_total;
        
        % Integration (Euler)
        current(i+1) = current(i) + di_dt * dt;
        beta_dot(i+1) = beta_dot(i) + beta_ddot * dt;
        beta(i+1) = beta(i) + beta_dot(i) * dt;
    end
    
    % Check convergence by comparing last two cycles
    idx_cycle1 = find(time >= (t_cycles-2)*T_period & time < (t_cycles-1)*T_period);
    idx_cycle2 = find(time >= (t_cycles-1)*T_period);
    
    beta_range1 = max(beta(idx_cycle1)) - min(beta(idx_cycle1));
    beta_range2 = max(beta(idx_cycle2)) - min(beta(idx_cycle2));
    
    % Converged if change is < 5%
    converged = abs(beta_range2 - beta_range1) / beta_range1 < 0.05;
    
    % Analyze last cycle
    idx_start = find(time >= (t_cycles-1)*T_period, 1);
    time_ss = time(idx_start:end);
    beta_ss = beta(idx_start:end);
    beta_dot_ss = beta_dot(idx_start:end);
    current_ss = current(idx_start:end);
    
    % Maximum stroke angle
    beta_max = max(abs(beta_ss)) * 180/pi;
    
    % Calculate performance metrics
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
    % Calculate wing moment of inertia (Equation 12)
    
    Lb = materials.Lb;
    Awb = materials.Awb;
    Awf = materials.Awf;
    t_mb = materials.t_mb;
    rho_wf = materials.rho_wf;
    rho_mb = materials.rho_mb;
    
    % Accurate aerodynamic center
    n = 50;
    r = linspace(Lb/2, Le + Lb/2, n);
    dr = r(2) - r(1);
    weight = zeros(size(r));
    
    for i = 1:n
        c = Lc * sqrt(max(0, 1 - ((r(i) - Lb/2) / Le)^2));
        weight(i) = r(i)^2 * c;
    end
    
    r_bar = sum(r .* weight * dr) / sum(weight * dr);
        
    % Equation 12
    Jw = (1/3) * Awb * Lb^3 * rho_wf + ...
         (1/3) * Awf * Le * rho_wf * (Le + Lb/2)^2 + ...
         (1/3) * Awf * Lc * rho_wf * (Lb/2)^2 + ...
         (pi/4) * Lc * Le * t_mb * rho_mb * r_bar^2;
end

function [Cw, lift, drag_torque] = compute_aerodynamics(~, beta_dot, Le, Lc, materials)
    % Compute aerodynamic forces and damping
    
    Lb = materials.Lb;
    rho = materials.rho_air;
    Calpha = materials.Calpha;
    
    % Angle of attack (Eq. 10)
    alpha = pi/2 - Calpha * abs(beta_dot);
    alpha = max(alpha, pi/4);
    
    % Lift and drag coefficients (Eqs. 8, 9)
    CL = 0.225 + 1.58 * sin(2.13*alpha - 0.1257);
    CD = 1.92 - 1.55 * cos(2.04*alpha - 0.1714);
    
    % Blade element integration
    n = 100;
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