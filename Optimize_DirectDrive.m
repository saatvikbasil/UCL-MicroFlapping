%% Motor_Selection_Complete.m
% Complete FWMAV Optimization Tool - All-in-One Version
% Input your motor specs, get optimal wing design

clear; clc; close all;

fprintf('\n');
fprintf('=============================================\n');
fprintf('  FWMAV Direct Drive Motor Selection Tool\n');
fprintf('=============================================\n\n');

%% ==================== MOTOR PARAMETERS ====================

% Using RS PRO DC Motor, 5.75 W
motor_Ra = 3.035;               % Armature resistance [Ohm]
motor_La = 0.005;                % Armature inductance [H] ESTIMATED
motor_Kt = 0.00893;               % Torque constant [Nm/A] ESTIMATED: τ_stall = K_t × I_stall
motor_Kb = 0.0097;               % Back-EMF constant [V/(rad/s)]  E = V - I_no-load × R_a
motor_Jm = 5e-7;                 % Motor inertia [kg·m²] ESTIMATED
motor_bm = 5e-6;               % Mechanical damping [Ns/rad] ESTIMATED 

V_amplitude = 5;                 % Input voltage [V]

fprintf('Motor Specifications:\n');
fprintf('  Resistance:    %.1f Ω\n', motor_Ra);
fprintf('  Inductance:    %.1f mH\n', motor_La*1000);
fprintf('  Torque const:  %.2f mNm/A\n', motor_Kt*1000);
fprintf('  Inertia:       %.2e kg·m²\n', motor_Jm);
fprintf('  Voltage:       %.1f V\n\n', V_amplitude);

%% ==================== MATERIAL PROPERTIES ====================
rho_air = 1.225;                 % Air density [kg/m³]
Calpha = 5e-3;                   % Passive rotation coefficient

% Wing materials (CFRP frame + Kapton membrane)
rho_wf = 1900;                   % CFRP density [kg/m³]
rho_mb = 1100;                   % Polyimide density [kg/m³]
Awf = 1e-6;                      % Frame cross-section [m²]
t_mb = 25e-6;                    % Membrane thickness [m]
aspect_ratio = 3;                % Le/Lc ratio

Lb = 0.010;                      % Wing base length [m]
wb = 0.005;                      % Wing base width [m]
hb = 0.001;                      % Wing base height [m]
Awb = hb * wb;

%% ==================== OPTIMIZATION SETUP ====================
fprintf('Setting up optimization...\n');

% Search ranges
Le_range = linspace(0.050, 0.080, 15);    % Wing leading edge [m]
freq_range = linspace(18, 30, 15);         % Frequency [Hz]

total_designs = length(Le_range) * length(freq_range);
fprintf('  Testing %d different designs\n', total_designs);
fprintf('  This will take 5-10 minutes...\n\n');

%% ==================== RUN OPTIMIZATION ====================
results = zeros(total_designs, 8);
result_idx = 0;

best_lift = -inf;
best_design = [0, 0, 0, 0, 0, 0, 0];

start_time = tic;

for i = 1:length(Le_range)
    for j = 1:length(freq_range)
        result_idx = result_idx + 1;
        
        Le = Le_range(i);
        freq = freq_range(j);
        
        % Calculate wing inertia
        Lc = Le / aspect_ratio;
        Jw = calculate_wing_inertia(Le, Lc, Lb, Awb, Awf, t_mb, rho_wf, rho_mb);
        J_total = motor_Jm + Jw;
        
        % Calculate resonant spring stiffness
        KDD = J_total * (2*pi*freq)^2;
        
        % Run simulation
        try
            [lift, Pe, Pa, eff] = run_simulation(Le, Lc, freq, KDD, V_amplitude, ...
                motor_Ra, motor_La, motor_Kt, motor_Kb, motor_Jm, motor_bm, ...
                Lb, rho_air, Calpha, aspect_ratio, rho_wf, rho_mb, Awf, t_mb, Jw);
            
            % Store results
            results(result_idx, :) = [Le, freq, KDD, lift, Pe, Pa, eff, Jw];
            
            % Check if best (constraints: Pe<1.5W, eff>15%)
            if lift > best_lift && Pe < 1.5 && eff > 15
                best_lift = lift;
                best_design = [Le, freq, KDD, lift, Pe, Pa, eff];
            end
            
        catch
            % Skip failed designs
            results(result_idx, :) = [Le, freq, KDD, 0, 0, 0, 0, Jw];
        end
        
        % Progress update
        if mod(result_idx, 25) == 0
            elapsed = toc(start_time);
            progress = result_idx / total_designs;
            eta = elapsed / progress - elapsed;
            fprintf('[%3.0f%%] Tested %d/%d designs. ETA: %.1f min\n', ...
                progress*100, result_idx, total_designs, eta/60);
        end
    end
end

elapsed_total = toc(start_time);
fprintf('\n✓ Optimization complete in %.1f minutes!\n\n', elapsed_total/60);

%% ==================== DISPLAY RESULTS ====================
fprintf('=============================================\n');
fprintf('         OPTIMAL DESIGN FOUND\n');
fprintf('=============================================\n');
fprintf('WING DESIGN:\n');
fprintf('  Leading Edge:    %.2f mm\n', best_design(1)*1000);
fprintf('  Root Chord:      %.2f mm\n', (best_design(1)/aspect_ratio)*1000);
fprintf('  Aspect Ratio:    %.1f\n', aspect_ratio);
fprintf('\nOPERATION:\n');
fprintf('  Frequency:       %.2f Hz\n', best_design(2));
fprintf('  Spring Stiffness: %.2f mNm/rad\n', best_design(3)*1000);
fprintf('\nPERFORMANCE:\n');
fprintf('  Average Lift:    %.2f mN\n', best_design(4)*1000);
fprintf('  Input Power:     %.3f W\n', best_design(5));
fprintf('  Aero Power:      %.3f W\n', best_design(6));
fprintf('  Efficiency:      %.2f %%\n', best_design(7));
fprintf('=============================================\n\n');

fprintf('Comparison with Paper (5V):\n');
fprintf('  Parameter         Your Result   Paper\n');
fprintf('  ----------------  -----------   ------\n');
fprintf('  Le [mm]:          %.2f         68.46\n', best_design(1)*1000);
fprintf('  Frequency [Hz]:   %.2f         23.89\n', best_design(2));
fprintf('  KDD [mNm/rad]:    %.2f         8.65\n', best_design(3)*1000);
fprintf('  Lift [mN]:        %.2f         32.35\n', best_design(4)*1000);
fprintf('  Efficiency [%%]:   %.2f         29.87\n', best_design(7));
fprintf('  Power [W]:        %.3f         0.877\n\n', best_design(5));

%% ==================== VISUALIZATION ====================
fprintf('Generating plots...\n');

% Remove failed results
valid_idx = results(:,4) > 0;
results_valid = results(valid_idx, :);

figure('Position', [100 100 1600 1000]);

% 1. Lift surface
subplot(2,3,1);
scatter3(results_valid(:,1)*1000, results_valid(:,2), results_valid(:,4)*1000, ...
    50, results_valid(:,4)*1000, 'filled');
xlabel('Wing Length [mm]'); ylabel('Freq [Hz]'); zlabel('Lift [mN]');
title('Lift vs Design Parameters');
colorbar;
hold on;
plot3(best_design(1)*1000, best_design(2), best_design(4)*1000, ...
    'r*', 'MarkerSize', 25, 'LineWidth', 3);
grid on; view(45, 30);

% 2. Efficiency surface
subplot(2,3,2);
scatter3(results_valid(:,1)*1000, results_valid(:,2), results_valid(:,7), ...
    50, results_valid(:,7), 'filled');
xlabel('Wing Length [mm]'); ylabel('Freq [Hz]'); zlabel('Efficiency [%]');
title('Efficiency vs Design Parameters');
colorbar;
hold on;
plot3(best_design(1)*1000, best_design(2), best_design(7), ...
    'r*', 'MarkerSize', 25, 'LineWidth', 3);
grid on; view(45, 30);

% 3. Power surface
subplot(2,3,3);
scatter3(results_valid(:,1)*1000, results_valid(:,2), results_valid(:,5), ...
    50, results_valid(:,5), 'filled');
xlabel('Wing Length [mm]'); ylabel('Freq [Hz]'); zlabel('Power [W]');
title('Input Power vs Design Parameters');
colorbar;
hold on;
plot3(best_design(1)*1000, best_design(2), best_design(5), ...
    'r*', 'MarkerSize', 25, 'LineWidth', 3);
grid on; view(45, 30);

% 4. Lift vs Efficiency
subplot(2,3,4);
scatter(results_valid(:,4)*1000, results_valid(:,7), 60, results_valid(:,5), 'filled');
xlabel('Lift [mN]'); ylabel('Efficiency [%]');
title('Performance Trade-off');
colorbar; ylabel(colorbar, 'Power [W]');
hold on;
plot(best_design(4)*1000, best_design(7), 'r*', 'MarkerSize', 25, 'LineWidth', 3);
grid on;

% 5. Frequency sweep at optimal Le
subplot(2,3,5);
idx_opt = abs(results_valid(:,1) - best_design(1)) < 0.003;
plot(results_valid(idx_opt, 2), results_valid(idx_opt, 4)*1000, 'o-', 'LineWidth', 2);
xlabel('Frequency [Hz]'); ylabel('Lift [mN]');
title(sprintf('Lift vs Freq (Le=%.1fmm)', best_design(1)*1000));
hold on;
plot(best_design(2), best_design(4)*1000, 'r*', 'MarkerSize', 25, 'LineWidth', 3);
grid on;

% 6. Wing size sweep at optimal freq
subplot(2,3,6);
idx_opt = abs(results_valid(:,2) - best_design(2)) < 0.6;
plot(results_valid(idx_opt, 1)*1000, results_valid(idx_opt, 4)*1000, 'o-', 'LineWidth', 2);
xlabel('Wing Length [mm]'); ylabel('Lift [mN]');
title(sprintf('Lift vs Wing Size (f=%.1fHz)', best_design(2)));
hold on;
plot(best_design(1)*1000, best_design(4)*1000, 'r*', 'MarkerSize', 25, 'LineWidth', 3);
grid on;

sgtitle('FWMAV Motor Selection - Optimization Results', 'FontSize', 16, 'FontWeight', 'bold');

fprintf('✓ Plots generated!\n\n');

%% REFINED SEARCH AROUND PAPER'S VALUES
fprintf('\n===========================================\n');
fprintf('   REFINING SEARCH (High Resolution)\n');
fprintf('===========================================\n');

% Focus on the region near paper's optimum
Le_refined = linspace(0.065, 0.072, 20);      % 65-72 mm (finer)
freq_refined = linspace(22, 26, 20);           % 22-26 Hz (finer)

best_lift_refined = -inf;
best_design_refined = [0, 0, 0, 0, 0, 0, 0];

fprintf('Testing %d additional designs in refined region...\n', length(Le_refined)*length(freq_refined));

for i = 1:length(Le_refined)
    for j = 1:length(freq_refined)
        Le = Le_refined(i);
        freq = freq_refined(j);
        
        Lc = Le / aspect_ratio;
        Jw = calculate_wing_inertia(Le, Lc, Lb, Awb, Awf, t_mb, rho_wf, rho_mb);
        J_total = motor_Jm + Jw;
        KDD = J_total * (2*pi*freq)^2;
        
        try
            [lift, Pe, Pa, eff] = run_simulation(Le, Lc, freq, KDD, V_amplitude, ...
                motor_Ra, motor_La, motor_Kt, motor_Kb, motor_Jm, motor_bm, ...
                Lb, rho_air, Calpha, aspect_ratio, rho_wf, rho_mb, Awf, t_mb, Jw);
            
            if lift > best_lift_refined && Pe < 1.5 && eff > 15
                best_lift_refined = lift;
                best_design_refined = [Le, freq, KDD, lift, Pe, Pa, eff];
            end
        catch
            continue;
        end
    end
end

fprintf('\n===========================================\n');
fprintf('   REFINED OPTIMAL DESIGN\n');
fprintf('===========================================\n');
fprintf('WING DESIGN:\n');
fprintf('  Leading Edge:    %.2f mm\n', best_design_refined(1)*1000);
fprintf('  Root Chord:      %.2f mm\n', (best_design_refined(1)/aspect_ratio)*1000);
fprintf('\nOPERATION:\n');
fprintf('  Frequency:       %.2f Hz\n', best_design_refined(2));
fprintf('  Spring Stiffness: %.2f mNm/rad\n', best_design_refined(3)*1000);
fprintf('\nPERFORMANCE:\n');
fprintf('  Average Lift:    %.2f mN\n', best_design_refined(4)*1000);
fprintf('  Input Power:     %.3f W\n', best_design_refined(5));
fprintf('  Aero Power:      %.3f W\n', best_design_refined(6));
fprintf('  Efficiency:      %.2f %%\n', best_design_refined(7));
fprintf('===========================================\n\n');

fprintf('Comparison:\n');
fprintf('                Initial    Refined     Paper\n');
fprintf('  Le [mm]:      %.2f      %.2f       68.46\n', best_design(1)*1000, best_design_refined(1)*1000);
fprintf('  Freq [Hz]:    %.2f      %.2f       23.89\n', best_design(2), best_design_refined(2));
fprintf('  Lift [mN]:    %.2f      %.2f       32.35\n', best_design(4)*1000, best_design_refined(4)*1000);
fprintf('  Eff [%%]:      %.2f      %.2f       29.87\n\n', best_design(7), best_design_refined(7));

%% ==================== SAVE RESULTS ====================
save('motor_selection_results.mat', 'results_valid', 'best_design', ...
    'motor_Ra', 'motor_La', 'motor_Kt', 'motor_Kb', 'motor_Jm', 'motor_bm', 'V_amplitude');
fprintf('✓ Results saved to: motor_selection_results.mat\n\n');

fprintf('=============================================\n');
fprintf('  Motor Selection Tool Complete!\n');
fprintf('=============================================\n');

%% ==================== HELPER FUNCTIONS ====================

function Jw = calculate_wing_inertia(Le, Lc, Lb, Awb, Awf, t_mb, rho_wf, rho_mb)
    % Calculate wing moment of inertia
    
    % Aerodynamic center (weighted average)
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

function [lift_avg, Pe, Pa, eff] = run_simulation(Le, Lc, freq, KDD, V_in, ...
    Ra, La, Kt, Kb, Jm, bm, Lb, rho, Calpha, AR, rho_wf, rho_mb, Awf, t_mb, Jw)
    
    % Simulation setup
    J_total = Jm + Jw;
    dt = 2e-5;
    T_period = 1/freq;
    time = 0:dt:(3*T_period);
    
    % Initialize
    beta = zeros(size(time));
    beta_dot = zeros(size(time));
    current = zeros(size(time));
    
    % Simulation loop
    for i = 1:length(time)-1
        v = V_in * sin(2*pi*freq*time(i));
        
        % Aerodynamics
        [Cw, ~, ~] = get_aero(beta(i), beta_dot(i), Le, Lc, Lb, rho, Calpha);
        
        % Dynamics
        di_dt = (v - Ra*current(i) - Kb*beta_dot(i)) / La;
        Tm = Kt * current(i);
        damping = bm*beta_dot(i) + Cw*beta_dot(i)*abs(beta_dot(i));
        spring = KDD*beta(i);
        beta_ddot = (Tm - damping - spring) / J_total;
        
        % Integrate
        current(i+1) = current(i) + di_dt * dt;
        beta_dot(i+1) = beta_dot(i) + beta_ddot * dt;
        beta(i+1) = beta(i) + beta_dot(i) * dt;
    end
    
    % Analyze last cycle
    idx = time >= 2*T_period;
    beta_ss = beta(idx);
    beta_dot_ss = beta_dot(idx);
    current_ss = current(idx);
    time_ss = time(idx);
    
    % Calculate performance
    lift_inst = zeros(size(time_ss));
    drag_torque_inst = zeros(size(time_ss));
    
    for i = 1:length(time_ss)
        [~, L, Dt] = get_aero(beta_ss(i), beta_dot_ss(i), Le, Lc, Lb, rho, Calpha);
        lift_inst(i) = L;
        drag_torque_inst(i) = Dt;
    end
    
    lift_avg = mean(abs(lift_inst));
    v_ss = V_in * sin(2*pi*freq*time_ss);
    Pe = mean(abs(v_ss .* current_ss));
    Pa = mean(abs(drag_torque_inst .* beta_dot_ss));
    eff = (Pa / Pe) * 100;
end

function [Cw, lift, drag_torque] = get_aero(beta, beta_dot, Le, Lc, Lb, rho, Calpha)
    % Aerodynamic forces
    
    alpha = pi/2 - Calpha * abs(beta_dot);
    alpha = max(alpha, pi/4);
    
    CL = 0.225 + 1.58 * sin(2.13*alpha - 0.1257);
    CD = 1.92 - 1.55 * cos(2.04*alpha - 0.1714);
    
    n = 20;
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
