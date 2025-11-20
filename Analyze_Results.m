%% Analyze_Results.m
% Run after Simulink simulation

%% Extract data from SimulationOutput object
if exist('ans', 'var') && isa(ans, 'Simulink.SimulationOutput')
    out = ans;  % Store the output
elseif exist('out', 'var')
    % Already have output
else
    error('Run the Simulink simulation first!');
end

% Extract time and signals
time = out.tout;
beta = out.beta.signals.values;
beta_dot = out.betadot.signals.values;
current = out.current.signals.values;
lift = out.lift.signals.values;
voltage = out.voltage.signals.values;

%% Convert to proper units
beta_deg = beta * 180/pi;
lift_mN = lift * 1000;
current_mA = current * 1000;

%% Find steady state (last cycle)
T_period = 1/freq;
idx_start = find(time >= time(end) - T_period, 1);

time_ss = time(idx_start:end) - time(idx_start);
beta_ss = beta(idx_start:end);
beta_dot_ss = beta_dot(idx_start:end);
current_ss = current(idx_start:end);
lift_ss = lift(idx_start:end);
voltage_ss = voltage(idx_start:end);

%% Calculate Performance Metrics

% Average Lift
lift_avg = mean(abs(lift_ss));

% Input Power (Eq. 20)
Pe = mean(abs(voltage_ss .* current_ss));

% Aerodynamic Power (Eq. 22) - CORRECTED
% Need to calculate drag TORQUE, not just drag force
drag_torque_ss = zeros(size(time_ss));

for i = 1:length(time_ss)
    % Angle of attack
    alpha_i = pi/2 - Calpha * abs(beta_dot_ss(i));
    alpha_i = max(alpha_i, pi/4);
    
    % Drag coefficient
    CD = 1.92 - 1.55 * cos(2.04*alpha_i - 0.1714);
    
    % Blade element integration for drag TORQUE
    n_elements = 100;
    r = linspace(Lb/2, Le + Lb/2, n_elements);
    dr = r(2) - r(1);
    
    drag_torque_total = 0;
    for j = 1:n_elements
        c_r = Lc * sqrt(max(0, 1 - ((r(j) - Lb/2) / Le)^2));
        Ar = c_r * dr;
        Vr = abs(r(j) * beta_dot_ss(i));
        
        % Drag force at position r
        Dr = 0.5 * rho * Vr * abs(Vr) * Ar * CD;
        
        % Drag TORQUE = drag force × distance from hinge
        drag_torque_total = drag_torque_total + Dr * r(j);
    end
    drag_torque_ss(i) = drag_torque_total;
end

% Aerodynamic power = average of |drag_torque × angular_velocity|
Pa = mean(abs(drag_torque_ss .* beta_dot_ss));

% Efficiency (Eq. 23)
efficiency = (Pa / Pe) * 100;

%% Display Results
fprintf('\n===================================================\n');
fprintf('           SIMULATION RESULTS\n');
fprintf('===================================================\n');
fprintf('Average Lift:        %.2f mN\n', lift_avg*1000);
fprintf('Peak Lift:           %.2f mN\n', max(abs(lift_ss))*1000);
fprintf('Input Power:         %.3f W\n', Pe);
fprintf('Aerodynamic Power:   %.3f W\n', Pa);
fprintf('System Efficiency:   %.2f %%\n', efficiency);
fprintf('Peak Stroke Angle:   %.2f deg\n', max(abs(beta_ss))*180/pi);
fprintf('Peak Current:        %.2f mA\n', max(abs(current_ss))*1000);
fprintf('===================================================\n');

%% Plot Results
figure('Position', [100 100 1400 800]);

% Stroke angle
subplot(3,2,1);
plot(time_ss*1000, beta_ss*180/pi, 'LineWidth', 2);
xlabel('Time [ms]'); ylabel('Angle [deg]');
title('Wing Stroke Angle');
grid on;

% Angular velocity
subplot(3,2,2);
plot(time_ss*1000, beta_dot_ss*180/pi, 'LineWidth', 2);
xlabel('Time [ms]'); ylabel('Velocity [deg/s]');
title('Angular Velocity');
grid on;

% Motor current
subplot(3,2,3);
plot(time_ss*1000, current_ss*1000, 'LineWidth', 2);
xlabel('Time [ms]'); ylabel('Current [mA]');
title('Motor Current');
grid on;

% Lift
subplot(3,2,4);
plot(time_ss*1000, lift_ss*1000, 'LineWidth', 2);
hold on;
yline(lift_avg*1000, 'r--', 'LineWidth', 1.5);
xlabel('Time [ms]'); ylabel('Lift [mN]');
title('Instantaneous Lift');
legend('Instantaneous', 'Average');
grid on;

% Input voltage
subplot(3,2,5);
plot(time_ss*1000, voltage_ss, 'LineWidth', 2);
xlabel('Time [ms]'); ylabel('Voltage [V]');
title('Input Voltage');
grid on;

% Phase portrait
subplot(3,2,6);
plot(beta_ss*180/pi, beta_dot_ss*180/pi, 'LineWidth', 1.5);
xlabel('Angle [deg]'); ylabel('Velocity [deg/s]');
title('Phase Portrait');
grid on;

sgtitle('FWMAV Simulation Results', 'FontSize', 14, 'FontWeight', 'bold');

%% Compare with Paper (Table 4)
fprintf('\n===================================================\n');
fprintf('           COMPARISON WITH PAPER\n');
fprintf('===================================================\n');
fprintf('Parameter              Simulation    Paper (5V)\n');
fprintf('---------------------------------------------------\n');
fprintf('Avg. Lift [mN]:        %.2f          32.35\n', lift_avg*1000);
fprintf('Efficiency [%%]:        %.2f          29.87\n', efficiency);
fprintf('Input Power [W]:       %.3f          0.877\n', Pe);
fprintf('Aero Power [W]:        %.3f          0.262\n', Pa);
fprintf('===================================================\n');