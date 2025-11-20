%% Create Simulink Model for Direct Drive FWMAV
function create_fwmav_simulink_model()

model_name = 'FWMAV_DirectDrive';

% Close model if it exists
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Create new model
new_system(model_name);
open_system(model_name);

% Add blocks
add_block('simulink/Sources/Sine Wave', [model_name '/Input_Voltage']);
add_block('simulink/Math Operations/Add', [model_name '/Voltage_Sum']);
add_block('simulink/Math Operations/Divide', [model_name '/Current_Dynamics']);
add_block('simulink/Continuous/Integrator', [model_name '/Integrator_Current']);
add_block('simulink/Math Operations/Gain', [model_name '/Motor_Torque']);
add_block('simulink/Math Operations/Product', [model_name '/Damping_Product']);
add_block('simulink/Math Operations/Abs', [model_name '/Abs_BetaDot']);
add_block('simulink/Math Operations/Gain', [model_name '/Spring_Torque']);
add_block('simulink/Math Operations/Sum', [model_name '/Torque_Sum']);
add_block('simulink/Math Operations/Gain', [model_name '/Inertia_Inv']);
add_block('simulink/Continuous/Integrator', [model_name '/Integrator_BetaDot']);
add_block('simulink/Continuous/Integrator', [model_name '/Integrator_Beta']);
add_block('simulink/Sinks/Scope', [model_name '/Scope_Beta']);
add_block('simulink/Sinks/Scope', [model_name '/Scope_Lift']);
add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/Aerodynamics']);

% Set parameters
set_param([model_name '/Input_Voltage'], 'Amplitude', '5', 'Frequency', '2*pi*23.89');

% Position blocks (approximate)
set_param([model_name '/Input_Voltage'], 'Position', [50 50 100 80]);
set_param([model_name '/Scope_Beta'], 'Position', [800 50 850 100]);
set_param([model_name '/Scope_Lift'], 'Position', [800 150 850 200]);

% Connect blocks
add_line(model_name, 'Input_Voltage/1', 'Voltage_Sum/1');
add_line(model_name, 'Voltage_Sum/1', 'Current_Dynamics/1');
add_line(model_name, 'Current_Dynamics/1', 'Integrator_Current/1');
add_line(model_name, 'Integrator_Current/1', 'Motor_Torque/1');

% Save model
save_system(model_name);

fprintf('Simulink model "%s" created successfully!\n', model_name);
fprintf('Please open and configure the model manually for full functionality.\n');

end