clear; clc;

% 1. Load parameters
FWMAV_Parameters

% 2. Run simulation and store output
out = sim('FWMAV_DirectDrive.slx');

% 3. Analyze
Analyze_Results