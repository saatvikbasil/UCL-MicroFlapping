% FWMAV_Run  Single simulation: setup -> simulate -> analyse.

FWMAV_Setup ;
out = sim('FWMAV_DirectDrive.slx');
FWMAV_Analyze;
