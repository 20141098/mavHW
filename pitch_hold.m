
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_hold
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_hold(theta_c, theta, q, P)
 
  % compute the current error
  error = theta_c - theta;
  
  % proportional term
  up = P.pitch_kp * error;
  
  % derivative term
  ud = -P.pitch_kd * q;
  
  % implement PID control
  delta_e = sat(up + ud, 45*pi/180, -45*pi/180);
  
end
