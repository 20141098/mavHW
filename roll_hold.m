%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_hold
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, P)
 
  % compute the current error
  error = phi_c - phi;
  
  % proportional term
  up = P.roll_kp * error;
  
  % derivative term
  ud = -P.roll_kd*p;
  
  % implement PID control
  delta_a = P.roll_kp * error - P.roll_kd*p;
  delta_a = sat(delta_a, 45*pi/180, -45*pi/180);
  
end
