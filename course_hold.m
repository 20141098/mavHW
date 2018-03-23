
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% course_hold
%   - regulate heading using the roll command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c = course_hold(chi_c, chi, r, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
 
  % compute the current error
  error = chi_c - chi;
  
  % update the integrator
  if abs(error)>15*pi/180,
      integrator = 0;
  else
      integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  end
  
  % proportional term
  up = P.course_kp * error;
  
  % integral term
  ui = P.course_ki * integrator;
  
  % derivative term
  ud = -P.course_kd*r;
  
  
  % implement PID control
  phi_c = sat(up + ui + ud, 45*pi/180, -45*pi/180);
  
  % implement integrator anti-windup
  if P.course_ki~=0,
    phi_c_unsat = up+ui+ud;
    k_antiwindup = P.Ts/P.course_ki;
    integrator = integrator + k_antiwindup*(phi_c-phi_c_unsat);
  end

  % update persistent variables
  error_d1 = error;

end