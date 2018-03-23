
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle_hold
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end
 
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
    
  % proportional term
  up = P.airspeed_throttle_kp * error;
  
  % integral term
  ui = P.airspeed_throttle_ki * integrator;
    
  % implement PID control
  delta_t = sat(P.u_trim(4)+up + ui, 1, 0);
  
  % implement integrator anti-windup
  if P.airspeed_throttle_ki~=0,
    delta_t_unsat = P.u_trim(4) + up + ui;
    k_antiwindup = P.Ts/P.airspeed_throttle_ki;
    integrator = integrator + k_antiwindup*(delta_t-delta_t_unsat);
  end
  
  % update persistent variables
  error_d1 = error;

end
