% path_manager_line
%   - follow lines between waypoints.
%
% input is:
%   num_waypoints - number of waypoint configurations
%   waypoints    - an array of dimension 5 by P.size_waypoint_array.
%                - the first num_waypoints rows define waypoint
%                  configurations
%                - format for each waypoint configuration:
%                  [wn, we, wd, dont_care, Va_d]
%                  where the (wn, we, wd) is the NED position of the
%                  waypoint, and Va_d is the desired airspeed along the
%                  path.
%
% output is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d - desired airspeed
%   r    - inertial position of start of waypoint path
%   q    - unit vector that defines inertial direction of waypoint path
%   c    - center of orbit
%   rho  - radius of orbit
%   lambda = direction of orbit (+1 for CW, -1 for CCW)
%
function out = path_manager_line(in,P,start_of_simulation)

  NN = 0;
  num_waypoints = in(1+NN);
  waypoints = reshape(in(2+NN:5*P.size_waypoint_array+1+NN),5,P.size_waypoint_array);
  NN = NN + 1 + 5*P.size_waypoint_array;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  % chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  state     =  in(1+NN:16+NN);
  NN        = NN + 16;
  t         = in(1+NN);
 
  
  p = [pn; pe; -h];

  persistent waypoints_old   % stored copy of old waypoints
  persistent ptr_a           % waypoint pointer
  persistent flag_need_new_waypoints % flag that request new waypoints from path planner
  persistent q_l;
  
  if start_of_simulation || isempty(waypoints_old),
      waypoints_old = zeros(5,P.size_waypoint_array);
      flag_need_new_waypoints = 0;
      q_l = [pn;pe;-h];
  end
  
  % if the waypoints have changed, update the waypoint pointer
  if min(min(waypoints==waypoints_old))==0,
      ptr_a = 2;
      waypoints_old = waypoints;
      flag_need_new_waypoints = 0;
  end
  
 
  % construct output for path follower
  Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
  if ptr_a > 1
      r  = waypoints(1:3,ptr_a-1);
  else
      r  = [pn;pe;-h];
  end
%   q_l    = waypoints(1:3,ptr_a) - 
  q      = waypoints(1:3,ptr_a) - r;
  q      = q/norm(q);
  c      = waypoints(1:3,ptr_a);
  rho    = P.R_min;
  lambda = -1;
  
  n = (q_l + q)/norm(q_l + q);
  if ([pn;pe;-h]-waypoints(1:3,ptr_a))'*n >= 0
      ptr_a = ptr_a + 1;
  end
%   d = waypoints(:,ptr_a);
%   dist = (d(1) - pn)^2 + (d(2)-pe)^2 + (d(3)+h)^2;
%   if ((dist) < 5) && (waypoints(1,ptr_a+1) ~= -9999)
%       ptr_a = ptr_a + 1;
%   end
%   if (waypoints(1,ptr_a+1) ~= -9999)
      flag = 1;
%   else
%       if dist < (P.R_min + 30)^2
%           flag = 2;
%       else
%           flag = 1;
%       end
%   end
  out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints];
%   if waypoints(1, ptr_a+1) ~= -9999 
%     flag = 2;           % following straight line path
%   else
%     flag = 1;                  
%   end
  % determine if next waypoint path has been reached, and rotate ptrs if
  % necessary
  q_l = q;
end