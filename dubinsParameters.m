% dubinsParameters
%   - Find Dubin's parameters between two configurations
%
% input is:
%   start_node  - [wn_s, we_s, wd_s, chi_s, 0, 0]
%   end_node    - [wn_e, wn_e, wd_e, chi_e, 0, 0]
%   R           - minimum turn radius
%
% output is:
%   dubinspath  - a matlab structure with the following fields
%       dubinspath.ps   - the start position in re^3
%       dubinspath.chis - the start course angle
%       dubinspath.pe   - the end position in re^3
%       dubinspath.chie - the end course angle
%       dubinspath.R    - turn radius
%       dubinspath.L    - length of the Dubins path
%       dubinspath.cs   - center of the start circle
%       dubinspath.lams - direction of the start circle
%       dubinspath.ce   - center of the end circle
%       dubinspath.lame - direction of the end circle
%       dubinspath.w1   - vector in re^3 defining half plane H1
%       dubinspath.q1   - unit vector in re^3 along straight line path
%       dubinspath.w2   - vector in re^3 defining position of half plane H2
%       dubinspath.w3   - vector in re^3 defining position of half plane H3
%       dubinspath.q3   - unit vector defining direction of half plane H3
% 

function dubinspath = dubinsParameters(start_node, end_node, R)

  ell = norm(start_node(1:2)-end_node(1:2));
  if ell<2*R
      disp('The distance between nodes must be larger than 2R.');
      dubinspath = [];
  else
    
    ps   = start_node(1:3);
    chis = start_node(4);
    pe   = end_node(1:3);
    chie = end_node(4);
    
    Rz_angle = pi/2;
    Rz = [
        cos(Rz_angle),  sin(Rz_angle),  0;
        -sin(Rz_angle), cos(Rz_angle),  0;
        0,              0,              1];
    

    crs = ps' + R*Rz*[cos(chis),sin(chis), 0]';
    cls = ps' + R*Rz*[cos(chis),sin(chis), 0]';
    cre = pe' + R*Rz*[cos(chie), sin(chie), 0]';
    cle = pe' + R*Rz*[cos(chie), sin(chie), 0]';
    
   
    % compute L1
    theta = atan((pe(2)-ps(2))/(pe(1)-ps(1)));
    L1 = norm(crs - cre) + R*(2*pi+(theta-pi/2)-(chis-pi/2))+R*(2*pi+(chie-pi/2)-(theta-pi/2));
    % compute L2
    ell = norm(cle-cre);
    theta = atan((pe(2)-ps(2))/(pe(1)-ps(1)));
    theta2 = theta - pi/2 + asin(2*R/ell);
    if isreal(theta2)==0 
      L2 = 9999; 
    else
      L2 = sqrt(ell^2-4*R^2)+R*(2*pi+theta2-(chis-pi/2))+R*(2*pi+(theta2+pi)-(chie+pi/2));
    end
    % compute L3
    ell = norm(cre-cls);
    theta = atan((pe(2)-ps(2))/(pe(1)-ps(1)));
    theta2 = acos(2*R/ell);
    if isreal(theta2)==0
      L3 = 9999;
    else
      L3 = sqrt(ell^2-4*R^2)+R*(2*pi+(chis+pi/2)-(theta+theta2)) + R*(2*pi+(chie-pi/2)-(theta+theta2-pi));
    end
    % compute L4
    theta = atan((pe(2)-ps(2))/(pe(1)-ps(1)));
    L4 = norm(cls-cle) + R*(2*pi+(chis+pi/2)-(theta+pi/2)) + R*(2*pi + (theta+pi/2) - (chie+pi/2));
    % L is the minimum distance
    [L,idx] = min([L1,L2,L3,L4]);
    e1 = [1; 0; 0];
    switch(idx)
        case 1
            cs = crs;
            lams = 1;
            ce = cre;
            lame = 1;
            q1 = (ce-cs)/norm(ce-cs);
            Rz = [cos(pi/2),sin(pi/2),0;
                -sin(pi/2),cos(pi/2),0;
                0,0,1];
            z1 = cs + R*Rz*q1;
            z2 = ce + R*Rz*q1;
            w1 = z1;
            w2 = z2;
        case 2
            cs = crs;
            lams = 1;
            ce = cle;
            lame = -1;
            ell = norm(ce-cs);
            theta = atan(cle(2)-cls(2),cle(1)-cls(1)); %should be cl?
            theta2 = theta - pi/2 + asin(2*R/ell);
            Rz_angle = (theta2+pi/2);
            Rz = [
                cos(Rz_angle),  sin(Rz_angle),  0;
                -sin(Rz_angle), cos(Rz_angle),  0;
                0,              0,              1];
            q1 = Rz*e1;
            Rz_angle = theta2;
            Rz = [
                cos(Rz_angle),  sin(Rz_angle),  0;
                -sin(Rz_angle), cos(Rz_angle),  0;
                0,              0,              1];
            z1 = cs + R*Rz*theta2*e1;
            Rz_angle = theta2 + pi;
            Rz = [
                cos(Rz_angle),  sin(Rz_angle),  0;
                -sin(Rz_angle), cos(Rz_angle),  0;
                0,              0,              1];
            z2 = ce + R*Rz*e1;
            w1 = z1;
            w2 = z2;
        case 3
            cs = cls;
            lams = -1;
            ce = cre;
            lame = 1;
            ell = norm(ce-cs);
            theta = atan(cle(2)-cls(2),cle(1)-cls(1)); %should be cr?
            theta2 = acos(2*R/ell);
            Rz_angle = theta+theta2-pi/2;
            Rz = [
                cos(Rz_angle),  sin(Rz_angle),  0;
                -sin(Rz_angle), cos(Rz_angle),  0;
                0,              0,              1];
            q1 = Rz*e1;
            Rz_angle = theta+theta2;
            Rz = [
                cos(Rz_angle),  sin(Rz_angle),  0;
                -sin(Rz_angle), cos(Rz_angle),  0;
                0,              0,              1];
            z1 = cs+R*Rz*e1;
            Rz_angle = theta+theta2-pi;
            Rz = [
                cos(Rz_angle),  sin(Rz_angle),  0;
                -sin(Rz_angle), cos(Rz_angle),  0;
                0,              0,              1];
            z2 = ce + R*Rz*e1;
            w1 = z1;
            w2 = z2;
         case 4
            cs = cls;
            lams = -1;
            ce = cle;
            lame = -1;
            q1 = (ce-cs)/norm(ce-cs);
            Rz_angle = pi/2;
            Rz = [
                cos(Rz_angle),  sin(Rz_angle),  0;
                -sin(Rz_angle), cos(Rz_angle),  0;
                0,              0,              1];
            z1 = cs + R*Rz*q1;
            z2 = ce + R*Rz*q1;
            w1 = z1;
            w2 = z2;
    end
    w3 = pe;
    Rz_angle = chie;
    Rz = [
        cos(Rz_angle),  sin(Rz_angle),  0;
        -sin(Rz_angle), cos(Rz_angle),  0;
        0,              0,              1];
    q3 = Rz*e1;
    
    % assign path variables
    dubinspath.ps   = ps;
    dubinspath.chis = chis;
    dubinspath.pe   = pe;
    dubinspath.chie = chie;
    dubinspath.R    = R;
    dubinspath.L    = L;
    dubinspath.cs   = cs;
    dubinspath.lams = lams;
    dubinspath.ce   = ce;
    dubinspath.lame = lame;
    dubinspath.w1   = w1;
    dubinspath.q1   = q1;
    dubinspath.w2   = w2;
    dubinspath.w3   = w3;
    dubinspath.q3   = q3;
    dubinspath.z1   = z1;
    dubinspath.z2   = z2;
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% rotz(theta)
%%   rotation matrix about the z axis.
function R = rotz(theta)
    R = [...
        cos(theta), -sin(theta), 0;...
        sin(theta), cos(theta), 0;...
        0, 0, 1;...
        ];
end
