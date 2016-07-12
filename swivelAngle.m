%swivelAngle Compute the swivel angle based on the orientations of the shoulder,
%humerus and ulna
%
%   swivelAngle(c_s, c_h, c_u, side)
%
%return the swivel angle (in rad) (see ...) based on:
% c_s the 3x3 orientation matrix of the shoulder
% c_h the 3x3 orientation matrix of the humerus (upper-arm)
% c_u the 3x3 orientation matrix of the ulna (forearm)
% side either 'L' or 'R' for respectively left or right arm
%
%   Vincent Crocher - The University of Melbourne - 2016


function [swivel ideal_swivel] = swivelAngle(c_s, c_h, c_u, side)
    x_s = squeeze(c_s(1:3,1,:))';
    y_s = squeeze(c_s(1:3,2,:))';
    z_s = squeeze(c_s(1:3,3,:))';
    y_h = squeeze(c_h(1:3,2,:))';
    y_u = squeeze(c_u(1:3,2,:))';


    %Vector orthogonal to the arm plane (Ps, Pe, Pw)
    if(side=='R')
        u = cross(y_h, y_u);
    else
        u = cross(y_u, y_h);
    end

    %Projection of u on the shoulder vertical plane (Ys, Zs)
    u_p = u-dot(u, x_s)*u;
    u_p = u_p./norm(u_p);

    %Angle is between u_p and z_s
    swivel = atan2(norm(cross(u_p, z_s)), dot(u_p, z_s));

    %Angle is undefined when y_s and y_u are aligned (or close to be)
    if(dot(y_h, y_u)>0.95)
        swivel=nan;
    end

    %Compute "ideal" swivel angle for the current configuration
    
end