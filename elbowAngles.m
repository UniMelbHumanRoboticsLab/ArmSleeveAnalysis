%elbowAngles Compute the flexion and pronation/supination angles based on
%the orientations of the humerus and ulna
%
%   [flex, prono] = elbowAngles(c_h, c_u, side)
%
%return the flexion and pronosupination angles (in rad) based on:
% c_h the 3x3 orientation matrix of the humerus (upper-arm)
% c_u the 3x3 orientation matrix of the ulna (forearm)
% side either 'L' or 'R' for respectively left or right arm
%
%   Vincent Crocher - The University of Melbourne - 2016


function [flex, prono] = elbowAngles(c_h, c_u, side)
    x_h = c_h(1:3,1);
    y_h = c_h(1:3,2);
    z_h = c_h(1:3,3);
    x_u = c_u(1:3,1);
    y_u = c_u(1:3,2);
    z_u = c_u(1:3,3);

    %Flexion between y_h and y_u
    flex=acos(dot(y_h, y_u));

    %x_h rotated around z_h by flex
    q = [flex; z_h]';
    x_r = quatrotate(q, x_h');

    %Angle between x_r and z_u
    if(side=='R')
        prono=atan2(norm(cross(x_r, z_u)), dot(x_r, z_u))-pi;
    else
        prono=-atan2(norm(cross(x_r, z_u)), dot(x_r, z_u));
    end
end