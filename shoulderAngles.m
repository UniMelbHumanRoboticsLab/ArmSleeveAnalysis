% Function to calculate the angles of the shoulder rotation
% Where possible this is done in accordance to the recommendataions of the 
% International Society of Biomechanics (ISB), in their paper "ISB 
% Recommendation on definitions of joint coordinate systems of various 
% joints for the reporting of human joint motion -- Part II: shoulder, 
% elbow, wrist and hand" (Journal of Biomechanics, vol 38, 2005, pg 981-992) 
%
% This is calculated in accordance to section 2.4.7 in the article
% Angles are calculated as a Y0-X-Y1 Euler angles.
% Y0 - Plane of Elevation
% X - Elevation
% Y1 - Axial rotation
%
% v1.1 - Justin Fong 15/11/2013
% 20/7/2015 - Changed axial rotation calculation to take into account
% direction
% v1.2 - Vincent Crocher 14/06/2016
% 14/06/2016 - Take arm side as parameter and change angles computation
% accordingly

function [planeele, elevation, axial] = shoulderAngles(c_s, c_h, side)
% Take the directions of the x and y axes from the transformation matrices
x_s = squeeze(c_s(1:3,1,:))';
y_s = squeeze(c_s(1:3,2,:))';
x_h = squeeze(c_h(1:3,1,:))';
y_h = squeeze(c_h(1:3,2,:))';
z_h = squeeze(c_h(1:3,3,:))';

% Calculate the number of datapoints and initiate the returned variables
datapoints = length(y_s(:,1));
planeele = zeros(datapoints,1);
elevation = zeros(datapoints,1);
axial = zeros(datapoints,1);

% For each datapoint
for i=1:datapoints
    % Calculate the orientation of an intermediate axis representing the
    % x-axis after the first rotation
    if(side=='R')
        interm_x = cross(y_s(i,:), y_h(i,:)); %For right arm
    else
        interm_x = cross(y_h(i,:), y_s(i,:)); %For left arm
    end
    interm_x = interm_x/norm(interm_x);


    % Calculate the first Y1 rotation. If the x-component of the y-axis is
    % positive, the angle is negative, otherwise it is positive
    check = (c_s(:,:,i))\(y_h(i,:)');
    if check(1)> 0
       planeele(i) = acos(dot(x_s(i,:), interm_x));
    else
       planeele(i) = -acos(dot(x_s(i,:), interm_x));
    end
    %Planele is undefined when y_h and y_s are aligned (arm along the
    %trunk)
    if(abs(dot(y_h(i,:),y_s(i,:)))>0.90)
        planeele(i)=nan;
        interm_x=-x_s(i,:); %When arm along body (y_s and y_h aligned), inerm_x is directly x_s
    else
        planeele(i)=planeele(i);
        interm_x=-interm_x; %Need the other way around for axial rotation....
    end


    % Elevation angle is always negative 
    elevation(i) = -acos(dot(y_h(i,:), y_s(i,:)));


    % Axial rotation is the final rotation from the intermediate x-axis to
    % the final x-axis
    
    if(side=='R')
        %axial(i) = atan2(norm(cross(x_h(i,:), interm_x)), dot(x_h(i,:), interm_x));
        axial(i) = -atan2( -dot(interm_x, z_h(i,:)), dot(x_h(i,:), interm_x));
    else
        %axial(i) = -atan2(norm(cross(x_h(i,:), -interm_x)), dot(x_h(i,:), -interm_x));
        axial(i) = atan2( -dot(interm_x, z_h(i,:)), dot(x_h(i,:), interm_x));
    end
end