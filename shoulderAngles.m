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

function [planeele, elevation, axial] = shoulderAngles(c_o,c_s,side)
% Take the directions of the x and y axes from the transformation matrices
x_o = squeeze(c_o(1:3,1,:))';
y_o = squeeze(c_o(1:3,2,:))';
x_s = squeeze(c_s(1:3,1,:))';
y_s = squeeze(c_s(1:3,2,:))';

% Calculate the number of datapoints and initiate the returned variables
datapoints = length(y_o(:,1));
planeele = zeros(datapoints,1);
elevation = zeros(datapoints,1);
axial = zeros(datapoints,1);

% For each datapoint
for i=1:datapoints
    % Calculate the orientation of an intermediate axis representing the
    % x-axis after the first rotation
    if(side=='R')
        interm_x = cross(y_o(i,:),y_s(i,:)); %For right arm
    else
        interm_x = cross(y_s(i,:),y_o(i,:)); %For left arm
    end
    interm_x = interm_x/norm(interm_x);
    
    % Calculate the first Y1 rotation. If the x-component of the y-axis is
    % positive, the angle is negative, otherwise it is positive
    check = (c_o(:,:,i))\(y_s(i,:)');
    if check(1)> 0
       planeele(i) = acos(dot(x_o(i,:),interm_x));
    else
       planeele(i) = -acos(dot(x_o(i,:),interm_x));
    end
    
    %Planele is undefined when y_s and y_o are aligned (arm along the
    %trunk)
    if(abs(dot(y_s(i,:),y_o(i,:)))>0.9)
        planeele(i)=nan;
    else
        planeele(i)=planeele(i);
    end
    
    
    % Angle is always negative 
    elevation(i) = -acos(dot(y_s(i,:),y_o(i,:)));

    % Axial rotation is the final rotation from the intermediate x-axis to
    % the final x-axis
 %   axial(i) = acos(dot(x_s(i,:),interm_x));
    x_temp = dot(x_s(i,:),interm_x);
    x_off = x_temp*interm_x;
    x_temp2 = (x_s(i,:) - x_off);

    axial(i) = atan2(norm(x_temp2) ,x_temp);

    if(dot(cross(interm_x,x_s(i,:)),y_s) < 0)
        axial(i)  = -axial(i);
    end

	if axial(i) > pi/2
        axial(i) = axial(i) -pi;
    end
end