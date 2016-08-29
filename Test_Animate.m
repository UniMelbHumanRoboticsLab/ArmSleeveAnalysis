clear all
%close all

Sensors = {'SI-2500','SI-2532','SI-2545','SI-2555','SI-2561','SI-2575'};
SensorLabels = {'L Shoulder','L Elbow','R Shoulder','R Elbow','R Wrist','L Wrist'};

%W,E,S
fn = 'Data/20160819-P5.h5'
rsdata = h5read(fn,'/SI-002545/Calibrated/Orientation')';
redata = h5read(fn,'/SI-002555/Calibrated/Orientation')';
rwdata = h5read(fn,'/SI-002561/Calibrated/Orientation')';

lsdata = h5read(fn,'/SI-002500/Calibrated/Orientation')';
ledata = h5read(fn,'/SI-002532/Calibrated/Orientation')';
lwdata = h5read(fn,'/SI-002575/Calibrated/Orientation')';

rstime = h5read(fn,'/SI-002545/Time');
retime = h5read(fn,'/SI-002555/Time');
rwtime = h5read(fn,'/SI-002561/Time');

lstime = h5read(fn,'/SI-002500/Time');
letime = h5read(fn,'/SI-002532/Time');
lwtime = h5read(fn,'/SI-002575/Time');
%% Identify time of interest

startTime = max([rstime(1) retime(1) rwtime(1) lstime(1) letime(1) lwtime(1)]);
endTime = min([rstime(length(rstime)) retime(length(retime)) rwtime(length(rwtime)) lstime(length(lstime)) letime(length(letime)) lwtime(length(lwtime))]);

startTimeInd = [find(rstime==startTime), find(retime==startTime), find(rwtime==startTime), find(lstime==startTime), find(letime==startTime), find(lwtime==startTime)];
endTimeInd = [find(rstime==endTime), find(retime==endTime), find(rwtime==endTime), find(lstime==endTime), find(letime==endTime), find(lwtime==endTime)];

%%


rsdata = rsdata(startTimeInd(1):endTimeInd(1),:);
redata = redata(startTimeInd(2):endTimeInd(2),:);
rwdata = rwdata(startTimeInd(3):endTimeInd(3),:);

lsdata = lsdata(startTimeInd(4):endTimeInd(4),:);
ledata = ledata(startTimeInd(5):endTimeInd(5),:);
lwdata = lwdata(startTimeInd(6):endTimeInd(6),:);

time = retime(startTimeInd(1):endTimeInd(1),:);
time = (double(time) - double(time(1))*ones(size(time)))/1e6;


% data = zeros(length(rsdata),13);
% data(:,1) = (double(time) - double(time(1))*ones(size(time)))/1e6;
% data(:,2:5) = rwdata';
% data(:,6:9) = redata';
% data(:,10:13) = rsdata';

%%

anim = 1;
count = 10;

% Setup for Graphic
% Assumed limb lengths
% Length Humerus: l_h
% Length Ulna: l_u
l_h = 0.34;
l_u = 0.34;
l_s = 0.21;

if anim
    % Setup for plots
    try
        close(5)
    end
    figure(5)
    
    hold on
    h_home = plot3([0 0.2; 0 0; 0 0]',[0 0; 0 0.2; 0 0]',[0 0; 0 0; 0 0.2]');
    h7 = plot3([0 0.2; 0 0; 0 0]',[0 0; 0 0.2; 0 0]',[0 0; 0 0; 0 0.2]');
    h4 = plot3([0 0.2; 0 0; 0 0]',[0 0; 0 0.2; 0 0]',[0 0; 0 0; 0 0.2]');
    h5 = plot3([-0.4 -0.4; -0.4 -0.2; -0.4 -0.4]',[0 0.2; 0 0; 0 0]',[0 0; 0 0; 0 0.2]');
    h6 = plot3([0 0 0.4 0.8]',[0 0 0 0]',[0 0 0 0]','linewidth',10);
    hleft = plot3([0 0 0.4 0.8]',[0 0 0 0]',[0 0 0 0]','linewidth',10);
    hold off
    axis([-1 1 -1 1 -1 1])
    legend('x','y','z')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    
    view([1 0 -0.1])
end

ssTs = [0 1 0 0;
    0 0 1 0;
    1 0 0 0;
    0 0 0 1];
hsTh = [0 0 -1 0;
    -1 0 0 0;
    0 1 0 0 ;
    0 0 0 1];

angleData = zeros(length(time), 5);
angleDataM = zeros(length(time), 5);

framerate =30;
lastframetime = 0;
dt = 1/framerate;
j = 1;

for i = 1:1:length(time)
    
        if (mod(i,count) == 0)
            time(i)/60

            %q_s = (data(i,10:13));
            %q_h = (data(i,6:9));
            %q_u = (data(i,2:5));

            q_rh = quatinv(redata(i,:)/quatnorm(redata(i,:)));
            q_ru = quatinv(rwdata(i,:)/quatnorm(rwdata(i,:)));
            q_rs = quatinv(rsdata(i,:)/quatnorm(rsdata(i,:)));

            c_rss = quatrotate(q_rs,eye(3))';
            c_rhs = quatrotate(q_rh,eye(3))';
            c_rus = quatrotate(q_ru,eye(3))';

            q_lh = quatinv(ledata(i,:)/quatnorm(ledata(i,:)));
            q_lu = quatinv(lwdata(i,:)/quatnorm(lwdata(i,:)));
            q_ls = quatinv(lsdata(i,:)/quatnorm(lsdata(i,:)));

            c_lss = quatrotate(q_ls,eye(3))';
            c_lhs = quatrotate(q_lh,eye(3))';
            c_lus = quatrotate(q_lu,eye(3))';

            %T_s = [c_ss, [0 0 0]'; 0 0 0 1]*ssTs;
            %T_h = [c_hs, [0 0 0]'; 0 0 0 1]*hsTh;
            %T_u = [c_us, [0 0 0]'; 0 0 0 1]*usTu;

            %c_s = T_s(1:3,1:3);
            %c_h = T_h(1:3,1:3);
            %c_u = T_u(1:3,1:3);

            c_rs = [-c_rss(:,2) [-c_rss(1:2,3); c_rss(3,3)] [-c_rss(1:2,1); c_rss(3,1)] ];
            c_rh = [-c_rhs(:,3) -c_rhs(:,1) c_rhs(:,2)];
            c_ru = [-c_rus(:,3) -c_rus(:,1) c_rus(:,2)];

            T_rs = [c_rs [0 0 0]'; 0 0 0 1];
            T_rh = [c_rh [0 0 0]'; 0 0 0 1];
            T_ru = [c_ru [0 0 0]'; 0 0 0 1];

            c_ls = [c_lss(:,2) c_lss(:,3) c_lss(:,1)];
            c_lh = [-c_lhs(:,3) -c_lhs(:,1) c_lhs(:,2)];
            c_lu = [-c_lus(:,3) -c_lus(:,1) c_lus(:,2)];

            T_ls = [c_ls [0 0 0]'; 0 0 0 1];
            T_lh = [c_lh [0 0 0]'; 0 0 0 1];
            T_lu = [c_lu [0 0 0]'; 0 0 0 1];

            p_rs = c_rs(:,3)*l_s;
            p_re =  p_rs - c_rh(:,2)*l_h;
            p_rhd = p_re - c_ru(:,2)*l_u;

            p_ls = c_ls(:,3)*l_s;
            p_le =  p_ls - c_lh(:,2)*l_h;
            p_lhd = p_le - c_lu(:,2)*l_u;


            % Modify to draw frames
            c_rh = c_rh*0.2;
            c_ru = c_ru*0.2;
            c_rs = c_rs*0.2;

            if (anim )
                % Draw Frames on Animation
                set(h7(1),'XData',[p_rs(1) p_rs(1) + c_rs(1,1)],'YData',[p_rs(2) p_rs(2) + c_rs(2,1)],'ZData',[p_rs(3) p_rs(3) + c_rs(3,1)]);
                set(h7(2),'XData',[p_rs(1) p_rs(1) + c_rs(1,2)],'YData',[p_rs(2) p_rs(2) + c_rs(2,2)],'ZData',[p_rs(3) p_rs(3) + c_rs(3,2)]);
                set(h7(3),'XData',[p_rs(1) p_rs(1) + c_rs(1,3)],'YData',[p_rs(2) p_rs(2) + c_rs(2,3)],'ZData',[p_rs(3) p_rs(3) + c_rs(3,3)]);

                set(h4(1),'XData',[p_re(1) p_re(1) + c_rh(1,1)],'YData',[p_re(2) p_re(2) + c_rh(2,1)],'ZData',[p_re(3) p_re(3) + c_rh(3,1)]);
                set(h4(2),'XData',[p_re(1) p_re(1) + c_rh(1,2)],'YData',[p_re(2) p_re(2) + c_rh(2,2)],'ZData',[p_re(3) p_re(3) + c_rh(3,2)]);
                set(h4(3),'XData',[p_re(1) p_re(1) + c_rh(1,3)],'YData',[p_re(2) p_re(2) + c_rh(2,3)],'ZData',[p_re(3) p_re(3) + c_rh(3,3)]);

                set(h5(1),'XData',[p_rhd(1) p_rhd(1) + c_ru(1,1)],'YData',[p_rhd(2) p_rhd(2) + c_ru(2,1)],'ZData',[p_rhd(3) p_rhd(3) + c_ru(3,1)]);
                set(h5(2),'XData',[p_rhd(1) p_rhd(1) + c_ru(1,2)],'YData',[p_rhd(2) p_rhd(2) + c_ru(2,2)],'ZData',[p_rhd(3) p_rhd(3) + c_ru(3,2)]);
                set(h5(3),'XData',[p_rhd(1) p_rhd(1) + c_ru(1,3)],'YData',[p_rhd(2) p_rhd(2) + c_ru(2,3)],'ZData',[p_rhd(3) p_rhd(3) + c_ru(3,3)]);

                % Draw the Arm
                set(h6,'XData',[0 p_rs(1) p_re(1) p_rhd(1)],'YData',[0 p_rs(2) p_re(2) p_rhd(2)],'ZData',[0 p_rs(3) p_re(3) p_rhd(3)]);
                set(hleft,'XData',[0 p_ls(1) p_le(1) p_lhd(1)],'YData',[0 p_ls(2) p_le(2) p_lhd(2)],'ZData',[0 p_ls(3) p_le(3) p_lhd(3)]);

                if time(i) > lastframetime + dt;
                    M(j) = getframe(gcf);
                    lastframetime = time(i);
                    j = j+1;
                end

                % Small delay for animation 
                pause(0.01)
            end

            c_ls = [c_lss(:,2) c_lss(:,3) -c_lss(:,1)];
            c_lh = [-c_lhs(:,3) -c_lhs(:,1) -c_lhs(:,2)];
            c_lu = [-c_lus(:,3) -c_lus(:,1) -c_lus(:,2)];

            T_ls = [c_ls [0 0 0]'; 0 0 0 1];
            T_lh = [c_lh [0 0 0]'; 0 0 0 1];
            T_lu = [c_lu [0 0 0]'; 0 0 0 1];

            % Calculate Joint Angles
            angleData(i,4) = acos(dot(T_lh(1:3,2),T_lu(1:3,2)));

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%% This thing here is bad. We should instead implement an
            %%%%%%%%% Extended Kalman Filter 
            %%%%%%%%% This is just a stop measure 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if (angleData(i,4) > pi/10)
                % Calculate the x and z axes of the humerus according to the
                % direction of the wrist sensor

            else
                % Calcualte the x and z axes of the humerus according to
                % changes as defined by the elbow sensor

            end

    %         [p, e, a] = shoulderAngles(T_ls,T_lh,'R');
    %         
    %         angleData(i,1) = p;
    %         angleData(i,2) = e;
    %         angleData(i,3) = a;
    %         
    %         
    %         angleData(i,5) = acos(dot(T_lh(1:3,3),T_lu(1:3,3)));   
    %         
    %         PE(i,:) = [sin(-e)*sin(p), -cos(e), sin(-e)*cos(p)];
    %         
    %         angleDataM(i,1) = -e*cos(p);
    %         angleDataM(i,2) = -e*sin(p);
    %         angleDataM(i,3) = a +p+pi/2 ;
    %         
    %         angleDataM(i,4:5) = angleData(i,4:5);
        end
end
%%
if 0
    movie2avi(M,'Movie','fps',framerate)
end

start = 1;
pend = length(time);

figure(10)
subplot(2,1,1)
plot(time(start:pend)-ones(size(time(start:pend)))*time(start),angleDataM(start:pend,1:3)*180/pi)
legend('Abd/Add','Flex/Ext','Rot')
title('Joint Angles - Shoulder')
ylabel('Angle (degs)')
subplot(2,1,2)
plot(time(start:pend)-ones(size(time(start:pend)))*time(start),angleDataM(start:pend,4:5)*(180/pi))
legend('Elbow','Wrist')
title('Joint Angles - Forearm')
ylabel('Angle (degs)')
xlabel('Time (s)')
title('Joint Angles - Forearm')