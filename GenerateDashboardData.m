% Script to take the Recording data values and put them into the format of
% the dashboard
R = Recording('Data/20160406-Subject1.h5', 60, 60*35, 1, 'R')

R = R.calcEverything()

%%
% Calculate activity values of each minute
t = R.t;
MoveIdx = R.MovIdx;
theta = R.Theta;
dt = R.dt;
movements = R.Movements;

% Somehow calculate the start time
%startTime;

% Indices of each minute start
minIndx = [10:60/dt:length(t)];


    % Headers for CSV
headers = {'Time',...
    'Activity Level',...
    'Total Movements',...
    '# Shoulder Abd/Add',...
    '# Shoulder Flex/Ext',...
    '# Shoulder Axial Rot',...
    '# Elbow Flex/Ext',...
    '# Wrist Pro/Sup',...
    'Max Shoulder Abd',...
    'Max Shoulder Flex',...
    'Max Shoulder Axial Rot',...
    'Max Elbow Flex',...
    'Max Wrist Pro',...
    'Min Shoulder Add',...
    'Min Shouder Ext',...
    'Min Shoulder Axial Rot',...
    'Min Elbow Ext',...
    'Min Wrist Sup'}

ActivityLevel = zeros(length(minIndx)+1,1);
NumMov = zeros(length(minIndx)+1,6)+ones(length(minIndx)+1,1)*[0, 0.02,0.04,0.06,0.08,0.1]*5;
ROM = zeros(length(minIndx)+1,10);


j = 1; % Movement Index
for i = 1:length(minIndx)+1
    if i == 1
        start = 1;
        stop = minIndx(1) -1;
    elseif i >length(minIndx)
        start = minIndx(length(minIndx));
        stop = length(t);
    else
        start = minIndx(i-1);
        stop = minIndx(i)-1;
    end
    
    ActivityLevel(i) = mean(MoveIdx(start:stop))*100;
    
    while (j <= length(movements) && movements(j).StartTime < (stop))
        NumMov(i,1) = NumMov(i,1)+1;
        NumMov(i,2:6) = NumMov(i,2:6) + movements(i).DoFPart;
        j = j+1;
    end
    
    % Calculate ROM for each Joint for each minute
    ROM(i,:) = [max(theta(start:stop,:)), min(theta(start:stop,:))];
    
end
ROM = rad2deg(ROM);

%%
figure(1)
subplot(3,1,1)
plot(ActivityLevel)
title('Activity Level')
subplot(3,1,2)
plot(NumMov)
legend('Total','Abd/add','Flex/Ext','AxRot','El','Wri');
title('Number of Movements')
subplot(3,1,3)
hold on
plot([1:length(minIndx)+1],ROM(:,1:5),'-')
ax = gca;
ax.ColorOrderIndex = 1;
plot([1:length(minIndx)+1],ROM(:,6:10),'--')
legend('Abd/add','Flex/Ext','AxRot','El','Wri');

hold off
title('ROM')

% Put everything into a CSV File


% Fetch Heat Maps