% Data from wearable sensors, and associated processing and metrics
classdef Recording
    properties (GetAccess=public)
        % Raw Data
        Filename
        q
        dt
    end
    properties (GetAccess=public)
        TMP
        
        %Constants
        JointsLimits=[[-80 180];[-80 180];[-90 90];[0 160];[0 180]];
        JointsNames={'Shoulder abduction', 'Shoulder flexion', 'Shoulder internal rotation', 'Elbow flexion', 'Pronation'};
        HistNbBins;


        %Recording parameters
        Arm
        DurationMin
        DurationSec
        NbPts
        t

        % Processed Data
        Theta %Actual joint angles
        Theta_d %Joint velocities
        SimplifiedTheta %Joint angles as: shoulder elevation (away from body) and elbow flexion
        SimplifiedTheta_d %Joint velocities
        XHand %Cartesian hand positions
        XHand_d %Cartesian hand velocities
        XShoulder
        XElbow
        L           % Neck-Should, UA, LA
        
        %Metrics
        MovIdx
        MovIdxPerJoint
        ThetaCum
        ROM
        NbMov
        MovTime
        MovTimePerJoint
        
        %Maps
        StaticHandMap
        MovHandMap
        GlobalHandMap
        StaticJointMap
        MovJointMap
        GlobalJointMap
        GlobalIndivJointHist
        StaticIndivJointHist
        MovIndivJointHist
        
        %For each movement
        Movements

    end
    
    methods
        %% Constructors
        
        function R = Recording(filename, StartTime, EndTime, fs, arm)
            
           % Save Filename into object
           R.Filename = filename;
           R.Arm=arm;
           
           %Test filename extension: if h5 assume Opal
           %If csv assume custom
           [~, ~, ext] = fileparts(R.Filename);
           switch(ext)
               case '.h5'
                   disp(['Opal sensors file: ' R.Filename]);
                   %Check number of sensors in the recording
                   hinfo = hdf5info(Filename);
                   nb_sensors=length(hinfo.GroupHierarchy.Groups);
                   if(nb_sensors<6 || ForceChestSensor)
                       R = RecordingH5ChestSensor(R, StartTime, EndTime);
                   else
                       R = RecordingH5(R, StartTime, EndTime); 
                   end
               
               case '.csv'
                   disp(['Own sensors file: ' R.Filename]);
                   R = RecordingCSV(R, StartTime, EndTime, fs);
                   
               otherwise
                   disp(['UNKNOWN sensors file: ' R.Filename]);
                   disp('ABORT');
           end
        end


        % Filename, StartTime, Endtime, Sampling Frequency (nominal)
        %for csv files (custom sensors)
        function R = RecordingCSV(R, StartTime, EndTime, fs)
            
            % Arm segments lengths
            R.L=[0.2 0.4 0.4]; %neck-shoulder, upper-arm, lower-arm
            
            % Load the data
            data = csvread(R.Filename,19,0);

            % Take only data of interest
            StartIndex = 1;
            EndIndex = 1;
            for i = 1:length(data)
                if data(i,1)/1000 < StartTime
                    StartIndex = i;
                end
                if data(i,1)/1000 < EndTime
                    EndIndex = i;
                end
            end
            data = data(StartIndex:EndIndex,:);
            
            % Save the raw quaternion data
            R.q = data;
            
            % Calculate an appropriate nominal dt
            R.dt = 1/fs;
            
            % Calculate the DoF angles
            R = calcDoFAngles(R);
        end


        % Filename, StartTime, Endtime, Sampling Frequency (nominal)
        %for Opal h5 files with shoulder sensors
        function R = RecordingH5(R, StartTime, EndTime)
            
            % Arm segments lengths
            R.L=[0.2 0.4 0.4]; %neck-shoulder, upper-arm, lower-arm
            
            % Load the data
                %Convenience sensor index variables
                L_Shoulder=1;
                L_Elbow=2;
                L_Wrist=3;
                R_Shoulder=4;
                R_Elbow=5;
                R_Wrist=6;

                %Opal sensors IDs
                SensorsIds(L_Shoulder, :)='SI-002500';
                SensorsIds(L_Elbow, :)='SI-002532';
                SensorsIds(L_Wrist, :)='SI-002575';
                SensorsIds(R_Shoulder, :)='SI-002545';
                SensorsIds(R_Elbow, :)='SI-002555';
                SensorsIds(R_Wrist, :)='SI-002561';

                %Corresponding labels
                SensorsLabels{L_Shoulder}='L Shoulder';
                SensorsLabels{L_Elbow}='L_Elbow';
                SensorsLabels{L_Wrist}='L_Wrist';
                SensorsLabels{R_Shoulder}='R_Shoulder';
                SensorsLabels{R_Elbow}='R_Elbow';
                SensorsLabels{R_Wrist}='R_Wrist';
            
                if(R.Arm=='R')
                    sdata = h5read(R.Filename,['/' SensorsIds(R_Shoulder, :) '/Calibrated/Orientation'])';
                    edata = h5read(R.Filename,['/' SensorsIds(R_Elbow, :) '/Calibrated/Orientation'])';
                    wdata = h5read(R.Filename,['/' SensorsIds(R_Wrist, :) '/Calibrated/Orientation'])';
                    stime = h5read(R.Filename,['/' SensorsIds(R_Shoulder, :) '/Time']);
                    etime = h5read(R.Filename,['/' SensorsIds(R_Elbow, :) '/Time']);
                    wtime = h5read(R.Filename,['/' SensorsIds(R_Wrist, :) '/Time']);
                end

                if(R.Arm=='L')
                    sdata = h5read(R.Filename,['/' SensorsIds(L_Shoulder, :) '/Calibrated/Orientation'])';
                    edata = h5read(R.Filename,['/' SensorsIds(L_Elbow, :) '/Calibrated/Orientation'])';
                    wdata = h5read(R.Filename,['/' SensorsIds(L_Wrist, :) '/Calibrated/Orientation'])';
                    stime = h5read(R.Filename,['/' SensorsIds(L_Shoulder, :) '/Time']);
                    etime = h5read(R.Filename,['/' SensorsIds(L_Elbow, :) '/Time']);
                    wtime = h5read(R.Filename,['/' SensorsIds(L_Wrist, :) '/Time']);
                end
                
                RecordingStartTime = max([stime(1) etime(1) wtime(1)]);
                RecordingEndTime = min([stime(length(stime)) etime(length(etime)) wtime(length(wtime))]);
                RecordingStartTimeIndex = [find(stime==RecordingStartTime), find(etime==RecordingStartTime), find(wtime==RecordingStartTime)];
                RecordingEndTimeIndex = [find(stime==RecordingEndTime), find(etime==RecordingEndTime), find(wtime==RecordingEndTime)];

                sdata = sdata(RecordingStartTimeIndex(1):RecordingEndTimeIndex(1),:);
                edata = edata(RecordingStartTimeIndex(2):RecordingEndTimeIndex(2),:);
                wdata = wdata(RecordingStartTimeIndex(3):RecordingEndTimeIndex(3),:);

                time = stime(RecordingStartTimeIndex(1):RecordingEndTimeIndex(1),:);
                time = (double(time) - double(time(1))*ones(size(time)))/1e6;
                
                %Build data vector: [time q_u q_h q_s]    
                data=[time.*1000 wdata edata sdata];
             
            % Take only data of interest
            [~, StartIndex] = min(abs(time-StartTime)); %Find closest time to start time
            [~, EndIndex] = min(abs(time-EndTime)); %Find closest time to end time
            tsec=time(EndIndex)-time(StartIndex);
            R.DurationMin=floor((time(EndIndex)-time(StartIndex))/60);
            R.DurationSec=round(tsec-R.DurationMin*60);
            disp(['Actual sequence length: ' num2str(R.DurationMin) 'min ' num2str(R.DurationSec) 's']);
            data = data(StartIndex:EndIndex,:);
            
            % Save the raw quaternion data
            R.q = data;
            
            % Calculate an appropriate nominal dt
            R.dt = mean(time(2:end)-time(1:end-1));
            
            % Calculate the DoF angles
            R = calcDoFAngles(R);
        end

        
        % Filename, StartTime, Endtime, Sampling Frequency (nominal)
        %for Opal h5 files with a chest sensor (instead of shoulder ones)
        function R = RecordingH5ChestSensor(R, StartTime, EndTime)
            
            % Arm segments lengths
            R.L=[0.2 0.4 0.4]; %neck-shoulder, upper-arm, lower-arm
            
            % Load the data
                %Convenience sensor index variables
                Chest=1;
                L_Elbow=2;
                L_Wrist=3;
                R_Elbow=4;
                R_Wrist=5;

                %Opal sensors IDs
                SensorsIds(Chest, :)='SI-002500';
                SensorsIds(L_Elbow, :)='SI-002532';
                SensorsIds(L_Wrist, :)='SI-002575';
                SensorsIds(R_Elbow, :)='SI-002555';
                SensorsIds(R_Wrist, :)='SI-002561';

                %Corresponding labels
                SensorsLabels{Chest}='Chest';
                SensorsLabels{L_Elbow}='L_Elbow';
                SensorsLabels{L_Wrist}='L_Wrist';
                SensorsLabels{R_Elbow}='R_Elbow';
                SensorsLabels{R_Wrist}='R_Wrist';
            
                if(R.Arm=='R')
                    cdata = h5read(R.Filename,['/' SensorsIds(Chest, :) '/Calibrated/Orientation'])';
                    edata = h5read(R.Filename,['/' SensorsIds(R_Elbow, :) '/Calibrated/Orientation'])';
                    wdata = h5read(R.Filename,['/' SensorsIds(R_Wrist, :) '/Calibrated/Orientation'])';
                    ctime = h5read(R.Filename,['/' SensorsIds(Chest, :) '/Time']);
                    etime = h5read(R.Filename,['/' SensorsIds(R_Elbow, :) '/Time']);
                    wtime = h5read(R.Filename,['/' SensorsIds(R_Wrist, :) '/Time']);
                end

                if(R.Arm=='L')
                    cdata = h5read(R.Filename,['/' SensorsIds(Chest, :) '/Calibrated/Orientation'])';
                    edata = h5read(R.Filename,['/' SensorsIds(L_Elbow, :) '/Calibrated/Orientation'])';
                    wdata = h5read(R.Filename,['/' SensorsIds(L_Wrist, :) '/Calibrated/Orientation'])';
                    ctime = h5read(R.Filename,['/' SensorsIds(Chest, :) '/Time']);
                    etime = h5read(R.Filename,['/' SensorsIds(L_Elbow, :) '/Time']);
                    wtime = h5read(R.Filename,['/' SensorsIds(L_Wrist, :) '/Time']);
                end
                
                RecordingStartTime = max([stime(1) etime(1) wtime(1)]);
                RecordingEndTime = min([stime(length(stime)) etime(length(etime)) wtime(length(wtime))]);
                RecordingStartTimeIndex = [find(stime==RecordingStartTime), find(etime==RecordingStartTime), find(wtime==RecordingStartTime)];
                RecordingEndTimeIndex = [find(stime==RecordingEndTime), find(etime==RecordingEndTime), find(wtime==RecordingEndTime)];

                cdata = cdata(RecordingStartTimeIndex(1):RecordingEndTimeIndex(1),:);
                edata = edata(RecordingStartTimeIndex(2):RecordingEndTimeIndex(2),:);
                wdata = wdata(RecordingStartTimeIndex(3):RecordingEndTimeIndex(3),:);

                time = stime(RecordingStartTimeIndex(1):RecordingEndTimeIndex(1),:);
                time = (double(time) - double(time(1))*ones(size(time)))/1e6;
                
                %Build data vector: [time q_u q_h q_c]
                data=[time.*1000 wdata edata cdata];
             
            % Take only data of interest
            [~, StartIndex] = min(abs(time-StartTime)); %Find closest time to start time
            [~, EndIndex] = min(abs(time-EndTime)); %Find closest time to end time
            tsec=time(EndIndex)-time(StartIndex);
            R.DurationMin=floor((time(EndIndex)-time(StartIndex))/60);
            R.DurationSec=round(tsec-R.DurationMin*60);
            disp(['Actual sequence length: ' num2str(R.DurationMin) 'min ' num2str(R.DurationSec) 's']);
            data = data(StartIndex:EndIndex,:);
            
            % Save the raw quaternion data
            R.q = data;
            
            % Calculate an appropriate nominal dt
            R.dt = mean(time(2:end)-time(1:end-1));
            
            % Calculate the DoF angles
            R = calcDoFAnglesChestSensor(R);
        end



        %% Data Processing
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Calculate angles of the joints with reference sensor on the
        % shoulder
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = calcDoFAngles(obj)
            q=obj.q;
            angleData = zeros(length(obj.q), 5);
            angleDataM = zeros(length(obj.q), 5);
            XHand = zeros(length(obj.q), 3);
            XShoulder = zeros(length(obj.q), 3);
            XElbow = zeros(length(obj.q), 3);
            SimplifiedTheta = zeros(length(obj.q), 2);
            %segments lengths
            l_s=obj.L(1);
            l_h=obj.L(2);
            l_u=obj.L(3);
            
            % Rotation 180 degrees about x
            R = [1 0 0;
                 0 -1 0;
                 0 0 -1];

            % Calculate the angles and hand position
            h = waitbar(0, 'Calculating joint angles...');
            
            for i = 1:length(q)
                waitbar(i/length(q));
                q_h = quatinv(q(i,6:9)/quatnorm(q(i,6:9)));
                q_u = quatinv(q(i,2:5)/quatnorm(q(i,2:5)));
                q_s = quatinv(q(i,10:13)/quatnorm(q(i,10:13)));

                % Creates frames for shoulder, humerus and ulna sensors
                c_ss = quatrotate(q_s,eye(3))';
                c_hs = quatrotate(q_h,eye(3))';
                c_us = quatrotate(q_u,eye(3))';

                c_s = [c_ss(:,2) c_ss(:,3) c_ss(:,1)];
                c_h = [-c_hs(:,3) -c_hs(:,1) c_hs(:,2)];
                c_u = [-c_us(:,3) -c_us(:,1) c_us(:,2)];

                T_s = [c_s [0 0 0]'; 0 0 0 1];
                T_h = [c_h [0 0 0]'; 0 0 0 1];
                T_u = [c_u [0 0 0]'; 0 0 0 1];
                
                p_s = c_s(:,3)*l_s;
                p_e =  p_s - c_h(:,2)*l_h;
                p_h = p_e - c_u(:,2)*l_u;
                XShoulder(i,:)=p_s;
                XElbow(i,:)=p_e;
                XHand(i,:)=p_h;

                % Calculate Joint Angles (as per ISB definition)
                [p, e, a] = shoulderAngles(T_s,T_h,obj.Arm);

                angleData(i,1) = p;
                angleData(i,2) = e;
                angleData(i,3) = a;
                angleData(i,4) = acos(dot(T_h(1:3,2),T_u(1:3,2)));
                angleData(i,5) = acos(dot(T_h(1:3,3),T_u(1:3,3)));
            end
            
            waitbar(1,h,'Resampling data...')
            
            % Normalise to start of file
            rawT = q(:,1)/1000 - q(1,1)/1000*ones(length(q),1);
            
            % Resample (and interpolate) the data:
            obj.t = [rawT(1):obj.dt:rawT(end)];
            
            % Ensure that first and last values are not NaNs:
            if(isnan(angleData(1,1)))
                non_nans_idx=find(isfinite(angleData(:,1)));
                angleData(1,1)=angleData(non_nans_idx(1),1);
            end
            if(isnan(angleData(end,1)))
                non_nans_idx=find(isfinite(angleData(:,1)));
                angleData(end,1)=angleData(non_nans_idx(end),1);
            end
            angleDataR = interp1(rawT,angleData,obj.t,'spline');
            XShoulder = spline(rawT,XShoulder',obj.t);
            XElbow = spline(rawT,XElbow',obj.t);
            XHand = spline(rawT,XHand',obj.t);

            
            %Create actual joint angles (abduction, flexion etc... from ISB
            %angles):
            angleDataM(:,1) = -angleDataR(:,2).*cos(angleDataR(:,1)); %Abduction/adduction (abduction external, -80 to 180)
            angleDataM(:,2) = -angleDataR(:,2).*sin(angleDataR(:,1)); %Flexion/extension (flexion forward, extension backward, -80 to 180)
            angleDataM(:,3) = angleDataR(:,3) + angleDataR(:,1);    %Axial rotation (internal -, external +, -90 to 90)
            angleDataM(:,4:5) = angleDataR(:,4:5);   %Elbow flexion (0-160) and pronation (0-180)

            %and simplified angles:
            SimplifiedTheta(:,1) = -angleDataR(:,2);  %Elevation: away from body
            SimplifiedTheta(:,2) = angleDataR(:,4);  %Elbow

%              figure();
%              subplot(2,1,1);hold on;
%              plot([angleData(:,1) angleData(:,2) angleData(:,3)]*180/pi);             
%              subplot(2,1,2);hold on;
%              plot(angleDataM(:,3)*180/pi, 'b');

            %Save to class members
            obj.Theta = angleDataM;
            obj.XShoulder = XShoulder';
            obj.XElbow = XElbow';
            obj.XHand = XHand';
            obj.SimplifiedTheta = SimplifiedTheta;
            obj.NbPts = length(obj.XHand);

            close(h);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Calculate angles of the joints with reference sensor on the
        % chest
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = calcDoFAnglesChestSensor(obj)
            q=obj.q;
            angleData = zeros(length(obj.q), 5);
            angleDataM = zeros(length(obj.q), 5);
            XHand = zeros(length(obj.q), 3);
            XShoulder = zeros(length(obj.q), 3);
            XElbow = zeros(length(obj.q), 3);
            SimplifiedTheta = zeros(length(obj.q), 2);
            %segments lengths
            l_s=obj.L(1);
            l_h=obj.L(2);
            l_u=obj.L(3);
            
            % Rotation 180 degrees about x
            R = [1 0 0;
                 0 -1 0;
                 0 0 -1];

            % Calculate the angles and hand position
            h = waitbar(0, 'Calculating joint angles...');
            
            for i = 1:length(q)
                waitbar(i/length(q));
                q_h = quatinv(q(i,6:9)/quatnorm(q(i,6:9)));
                q_u = quatinv(q(i,2:5)/quatnorm(q(i,2:5)));
                q_s = quatinv(q(i,10:13)/quatnorm(q(i,10:13)));

                % Creates frames for shoulder, humerus and ulna sensors
                c_ss = quatrotate(q_s,eye(3))';
                c_hs = quatrotate(q_h,eye(3))';
                c_us = quatrotate(q_u,eye(3))';


                % Use chest sensor as shoulder one, only nominal orientation change
                % but left and right arm have now different orientation since
                % based on a unique sensor
                if(obj.Arm=='R')
                    c_s = [c_ss(:,2) c_ss(:,3) c_ss(:,1)];
                else
                    c_s = [c_ss(:,2) c_ss(:,3) c_ss(:,1)];
                end
                c_h = [-c_hs(:,3) -c_hs(:,1) c_hs(:,2)];
                c_u = [-c_us(:,3) -c_us(:,1) c_us(:,2)];

                T_s = [c_s [0 0 0]'; 0 0 0 1];
                T_h = [c_h [0 0 0]'; 0 0 0 1];
                T_u = [c_u [0 0 0]'; 0 0 0 1];
                
                p_s = c_s(:,3)*l_s;
                p_e =  p_s - c_h(:,2)*l_h;
                p_h = p_e - c_u(:,2)*l_u;
                XShoulder(i,:)=p_s;
                XElbow(i,:)=p_e;
                XHand(i,:)=p_h;

                % Calculate Joint Angles (as per ISB definition)
                [p, e, a] = shoulderAngles(T_s,T_h,obj.Arm);

                angleData(i,1) = p;
                angleData(i,2) = e;
                angleData(i,3) = a;
                angleData(i,4) = acos(dot(T_h(1:3,2),T_u(1:3,2)));
                angleData(i,5) = acos(dot(T_h(1:3,3),T_u(1:3,3)));
            end
            
            waitbar(1,h,'Resampling data...')
            
            % Normalise to start of file
            rawT = q(:,1)/1000 - q(1,1)/1000*ones(length(q),1);
            
            % Resample (and interpolate) the data:
            obj.t = [rawT(1):obj.dt:rawT(end)];
            
            % Ensure that first and last values are not NaNs:
            if(isnan(angleData(1,1)))
                non_nans_idx=find(isfinite(angleData(:,1)));
                angleData(1,1)=angleData(non_nans_idx(1),1);
            end
            if(isnan(angleData(end,1)))
                non_nans_idx=find(isfinite(angleData(:,1)));
                angleData(end,1)=angleData(non_nans_idx(end),1);
            end
            angleDataR = interp1(rawT,angleData,obj.t,'spline');
            XShoulder = spline(rawT,XShoulder',obj.t);
            XElbow = spline(rawT,XElbow',obj.t);
            XHand = spline(rawT,XHand',obj.t);

            
            %Create actual joint angles (abduction, flexion etc... from ISB
            %angles):
            angleDataM(:,1) = -angleDataR(:,2).*cos(angleDataR(:,1)); %Abduction/adduction (abduction external, -80 to 180)
            angleDataM(:,2) = -angleDataR(:,2).*sin(angleDataR(:,1)); %Flexion/extension (flexion forward, extension backward, -80 to 180)
            angleDataM(:,3) = angleDataR(:,3) + angleDataR(:,1);    %Axial rotation (internal -, external +, -90 to 90)
            angleDataM(:,4:5) = angleDataR(:,4:5);   %Elbow flexion (0-160) and pronation (0-180)

            %and simplified angles:
            SimplifiedTheta(:,1) = -angleDataR(:,2);  %Elevation: away from body
            SimplifiedTheta(:,2) = angleDataR(:,4);  %Elbow

%              figure();
%              subplot(2,1,1);hold on;
%              plot([angleData(:,1) angleData(:,2) angleData(:,3)]*180/pi);             
%              subplot(2,1,2);hold on;
%              plot(angleDataM(:,3)*180/pi, 'b');

            %Save to class members
            obj.Theta = angleDataM;
            obj.XShoulder = XShoulder';
            obj.XElbow = XElbow';
            obj.XHand = XHand';
            obj.SimplifiedTheta = SimplifiedTheta;
            obj.NbPts = length(obj.XHand);

            close(h);
        end

        function obj = calcDoFVelocities(obj)
            theta = obj.Theta;
            simplified_theta = obj.SimplifiedTheta;
            t = obj.t;
            
            % Calculate the angular velocities of each DoF
            theta_dot = zeros(length(t), 5);
            dt = t(2)-t(1);
            for i = 1:length(t)-1
                theta_dot(i,:) = (theta(i+1,:)-theta(i,:))/dt;
            end
            theta_dot(length(t),:) = (theta(length(t),:)-theta(length(t)-1,:))/dt;
            % And filter
            Fs = 1/(t(2)-t(1));
            Fc = 10; 
            [b,a] = butter(5, Fc/Fs);
            obj.Theta_d = filter(b, a, theta_dot);
            
            % Calculate the angular velocities of simplified DoFs
            if(~isempty(simplified_theta))
                simplified_theta_dot = zeros(length(t), 2);
                dt = t(2)-t(1);
                for i = 1:length(t)-1
                    simplified_theta_dot(i,:) = (simplified_theta(i+1,:)-simplified_theta(i,:))/dt;
                end
                simplified_theta_dot(length(t),:) = (simplified_theta(length(t),:)-simplified_theta(length(t)-1,:))/dt;
                % And filter
                [b,a] = butter(5, Fc/Fs);
                obj.SimplifiedTheta_d = filter(b, a, simplified_theta_dot);
            end
        end

        %MATLAB can't find it for whatever reason...
        function s = createMovementStructure()
            %Per movement metrics and properties
            s = struct(...
            'IndROM', [],...
            'PeakVel', [],...
            'RatioSwivel', [],...
            'StartTime', -1,...
            'StopTime', -1,....
            'DoFPart',[]);
        end
        
        
        function obj = calcMove(obj)
            theta_dot = obj.Theta_d;
            f = 1/obj.dt;
            
            % Parameters
            % For each Joint
            velThres = 40;
            timeThresOn = 0.2;
            timeThresOff = 0.1;
            
            % For the sum of all joints
            velThresAll = 60;
            timeThresOnAll = 0.2;
            timeThresOffAll = 0.2;

            % Calculate whether a movement is currently being made
            % Index to determine whether each DoF is moving
            DoFMove = zeros(length(theta_dot), 5);
            
            sampleThresOn = floor(timeThresOn*f);
            sampleThresOff = floor(timeThresOff*f);

            % Filter the data
            Fs = f;    % Nominal Sampling Rate
            Fc = 10;    % Cutoff frequency for filter
            [b,a] = butter(5,Fc/Fs);
            filtheta_dot = filter(b,a,theta_dot);

            flagon = zeros(5,1);
            flagoff = zeros(5,1);
            for i = 1:length(theta_dot)
                for j = 1:5
                    if abs(filtheta_dot(i,j)*180/pi) > velThres
                        if flagon(j) > sampleThresOn
                            DoFMove(i,j) = 1;
                            flagon(j) = flagon(j) + 1;
                            flagoff(j) = 0;
                        elseif flagon(j) == sampleThresOn
                            DoFMove(i-sampleThresOn+1:i,j) = ones(sampleThresOn,1);       
                            flagoff(j) = 0;
                            flagon(j) = flagon(j) + 1;
                        else 
                            flagon(j) = flagon(j) + 1;
                        end
                    else
                        if flagon(j) > sampleThresOn
                            if flagoff(j) < sampleThresOff
                                DoFMove(i,j) = 1;
                                flagoff(j) = flagoff(j) + 1;
                                flagon(j) = flagon(j) + 1;
                            elseif flagoff(j) == sampleThresOff
                                DoFMove(i-sampleThresOff+1:i,j) = zeros(sampleThresOff,1); 
                                flagon(j) = 0;
                                flagoff(j) = flagoff(j) + 1;
                            end
                        else
                            flagon(j) = 0;
                            flagoff(j) = 0;
                        end
                    end
                end
            end
        
            % Calculate based on the sum of movements
            sampleThresOnAll = floor(timeThresOnAll*f);
            sampleThresOffAll = floor(timeThresOffAll*f);

            totVel = sum(abs(theta_dot)'.^2).^(1/2);          
            flagonTot = 0;
            flagoffTot = 0;
            movement = zeros(length(theta_dot), 1);

            for i = 1:length(theta_dot)
                if abs(totVel(i)*180/pi) > velThresAll
                    if flagonTot > sampleThresOnAll
                        movement(i) = 1;
                        flagonTot = flagonTot + 1;
                        flagoffTot = 0;
                    elseif flagonTot == sampleThresOnAll
                        movement(i-sampleThresOnAll+1:i) = ones(sampleThresOnAll,1);      
                        flagonTot  = flagonTot + 1;
                        flagoffTot = 0;
                    else 
                        flagonTot = flagonTot + 1;
                    end
                else               
                    if flagonTot > sampleThresOnAll
                        if flagoffTot < sampleThresOffAll
                            movement(i) = 1;
                            flagoffTot = flagoffTot + 1;
                            flagonTot = flagonTot + 1;

                        elseif flagoffTot == sampleThresOffAll
                            movement(i-sampleThresOffAll+1:i) = zeros(sampleThresOffAll,1); 

                            flagonTot = 0;
                            flagoffTot = flagoffTot + 1;
                        end
                    else
                        flagonTot = 0;
                        flagoffTot = 0;
                    end
                end
            end
            
            % Use movement as the method for determining a movement, and
            % moving as a method for determining whether each DoF is
            % moving.
            inMove = 0;
            numMov = 0;
            
            % Create Movement Structure
            for i = 1:length(movement)
                if inMove == 1
                    if movement(i) == 0
                        inMove = 0;
                        Movements(numMov).StopTime = i;
                    end
                else
                    if movement(i) > 0
                        numMov = numMov+1;
                        s = struct(...
                        'IndROM', [],...
                        'PeakVel', [],...
                        'RatioSwivel', [],...
                        'StartTime', -1,...
                        'StopTime', -1,....
                        'DoFPart',[]);
                        Movements(numMov) = s;
                        Movements(numMov).StartTime = i;
                        inMove = 1;
                    end
                end
            end
            
            for i = 1:length(Movements)
                Movements(i).DoFPart = zeros(1,5);
                for j = 1:5
                    if sum(DoFMove(Movements(i).StartTime:Movements(i).StopTime,j)) >0
                        Movements(i).DoFPart(j) = 1;
                    end
                end
            end
            
            % Now fill out the structure indicating whether a movement is
            % occuring at any particular point in time
            %TODO: WHAT IS movement: A KIND OF GLOBAL MOVEMENT INDEX ???
            obj.MovIdxPerJoint = DoFMove;
            obj.MovIdx = DoFMove(:,1)|DoFMove(:,2)|DoFMove(:,3)|DoFMove(:,4);
            obj.Movements = Movements;
        end
                
         
        %Done in calcDoFAngles for speed
%         function obj = calcHandPos(obj)
%             % Calcualte the Hand Positions
%             % VINCENT - TODO
%             theta = obj.Theta;
%             L = obj.L;
%         end
        

        %% Global Metrics
        function obj = calcEverything(obj)
            obj=calcDoFVelocities(obj);
            obj=calcMove(obj);
            obj=calcCumDoF(obj);
            obj=calcROM(obj);
            obj=calcNumMov(obj);
            obj=calcMovTime(obj);
            obj=createHandMaps(obj);
            obj=createJointMaps(obj);
            obj=createJointHist(obj);
        end
        
        
        function obj = calcCumDoF(obj)
            Fs = 1/obj.dt;
            theta = obj.Theta;
            
            Fc = 10;    % Cutoff frequency for smoothness measurement

            [b,a] = butter(5,Fc/Fs);
            filAngles = filter(b,a,theta);

            theta_cum = zeros(1,5);
            for i = 1:length(theta)-1
                theta_cum = theta_cum + abs(filAngles(i+1,:) - filAngles(i,:));
            end
            
            obj.ThetaCum = theta_cum;
        end
 
        function obj = calcROM(obj)
            % Requires 
            Theta = obj.Theta;
            
            %For each joint
            for i=1:size(Theta, 2)
                ROM(i, 1)=min(Theta(:,i));
                ROM(i, 2)=max(Theta(:,i));
            end
            obj.ROM=ROM;
        end

        function obj = calcNumMov(obj)
            obj.NbMov = length(obj.Movements);
        end

        function obj = calcMovTime(obj)
            %TODO: improve with actual dt for each
            dt=obj.dt;
            MovIdx=obj.MovIdx;
            MovIdxPerJoint=obj.MovIdxPerJoint;
            
            %Global
            obj.MovTime=sum(MovIdx)*dt;
            
            %PerJoint
            obj.MovTimePerJoint=sum(MovIdxPerJoint).*dt;
        end


        
        function [XEdges, YEdges, ZEdges]=DefineHandHistProperties(obj)%RESOLUTION as a parameter ?
            L=obj.L; % Neck-Should, UA, LA

            Resolution=0.02;
            %Center at neck point
            XEdges=[-(L(2)+L(3)):Resolution:L(2)+L(3)]; %X positive forward
            if(obj.Arm=='R')
                YEdges=[-(L(2)+L(3)+L(1)):Resolution:(L(2)+L(3)-L(1))]; %Y positive subject left
            else
                YEdges=[-(L(2)+L(3)-L(1)):Resolution:(L(2)+L(3)+L(1))]; %Y positive subject left
            end
            ZEdges=[-(L(2)+L(3)):Resolution:L(2)+L(3)]; %Z positive up
        end
        
        function obj = createHandMaps(obj)
            L=obj.L; % Neck-Should, UA, LA
            XHand=obj.XHand;
            
            [XEdges, YEdges, ZEdges]=DefineHandHistProperties(obj);

            %Global (all postures) histograms
                %Sagittal: Histogram x,z: side view, x positive forward, z positive up
                [XZCount, XZEdges, XZMid, XZLoc]=histcn([XHand(:,1) XHand(:,3)], XEdges, ZEdges);
                XZCount=(XZCount./max(max(XZCount)))*2-1;
                GlobalHandMap{1}=(XZCount(:,end:-1:1)'+1).*50;%Transpose to get Z on vertical

                %Coronal: Histogram x,y: front view, x positive forward, y side
                [ZYCount, ZYEdges, ZYMid, ZYLoc]=histcn([XHand(:,3) XHand(:,2)], ZEdges, YEdges);
                ZYCount=ZYCount./max(max(ZYCount))*2-1;
                GlobalHandMap{2}=(ZYCount(end:-1:1, :)+1).*50;

                obj.GlobalHandMap=GlobalHandMap;


            %Static only postures histograms
                MovIdx=obj.MovIdx;
                XHand=obj.XHand;
                XHand=XHand(find(MovIdx==0),:);

                %Sagittal: Histogram x,z: side view, x positive forward, z positive up
                [XZCount, XZEdges, XZMid, XZLoc]=histcn([XHand(:,1) XHand(:,3)], XEdges, ZEdges);
                XZCount=(XZCount./max(max(XZCount)))*2-1;
                StaticHandMap{1}=(XZCount(:,end:-1:1)'+1).*50;%Transpose to get Z on vertical

                %Coronal: Histogram x,y: front view, x positive forward, y side
                [ZYCount, ZYEdges, ZYMid, ZYLoc]=histcn([XHand(:,3) XHand(:,2)], ZEdges, YEdges);
                ZYCount=ZYCount./max(max(ZYCount))*2-1;
                StaticHandMap{2}=(ZYCount(end:-1:1, :)+1).*50;

                obj.StaticHandMap=StaticHandMap;


            %Movement only postures histograms
                MovIdx=obj.MovIdx;
                XHand=obj.XHand;
                XHand=XHand(find(MovIdx==1),:);
            
                %Sagittal: Histogram x,z: side view, x positive forward, z positive up
                [XZCount, XZEdges, XZMid, XZLoc]=histcn([XHand(:,1) XHand(:,3)], XEdges, ZEdges);
                XZCount=(XZCount./max(max(XZCount)))*2-1;
                MovHandMap{1}=(XZCount(:,end:-1:1)'+1).*50;%Transpose to get Z on vertical

                %Coronal: Histogram x,y: front view, x positive forward, y side
                [ZYCount, ZYEdges, ZYMid, ZYLoc]=histcn([XHand(:,3) XHand(:,2)], ZEdges, YEdges);
                ZYCount=ZYCount./max(max(ZYCount))*2-1;
                MovHandMap{2}=(ZYCount(end:-1:1, :)+1).*50;

                obj.MovHandMap=MovHandMap;
        end
        
        function obj = createJointMaps(obj)
            theta= obj.Theta;
        end

        function obj = createJointHist(obj)
            Theta=obj.Theta.*180/pi;
            JointsLimits=obj.JointsLimits;
            HistNbBins=30;
            obj.HistNbBins=HistNbBins;
            MovIdx=obj.MovIdx;
            MovTheta=Theta(find(MovIdx==1), :);
            StaticTheta=Theta(find(MovIdx==0), :);
            for i=1:size(Theta, 2)
                bin_size=(JointsLimits(i,2)-JointsLimits(i,1))/HistNbBins;
                xbins=[JointsLimits(i,1):bin_size:JointsLimits(i,2)];
                GlobalIndivJointHist(i,:)=hist(Theta(:, i), xbins)./length(Theta)*100; %Histogram and normalize
                StaticIndivJointHist(i,:)=hist(StaticTheta(:, i), xbins)./length(Theta)*100; %Histogram and normalize
                MovHistCoef = abs(obj.Theta_d(find(MovIdx==1),i))*180/pi/bin_size*obj.dt*10; %Correction coefficients for movement histogram: corrected by their velocity to not penalize fast samples (which are less 'likely')
                MovIndivJointHist(i,:)=whist(MovTheta(:, i), xbins, MovHistCoef)./length(find(MovIdx==1))*100; %Weighted histogram and normalize
            end
            obj.GlobalIndivJointHist=GlobalIndivJointHist;
            obj.StaticIndivJointHist=StaticIndivJointHist;
            obj.MovIndivJointHist=MovIndivJointHist;
        end

  
        %% Per Movement Metrics
        function obj = calcPMROM(obj)
            
        end
        
        function obj = calcPMPeakVel(obj)
        end
        
        function obj = calcPMRatioSwivel(obj)
        end

        %% Drawing methods
        
        function obj = drawEverything(obj)
            drawTheta(obj, 0);
            drawSimplifiedTheta(obj, 0);
            drawTheta_d(obj);
            drawSimplifiedTheta_d(obj);
            drawHandTraj(obj, 0);
            drawHandTraj3d(obj);
            drawGlobalHandMaps(obj, 0);
            drawStaticHandMaps(obj, 0);
            drawMovHandMaps(obj, 0);
            drawJointHists(obj);
            drawCircularJointHists(obj);
        end
        
        function h=drawTheta(obj, idx, varargin)
            Theta=obj.Theta;
            t=obj.t;
            MovIdx=obj.MovIdx;
            if(isempty(varargin))
                h=figure();
            else
                axes(varargin{1});
            end
            hold on;
            %Add movements area if any
            if(~isempty(MovIdx))
                yl=[min(min(Theta*180/pi)) max(max(Theta*180/pi))];
                for i=1:length(t)
                    if(MovIdx(i)==1)
                        line([t(i) t(i)], [yl(1) yl(2)], 'color', [0.8 0.8 0.8]) ;
                    end
                end
            end

            p(1)=plot(t, Theta(:,1)*180/pi, 'r');
            p(2)=plot(t, Theta(:,2)*180/pi, 'g');
            p(3)=plot(t, Theta(:,3)*180/pi, 'b');
            p(4)=plot(t, Theta(:,4)*180/pi, 'y');
            p(5)=plot(t, Theta(:,5)*180/pi, 'm');
            
            %Add a red line idx if needed
            if(idx>0)
                yl=ylim;
                %Line should be of different style than actual plots !
                line([t(idx) t(idx)], [yl(1) yl(2)], 'color', [1 0 0], 'LineStyle', '--', 'LineWidth', 2);
            end
            title('Joint angles');
            legend(p, ['q1';'q2';'q3';'q4';'q5']);
        end
        
        function h=drawSimplifiedTheta(obj, idx, varargin)
            if(~isempty(obj.SimplifiedTheta))
                SimplifiedTheta=obj.SimplifiedTheta;
                t=obj.t;
                MovIdx=obj.MovIdx;
                if(isempty(varargin))
                    h=figure();
                else
                    axes(varargin{1});
                end
                hold on;
                %Add movements area if any
                if(~isempty(MovIdx))
                    yl=[min(min(SimplifiedTheta*180/pi)) max(max(SimplifiedTheta*180/pi))];
                    for i=1:length(t)
                        if(MovIdx(i)==1)
                            line([t(i) t(i)], [yl(1) yl(2)], 'color', [0.8 0.8 0.8]) ;
                        end
                    end
                end

                p(1)=plot(t, SimplifiedTheta(:,1)*180/pi, 'r');
                p(2)=plot(t, SimplifiedTheta(:,2)*180/pi, 'g');

                title('Simplified joint angles');
                legend(p, {'Shoulder';'Elbow'});

                %Add a red line idx if needed
                if(idx>0)
                    yl=ylim;
                    %Line should be of different style than actual plots !
                    line([t(idx) t(idx)], [yl(1) yl(2)], 'color', [1 0 0], 'LineStyle', '--', 'LineWidth', 2);
                end
            end
        end
        
        function h=drawTheta_d(obj, varargin)
            Theta_d=obj.Theta_d;
            t=obj.t;
            if(isempty(varargin))
                h=figure();
            else
                axes(varargin{1});
            end
            hold on;
            plot(t, Theta_d(:,1)*180/pi, 'r');
            plot(t, Theta_d(:,2)*180/pi, 'g');
            plot(t, Theta_d(:,3)*180/pi, 'b');
            plot(t, Theta_d(:,4)*180/pi, 'y');
            plot(t, Theta_d(:,5)*180/pi, 'm');
            title('Joint velocities');
            legend(['dq1';'dq2';'dq3';'dq4';'dq5']);
        end
        
        function h=drawSimplifiedTheta_d(obj, varargin)
            SimplifiedTheta_d=obj.SimplifiedTheta_d;
            t=obj.t;
            if(isempty(varargin))
                h=figure();
            else
                axes(varargin{1});
            end
            hold on;
            plot(t, SimplifiedTheta_d(:,1)*180/pi, 'r');
            plot(t, SimplifiedTheta_d(:,2)*180/pi, 'g');
            title('Simplified joint velocities');
            legend({'Shoulder';'Elbow'});
        end
        
        function h=drawHandTraj(obj, idx, varargin)
            XHand=obj.XHand;
            t=obj.t;
            if(isempty(varargin))
                h=figure();
            else
                axes(varargin{1});
            end
            hold on;
            
            plot(t, XHand(:,1), 'r');
            plot(t, XHand(:,2), 'g');
            plot(t, XHand(:,3), 'b');
            
            %Add a red line idx if needed
            if(idx>0)
                yl=ylim;
                %Line should be of different style than actual plots !
                line([t(idx) t(idx)], [[yl(1) yl(2)]], 'color', [1 0 0], 'LineStyle', '--', 'LineWidth', 2) ;
            end
            
            title('Hand trajectory');
            legend(['x';'y';'z']);
        end
        function h=drawHandTraj3d(obj, varargin)
            XHand=obj.XHand;
            t=obj.t;
            if(isempty(varargin))
                h=figure();
            else
                axes(varargin{1});
            end
            plot3(XHand(:,1), XHand(:,2), XHand(:,3), 'r');
            title('Hand trajectory');
        end
 
        function h=drawArm3d(obj, i, varargin)
            XShoulder=obj.XShoulder;
            XElbow=obj.XElbow;
            XHand=obj.XHand;
            if(isempty(varargin))
                h=figure();
                h=plot3([0 0 0.4 0.8]',[0 0 0 0]',[0 0 0 0]','linewidth',10);
            else
                axes(varargin{1});
                h=plot3([0 0 0.4 0.8]',[0 0 0 0]',[0 0 0 0]','linewidth',10);
            end
            set(h,'XData',[0 XShoulder(i, 1) XElbow(i, 1) XHand(i, 1)],'YData',[0 XShoulder(i, 2) XElbow(i, 2) XHand(i, 2)],'ZData',[0 XShoulder(3) XElbow(i, 3) XHand(i, 3)]);
            axis equal
            title('Arm Posture');
        end
        
        function h=drawGlobalHandMaps(obj, write)
           GlobalHandMap=obj.GlobalHandMap;
           L=obj.L;
            
           [XEdges, YEdges, ZEdges]=DefineHandHistProperties(obj);
           
           %Where is (0,0,0) in the edges ? Z has to be inverted
            if(obj.Arm=='R')
               NeckCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (0-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ShouldCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (-L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ElbowCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (-L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               WristCoord=[(L(3)-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (-L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
            else
               NeckCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (0-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ShouldCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ElbowCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               WristCoord=[(L(3)-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
            end
            
           h=figure();
           colormap(hot);
           %Sagittal: Histogram x,z: side view, x positive forward, z positive up
                subplot(1,2,1);
                image(GlobalHandMap{1},'CDataMapping','scaled');%Transpose to get Z on vertical
                %plot neck point, elbow and wrist:
                hold on; plot(NeckCoord(1), NeckCoord(3), 'w+');plot(ShouldCoord(1), ShouldCoord(3), 'w+');plot(ElbowCoord(1), ElbowCoord(3), 'w+');plot(WristCoord(1), WristCoord(3), 'w+');
                line([NeckCoord(1) ShouldCoord(1) ElbowCoord(1) WristCoord(1)], [NeckCoord(3) ShouldCoord(3) ElbowCoord(3) WristCoord(3)], 'Color', 'w');
                %plot head: circle + eyes
                headPos=[NeckCoord(1)-3.5 NeckCoord(3)-9 7 8];
                rectangle('Position',headPos,'Curvature',[1 1], 'EdgeColor', 'w');%head
                plot(headPos(1)+headPos(3), headPos(2)+headPos(4)/3, 'wo');%eyes
                title('Sagittal');xlabel('X');ylabel('Z');
                if(write)
                    %imwrite(GlobalHandMap{1}, 'GlobalHandHeatMapSagittal.png');
                end
            
           %Coronal: Histogram x,y: front view, x positive forward, y side
                subplot(1,2,2);
                image(GlobalHandMap{2},'CDataMapping','scaled');
                %plot neck point, elbow and wrist:
                hold on; plot(NeckCoord(2), NeckCoord(3), 'w+');plot(ShouldCoord(2), ShouldCoord(3), 'w+');plot(ElbowCoord(2), ElbowCoord(3), 'w+');plot(WristCoord(2), WristCoord(3), 'w+');
                line([NeckCoord(2) ShouldCoord(2) ElbowCoord(2) WristCoord(2)], [NeckCoord(3) ShouldCoord(3) ElbowCoord(3) WristCoord(3)], 'Color', 'w');
                %plot head: circle + eyes
                headPos=[NeckCoord(2)-4 NeckCoord(3)-9 8 8];
                rectangle('Position',headPos,'Curvature',[1 1], 'EdgeColor', 'w');%head
                plot(headPos(1)+headPos(3)./3, headPos(2)+headPos(4)/3, 'wo');plot(headPos(1)+headPos(3)*2/3, headPos(2)+headPos(4)/3, 'wo');%eyes
                title('Coronal');xlabel('Y');ylabel('Z');
                if(write)
                    %imwrite(GlobalHandMap{2}, 'GlobalHandHeatMapCoronal.png');
                end

                suptitle('Global (all postures)');
                
                if(write)
                    print(h, '-dtiff', 'GlobalHandHeatMap.tif');
                end
        end       
        function h=drawStaticHandMaps(obj, write)
           StaticHandMap=obj.StaticHandMap;
           L=obj.L;
           
           [XEdges, YEdges, ZEdges]=DefineHandHistProperties(obj);
           
           %Where is (0,0,0) in the edges ? Z has to be inverted
          if(obj.Arm=='R')
               NeckCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (0-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ShouldCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (-L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ElbowCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (-L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               WristCoord=[(L(3)-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (-L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
           else
               NeckCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (0-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ShouldCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ElbowCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               WristCoord=[(L(3)-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
           end
            
           h=figure();
           colormap(hot);
           %Sagittal: Histogram x,z: side view, x positive forward, z positive up
                subplot(1,2,1);
                image(StaticHandMap{1},'CDataMapping','scaled');%Transpose to get Z on vertical
                %plot neck point, elbow and wrist:
                hold on; plot(NeckCoord(1), NeckCoord(3), 'w+');plot(ShouldCoord(1), ShouldCoord(3), 'w+');plot(ElbowCoord(1), ElbowCoord(3), 'w+');plot(WristCoord(1), WristCoord(3), 'w+');
                line([NeckCoord(1) ShouldCoord(1) ElbowCoord(1) WristCoord(1)], [NeckCoord(3) ShouldCoord(3) ElbowCoord(3) WristCoord(3)], 'Color', 'w');
                %plot head: circle + eyes
                headPos=[NeckCoord(1)-3.5 NeckCoord(3)-9 7 8];
                rectangle('Position',headPos,'Curvature',[1 1], 'EdgeColor', 'w');%head
                plot(headPos(1)+headPos(3), headPos(2)+headPos(4)/3, 'wo');%eyes
                title('Sagittal');xlabel('X');ylabel('Z');
                if(write)
                    %imwrite(StaticHandMap{1}, 'StaticHandHeatMapSagittal.png');
                end
            
           %Coronal: Histogram x,y: front view, x positive forward, y side
                subplot(1,2,2);
                image(StaticHandMap{2},'CDataMapping','scaled');
                %plot neck point, elbow and wrist:
                hold on; plot(NeckCoord(2), NeckCoord(3), 'w+');plot(ShouldCoord(2), ShouldCoord(3), 'w+');plot(ElbowCoord(2), ElbowCoord(3), 'w+');plot(WristCoord(2), WristCoord(3), 'w+');
                line([NeckCoord(2) ShouldCoord(2) ElbowCoord(2) WristCoord(2)], [NeckCoord(3) ShouldCoord(3) ElbowCoord(3) WristCoord(3)], 'Color', 'w');
                %plot head: circle + eyes
                headPos=[NeckCoord(2)-4 NeckCoord(3)-9 8 8];
                rectangle('Position',headPos,'Curvature',[1 1], 'EdgeColor', 'w');%head
                plot(headPos(1)+headPos(3)./3, headPos(2)+headPos(4)/3, 'wo');plot(headPos(1)+headPos(3)*2/3, headPos(2)+headPos(4)/3, 'wo');%eyes
                title('Coronal');xlabel('Y');ylabel('Z');
                if(write)
                    %imwrite(StaticHandMap{2}, 'StaticHandHeatMapCoronal.png');
                end

                suptitle('Static postures only');
                
                if(write)
                    print(h, '-dtiff', 'StaticHandHeatMap.tif');
                end
        end
        function h=drawMovHandMaps(obj, write)
           MovHandMap=obj.MovHandMap;
           L=obj.L;
            
           [XEdges, YEdges, ZEdges]=DefineHandHistProperties(obj);

           %Where is (0,0,0) in the edges ? Z has to be inverted
           if(obj.Arm=='R')
               NeckCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (0-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ShouldCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (-L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ElbowCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (-L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               WristCoord=[(L(3)-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (-L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
           else
               NeckCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (0-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ShouldCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (0-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               ElbowCoord=[(0-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
               WristCoord=[(L(3)-min(XEdges))*(length(XEdges)/(max(XEdges)-min(XEdges))) (L(1)-min(YEdges))*(length(YEdges)/(max(YEdges)-min(YEdges))) (-L(2)-max(ZEdges))*(length(ZEdges)/(min(ZEdges)-max(ZEdges)))];
           end
            
           h=figure();
           colormap(hot);
           %Sagittal: Histogram x,z: side view, x positive forward, z positive up
                subplot(1,2,1);
                image(MovHandMap{1},'CDataMapping','scaled');%Transpose to get Z on vertical
                %plot neck point, elbow and wrist:
                hold on; plot(NeckCoord(1), NeckCoord(3), 'w+');plot(ShouldCoord(1), ShouldCoord(3), 'w+');plot(ElbowCoord(1), ElbowCoord(3), 'w+');plot(WristCoord(1), WristCoord(3), 'w+');
                line([NeckCoord(1) ShouldCoord(1) ElbowCoord(1) WristCoord(1)], [NeckCoord(3) ShouldCoord(3) ElbowCoord(3) WristCoord(3)], 'Color', 'w');
                %plot head: circle + eyes
                headPos=[NeckCoord(1)-3.5 NeckCoord(3)-9 7 8];
                rectangle('Position',headPos,'Curvature',[1 1], 'EdgeColor', 'w');%head
                plot(headPos(1)+headPos(3), headPos(2)+headPos(4)/3, 'wo');%eyes
                title('Sagittal');xlabel('X');ylabel('Z');
                if(write)
                    %imwrite(MovHandMap{1}, 'MovHandHeatMapSagittal.png');
                end
            
           %Coronal: Histogram x,y: front view, x positive forward, y side
                subplot(1,2,2);
                image(MovHandMap{2},'CDataMapping','scaled');
                %plot neck point, elbow and wrist:
                hold on; plot(NeckCoord(2), NeckCoord(3), 'w+');plot(ShouldCoord(2), ShouldCoord(3), 'w+');plot(ElbowCoord(2), ElbowCoord(3), 'w+');plot(WristCoord(2), WristCoord(3), 'w+');
                line([NeckCoord(2) ShouldCoord(2) ElbowCoord(2) WristCoord(2)], [NeckCoord(3) ShouldCoord(3) ElbowCoord(3) WristCoord(3)], 'Color', 'w');
                %plot head: circle + eyes
                headPos=[NeckCoord(2)-4 NeckCoord(3)-9 8 8];
                rectangle('Position',headPos,'Curvature',[1 1], 'EdgeColor', 'w');%head
                plot(headPos(1)+headPos(3)./3, headPos(2)+headPos(4)/3, 'wo');plot(headPos(1)+headPos(3)*2/3, headPos(2)+headPos(4)/3, 'wo');%eyes
                title('Coronal');xlabel('Y');ylabel('Z');
                if(write)
                    %imwrite(MovHandMap{2}, 'MovHandHeatMapCoronal.png');
                end

                suptitle('Movement postures only');
                
                if(write)
                    print(h, '-dtiff', 'MovHandHeatMap.tif');
                end
        end
        
        function h=drawJointHists(obj, varargin)
            GlobalIndivJointHist=obj.GlobalIndivJointHist;
            StaticIndivJointHist=obj.StaticIndivJointHist;
            MovIndivJointHist=obj.MovIndivJointHist;
            JointsLimits=obj.JointsLimits;
            JointsNames=obj.JointsNames;
            HistNbBins=obj.HistNbBins;
            if(isempty(varargin))
                h=figure();
            else
                axes(varargin{1});
            end
            %For each joint
            for i=1:size(GlobalIndivJointHist, 1)
                subplot(size(GlobalIndivJointHist, 1), 1, i);
                xbins=[JointsLimits(i,1):(JointsLimits(i,2)-JointsLimits(i,1))/HistNbBins:JointsLimits(i,2)];
                bar(xbins, [GlobalIndivJointHist(i,:)' StaticIndivJointHist(i,:)' MovIndivJointHist(i,:)']);
                title(JointsNames{i});
                if(i==1)
                    legend({'Global' 'Static', 'Movement'}, 'Location','NorthWest');
                end
            end
        end
        
        function h=drawCircularJointHists(obj, varargin)
            GlobalIndivJointHist=obj.GlobalIndivJointHist;
            StaticIndivJointHist=obj.StaticIndivJointHist;
            MovIndivJointHist=obj.MovIndivJointHist;
            JointsLimits=obj.JointsLimits;
            JointsNames=obj.JointsNames;
            HistNbBins=obj.HistNbBins;
            if(isempty(varargin))
                h=figure();
            else
                axes(varargin{1});
            end
            
            %Polar (circular) histograms
            for i=1:size(GlobalIndivJointHist, 1)
                subplot(2,3,i);
                xbins=[JointsLimits(i,1):(JointsLimits(i,2)-JointsLimits(i,1))/HistNbBins:JointsLimits(i,2)];
                polar(xbins./180*pi, GlobalIndivJointHist(i,:), 'b');hold on
                polar(xbins./180*pi, StaticIndivJointHist(i,:), 'g');
                polar(xbins./180*pi, MovIndivJointHist(i,:), 'r');
                title(obj.JointsNames{i});
                if(i==1)
                    legend({'Global' 'Static', 'Movement'}, 'Location','NorthWestOutside');
                end
            end
        end


        %% Export methods (to csv files)
        
        function obj = exportAllCSV(obj)
            obj.exportHeatMaps();
        end

        function exportHeatMaps(obj)
            StaticHandMapSide=obj.StaticHandMap{1};
            StaticHandMapFront=obj.StaticHandMap{2};
            MovHandMapSide=obj.MovHandMap{1};
            MovHandMapFront=obj.MovHandMap{2};
            csvwrite([obj.Filename '_StaticHeatMapSide.csv'], StaticHandMapSide);
            csvwrite([obj.Filename '_StaticHeatMapFront.csv'], StaticHandMapFront);
            csvwrite([obj.Filename '_MovHeatMapSide.csv'], MovHandMapSide);
            csvwrite([obj.Filename '_MovHeatMapFront.csv'], MovHandMapFront);
        end
        
    end
    
end