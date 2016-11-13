function serialObject=Motion(a,serialObject)


for i = 1 : 10
  % Initialize communication
     d=dir('*.txt');
     [dx,dx]=sort([d.datenum]);
     lf=d(dx==1).name;
  FileInfo = dir(lf);
    TimeStamp = FileInfo.date;
    if a==0
        forward=0
        rotation=0
    elseif a==1
        forward=0.1;
        rotation=0
    elseif a==2
        forward=-0.1
        rotation=0
    elseif a==3
        forward=0.1
        rotation=0.01
    elseif a==4
        forward=0.1
        rotation=-0.01
    else
        forward=0
        rotation=0
    end


% Read encoders (provides baseline)
[StartLeftCounts, StartRightCounts] = EncoderSensorsRoomba(serialObject);

%sets forward velocity [m/s] and turning radius [m]
SetFwdVelRadiusRoomba(serialObject, forward, rotation);
pause(1)
SetFwdVelRadiusRoomba(serialObject, 0, 0);
encoderCountList = zeros(80, 18);


  [encL, encR] = EncoderSensorsRoomba(serialObject);
  angle = AngleSensorRoomba(serialObject);
  cliff = CliffSignalStrengthRoomba(serialObject);
  cliff1 = cliff(1);
  cliff2 = cliff(2);
  cliff3 = cliff(3);
  cliff4 = cliff(4);
  [bumpR, bumpL, bumpFront, dropL, dropR] = BumpsWheelDropsSensorsRoomba(serialObject);
  ir = RangeSignalStrengthRoomba(serialObject);
  ir1 = ir(1);
  ir2 = ir(2);
  ir3 = ir(3);
  ir4 = ir(4);
  ir5 = ir(5);
  ir6 = ir(6);

  disp([encL, encR, angle, cliff1, cliff2, cliff3, cliff4, bumpR, bumpL, bumpFront, dropL, dropR, ir1, ir2, ir3, ir4, ir5, ir6])
  stateList(i, :) = [encL, encR, angle, cliff1, cliff2, cliff3, cliff4, bumpR, bumpL, bumpFront, dropL, dropR, ir1, ir2, ir3, ir4, ir5, ir6];
    %-----------Kevin-------------------------- 
%%begin running rover
%%rover data input into MATLAB


    %%Real Time Distance Calculations
        if i > 1
        stateList(1,1) = 0;
        stateList(1,2) = 0;
        left_wheel = stateList(i,1) - stateList(i-1,1);
        right_wheel = stateList(i,2) - stateList(i-1,2);
    
        distance = (0.036*2*3.1415926*(left_wheel+right_wheel)/(2*508.8));
    
        Distance(i) = distance;

    
    %% Real time angle calculations
        step_angle_change = stateList(i,3) * 0.324956;
        MStep_Angle_Change(i) = step_angle_change;
        if i == 1;
            MCummulative_Angle_Change(i) = MStep_Angle_Change(i);  
        else 
            MCummulative_Angle_Change(i) = MCummulative_Angle_Change(i-1) + MStep_Angle_Change(i); 
        end 
    
    %% Real time velocity of each wheel
        if i > 1
        stateList(1,1) = 0;
        stateList(1,2) = 0;
        left_wheel = stateList(i,1) - stateList(i-1,1);
        right_wheel = stateList(i,2) - stateList(i-1,2);
    
        left_wheel_distance = (0.036*2*3.1415926*(left_wheel)/(508.8));
        right_wheel_distance = (0.036*2*3.1415926*(right_wheel)/(508.8));
    
        left_wheel_velocity = left_wheel_distance / 0.5;
        right_wheel_velocity = right_wheel_distance / 0.5;
    
        MLeft_Wheel_Velocity(i) = left_wheel_velocity;
        MRight_Wheel_Velocity(i) = right_wheel_velocity;

    else 
        MLeft_Wheel_Velocity(1) = 0;
        MRight_Wheel_Velocity(1) = 0;
    end 
    
    %%Real time encoder counter
    if i > 1
        left_count = stateList(i,1) - stateList(i-1,1);
        right_count = stateList(i,2) - stateList(i-1,2);
    else 
        left_count = 0;
        right_count = 0;
    end 
    
    MLeft_count(i) = left_count;
    MRight_count(i) = right_count;
    
    Index(i) = i;
    
    %% Movement Tracking calculations
    if i > 1
        cos_angle_theta = cos((MCummulative_Angle_Change(i) + MCummulative_Angle_Change(i-1))/2);
        sin_angle_theta = sin((MCummulative_Angle_Change(i) + MCummulative_Angle_Change(i-1))/2);
        
        d_x = Distance(i) * cos_angle_theta;
        d_y = Distance(i) * sin_angle_theta;
    
        Md_x(i) = Md_x(i-1) + d_x;
        Md_y(i) = Md_y(i-1) + d_y;
    else 
        Md_x(1) = 0;
        Md_y(1) = 0;
    end 
    
    

figure
plot(Index,Distance);
title('Distance moved by rover every 0.5s');

figure
plot(Index, MStep_Angle_Change);
title('Angle change per 0.5 seconds');

figure
plot(Index,MCummulative_Angle_Change);
title('Cummulative angle change');

figure
plot(Index, MLeft_Wheel_Velocity);
title('Left wheel velocity');

figure
plot(Index, MRight_Wheel_Velocity);
title('right wheel velocity');

figure
plot(Index, MRight_count);
title('Right wheel encoder count');

figure
plot(Index,MLeft_count);
title('left wheel encoder count');

figure
plot(Md_x, Md_y); 
title('Real_Time_Movement_Tracking');
   %----------kevin end--------------------------- 
    
    
    
  FileInfo = dir(lf);
    TimeStamp2 = FileInfo.date;
        a = textread(lf)

       
        
  pause(0.25)
 
end

save('roomba_integrate.mat', 'stateList');

% stop the robot (turning radius doesn’t matter, inf is straight )
SetFwdVelRadiusRoomba(serialObject, 0, inf);

[FinishLeftCounts, FinishRightCounts] = EncoderSensorsRoomba(serialObject)
Distance = (0.036*2*pi)*((FinishLeftCounts - StartLeftCounts) + ...
                         (FinishRightCounts - StartRightCounts) )/( 2 *508.8);

% Power down when finished,
% note physical power button is disabled

PowerOffRoomba(serialObject)
fclose(serialObject)