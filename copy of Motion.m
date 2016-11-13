function serialObject=Motion(a,serialObject)
% Initialize communication
     d=dir('*.txt');
     [dx,dx]=sort([d.datenum]);
     lf=d(dx==1).name
  FileInfo = dir(lf);
    TimeStamp = FileInfo.date
    if a==0
        forward=0
        rotation=0
    elseif a==1
        forward=0.5;
        rotation=0
    elseif a==2
        forward=-0.2
        rotation=0
    elseif a==3
        forward=0.2
        rotation=0.01
    elseif a==4
        forward=0.2
        rotation=-0.01
    else
        forward=0
        rotation=0
    end


% Read encoders (provides baseline)
[StartLeftCounts, StartRightCounts] = EncoderSensorsRoomba(serialObject);

%sets forward velocity [m/s] and turning radius [m]
SetFwdVelRadiusRoomba(serialObject, forward, rotation);

encoderCountList = zeros(80, 18);

for i = 1 : 80
    tic
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

  disp([encL, encR, angle, cliff1, cliff2, cliff3, cliff4, bumpR, bumpL, bumpFront, dropL, dropR, ir1, ir2, ir3, ir4, ir5, ir6]);
  stateList(i, :) = [encL, encR, angle, cliff1, cliff2, cliff3, cliff4, bumpR, bumpL, bumpFront, dropL, dropR, ir1, ir2, ir3, ir4, ir5, ir6];
   toc
  FileInfo = dir(lf);
    TimeStamp2 = FileInfo.date
        if cliff1==1 || cliff2==1 ||bumpL==1
            a=3
            serialObject=Motion(a,serialObject);
        elseif cliff1==3 || cliff4==1 ||bumpR==1
            a=4
            serialObject=Motion(a,serialObject);
        elseif bumpFront==1
            a=0
            serialObject=Motion(a,serialObject);
        else
        a = textread(lf)
        TimeStamp=TimeStamp2 
            serialObject=Motion(a,serialObject);
         end
        
  pause(0.01)
 
end

save('roomba_20sec.mat', 'stateList');

% stop the robot (turning radius doesn’t matter, inf is straight )
SetFwdVelRadiusRoomba(serialObject, 0, inf);

[FinishLeftCounts, FinishRightCounts] = EncoderSensorsRoomba(serialObject)
Distance = (0.036*2*pi)*((FinishLeftCounts - StartLeftCounts) + ...
                         (FinishRightCounts - StartRightCounts) )/( 2 *508.8);

% Power down when finished,
% note physical power button is disabled

PowerOffRoomba(serialObject)
fclose(serialObject)