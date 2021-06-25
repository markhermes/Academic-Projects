function [sweep]=SweepCalc(x,y,theta,GOAL,Param)

Angle=atan2((GOAL.Y-y),(GOAL.X-x));

ZoneAngRad=Angle-theta;

ZoneAng=ZoneAngRad*180/pi;  %Convert zone angle to degrees for ease of use

%Correct for full rotations of the model during locomotion
if ZoneAng<0
    ZoneAng=mod(ZoneAng,-360);
    ZoneAng=360+ZoneAng;
else
    ZoneAng=mod(ZoneAng,360);
end

%Determine which zone the model should operate in
if  ZoneAng<60  %Zone 1
    Angle=ZoneAng;
    %Correct for current zone angle limits
    if Angle>60         
        Angle=60;
    end
    %Interpolate to select new seep value
    newSweep=interp1(Param.AngOfTrans,Param.AngInput,Angle);
    sweepdeg=[newSweep 30 30];  %Amplitude of leg sweep [Deg] ([Leg1 Leg2 Leg3])
    anti=[1 1 -1];       %Anti-phase modifier
    
elseif ZoneAng>=60 && ZoneAng<120   %Zone 2
    Angle=120-ZoneAng;
    %Correct for current zone angle limits
    if Angle>60
        Angle=60;
    end
    %Interpolate to select new seep value
    newSweep=interp1(Param.AngOfTrans,Param.AngInput,Angle);
    sweepdeg=[30 newSweep  30];  %Amplitude of leg sweep [Deg] ([Leg1 Leg2 Leg3])
    anti=[-1 -1 1];       %Anti-phase modifier
    
elseif ZoneAng>=120 && ZoneAng<180  %Zone 3
    Angle=ZoneAng-120;
    %Correct for current zone angle limits
    if Angle>60
        Angle=60;
    end
    %Interpolate to select new seep value
    newSweep=interp1(Param.AngOfTrans,Param.AngInput,Angle);
    sweepdeg=[30 newSweep  30];  %Amplitude of leg sweep [Deg] ([Leg1 Leg2 Leg3])
    anti=[-1 1 1];       %Anti-phase modifier
    
elseif ZoneAng>=180 && ZoneAng<240  %Zone 4
    Angle=240-ZoneAng;
    %Correct for current zone angle limits
    if Angle>60
        Angle=60;
    end
    %Interpolate to select new seep value
    newSweep=interp1(Param.AngOfTrans,Param.AngInput,Angle);
    sweepdeg=[30 30 newSweep];  %Amplitude of leg sweep [Deg] ([Leg1 Leg2 Leg3])
    anti=[1 -1 -1];       %Anti-phase modifier
    %disp('4')
elseif ZoneAng>=240 && ZoneAng<300  %Zone 5
    Angle=ZoneAng-240;
    %Correct for current zone angle limits
    if Angle>60
        Angle=60;
    end
    %Interpolate to select new seep value
    newSweep=interp1(Param.AngOfTrans,Param.AngInput,Angle);
    sweepdeg=[30 30 newSweep];  %Amplitude of leg sweep [Deg] ([Leg1 Leg2 Leg3])
    anti=[1 -1 1];       %Anti-phase modifier
    
else                                %Zone 6
    Angle=360-ZoneAng;
    %Correct for current zone angle limits
    if Angle>60
        Angle=60;
    end
    %Interpolate to select new seep value
    newSweep=interp1(Param.AngOfTrans,Param.AngInput,Angle);
    sweepdeg=[newSweep 30 30];  %Amplitude of leg sweep [Deg] ([Leg1 Leg2 Leg3])
    anti=[-1 1 -1];       %Anti-phase modifier
    
end

sweep=sweepdeg*pi/180;

sweep=sweep.*anti;

end