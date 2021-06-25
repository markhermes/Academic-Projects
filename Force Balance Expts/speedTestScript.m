clear all;

epsilon = 5E-5;
stdErr = 10;
nSec = 0;
data = [];
forceValsMat = [];
stdErrVec = [];


%Create daq session,
s = daq.createSession('ni');
addAnalogInputChannel(s,'Dev1',0,'Voltage');
addAnalogInputChannel(s,'Dev1',1,'Voltage');
addAnalogInputChannel(s,'Dev1',2,'Voltage');
addAnalogInputChannel(s,'Dev1',3,'Voltage');
addAnalogInputChannel(s,'Dev1',4,'Voltage');
addAnalogInputChannel(s,'Dev1',5,'Voltage');
addAnalogInputChannel(s,'Dev1',6,'Voltage');

%Read in offset data: sea star_height_orientation
% speedVec = {'1.0','1.5','2.0','2.5','2.75','2.25','1.75','1.35','1.15','1.0','0.0'};

Re = '00';
AR = input('What AR we at, yo? (HIGH,MED,LOW)','s');
height = input('What height are we at, yo? (05mm,20mm)','s');
angle = input('What angle we at (for matrix rotation), yo?','s');
trial = strcat(Re,'_',AR,'_',height,'_speedTest');
offSetData = csvread(strcat(trial,'_offsets'));
offSets = offSetData(1,:);
load matrixVals;

%Read in experiment values
timeLength = 1;
s.Rate = 5000;
s.DurationInSeconds = timeLength;


plotPt = 1;
while (stdErr > epsilon)
    
    [voltVals,time] = s.startForeground;
    buttonData = voltVals(:,7);
    %Offset the data once finished and multiply by weights
    voltVals = voltVals(:,1:6) - ones(timeLength*s.Rate,1)*offSets;
    forceVals = matrixVals*voltVals';
    forceVals = [forceVals'];
    forceValsMat = [forceValsMat; forceVals];
    avgVoltVals = mean(voltVals);
    avgForceVals = matrixVals*avgVoltVals';
    
    %Plotting
    hold on;
    theta = str2num(angle)*pi/180;
    plotVals = [cos(theta) sin(theta);...
        -sin(theta) cos(theta)] * avgForceVals(1:2);
    subplot(2,1,1)
    hold on
    plot(plotPt,plotVals(1),'*k');
    plot(plotPt,plotVals(2),'*b');
    subplot(2,1,2)
    plot(plotPt,avgForceVals(3),'*g');
    plotPt = plotPt+1;

    %Check stdErr for y-axis
    nSec = nSec+1
    data = [data; forceVals(:,3)];
    zStd = std(data);
    stdErr = zStd/sqrt(nSec*5000)
    stdErrVec = [stdErrVec stdErr];
    
end
%%%
%Get file info so i dont mess it up
Re = input('what speed and rot?(.125_36deg)','s');
trial0 = strcat(AR,'_',height,'_speedTest');
trial = strcat(Re,'_',trial0,'_data');
timefile = strcat(Re,'_',trial0,'_time');

%Writingfiles
forceValsMat(:,1:2) = ([cos(theta) sin(theta);...
        -sin(theta) cos(theta)] * forceValsMat(:,1:2)')';
dlmwrite(trial,forceValsMat,'-append');
dlmwrite(timefile,clock,'-append');
dlmwrite('stdErr',stdErrVec,'-append');

mean(forceValsMat)

clear trial trial0
