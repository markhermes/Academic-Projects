%Create daq session, 
s = daq.createSession('ni');
addAnalogInputChannel(s,'Dev1',0,'Voltage');
addAnalogInputChannel(s,'Dev1',1,'Voltage');
addAnalogInputChannel(s,'Dev1',2,'Voltage');
addAnalogInputChannel(s,'Dev1',3,'Voltage');
addAnalogInputChannel(s,'Dev1',4,'Voltage');
addAnalogInputChannel(s,'Dev1',5,'Voltage');

%Get offsets for current trial
%Select rate and duration for bias averaging
s.Rate = 5000;
s.DurationInSeconds = 60;
[bias,time] = s.startForeground;
for i = 1:6
    offsets(1,i) = mean(bias(:,i));
    offsets(2,i) = std(bias(:,i))/sqrt(60*1000);    
end

%Write offsets to file, name is star, lift, and orientation
AR = input('What AR,yo?','s');
height = input('What height are we at, yo?','s');
Re = input('What Re we at, yo?','s');
trial = strcat(Re,'_',AR,'_',height,'_speedTest_offsets');
csvwrite(trial,offsets);
