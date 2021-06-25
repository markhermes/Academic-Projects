%% Genetic Algorithm
clear all

%% Initialize ardiuno and load cell configurations
arduinoObj.ard = serial('COM6');
fopen(arduinoObj.ard);
arduinoObj.Llimit = 10; %10cm stroke
arduinoObj.Slimit = 2; %3.5cm stroke
arduinoObj.Kbp = 200/0.5; %counts p cm
arduinoObj.Kvbig = 1/0.3937*4.95/100; %cm p in * in p mL = cm/mL
arduinoObj.Kvsmall = 1/0.3937*2.25/10; %cm p in * in p mL = cm/mL
arduinoObj.currentPos = [0 0 0]; %Big syringe
arduinoObj.c = videoinput('gentl');

%Create daq session,
arduinoObj.s = daq.createSession('ni');
addAnalogInputChannel(arduinoObj.s,'Dev1',0,'Voltage');
addAnalogInputChannel(arduinoObj.s,'Dev1',1,'Voltage');
addAnalogInputChannel(arduinoObj.s,'Dev1',2,'Voltage');
addAnalogInputChannel(arduinoObj.s,'Dev1',3,'Voltage');
addAnalogInputChannel(arduinoObj.s,'Dev1',4,'Voltage');
addAnalogInputChannel(arduinoObj.s,'Dev1',5,'Voltage');

%% Parameters1
arduinoObj.thetaOff = -8;

%% Go to first position
arduinoObj.angle = 36 + arduinoObj.thetaOff;
Rstr = strcat('R',num2str(arduinoObj.angle));
    
%return to update globalValue
arduinoObj.currentPos = [0,0,arduinoObj.angle];
    
%Actuate to new position
fprintf(arduinoObj.ard,Rstr);
exitLoop(arduinoObj.ard);

%% Get offsets
arduinoObj.s.Rate = 1000;
arduinoObj.s.DurationInSeconds = 30;
[offSetMat,time] = arduinoObj.s.startForeground; %30000 x 6 matrix
arduinoObj.offsets = mean(offSetMat);
arduinoObj.thetaOff = -8;

dlmwrite('offsets',arduinoObj.offsets,'-append');

'Turn On Da Channel'

pause(0); %Wait 3 mins

%% Parameters2
N = 10;
Pc = 0.9;
Pm = 0.10;%proability of mutation (small allows faster convergence)
SRange = [0 arduinoObj.Slimit];%height range corresponding to vertical actuator
LRange = [0 arduinoObj.Llimit];%volume range corresponding to volume actuator
RRange = [-10 70] + arduinoObj.thetaOff; %So that the range is 10 deg from arm straight in flow
nElite = 2;%number of "bests" selected for each iteration
threshold = 0.05;%convergence criteria - amount of variance in fitness


%% Setup
pause(5); %Pause allow arduino to begin
[P,popFitness] = mainLoop(N, Pc, Pm, SRange, LRange, RRange, nElite, threshold, arduinoObj);
xlabel('Number of loops for N=10');
ylabel('Fitness');
% 
% %Go to 36 deg
% pause(5); %Pause allow arduino to begin
% arduinoObj.angle = 36 + arduinoObj.thetaOff;
% Rstr = strcat('R',num2str(arduinoObj.angle));
%     
% %return to update globalValue
% arduinoObj.currentPos = [0,0,arduinoObj.angle];
%     
% %Actuate to new position
% fprintf(arduinoObj.ard,Rstr);
% exitLoop(arduinoObj.ard);
% 
% close all;
% 
% %Will append on top of other data, but oh well
% [P,popFitness] = mainLoop(N, Pc, Pm, SRange, LRange, RRange, nElite, threshold, arduinoObj);
% xlabel('Number of loops for N=10');
% ylabel('Fitness');

%% Main
function [P,popFitness] = mainLoop(N, Pc, Pm, SRange, LRange,RRange, nElite, threshold, arduinoObj)

[P,arduinoObj.currentPos,evalMat,arduinoObj.offsets] = firstPop(N, SRange, LRange,RRange, arduinoObj);
dlmwrite('offsets',arduinoObj.offsets,'-append');
frame = getsnapshot(arduinoObj.c);

%Fitness function is used to select which objective function we are
%interested
P = fitness(P,evalMat);
popFitness(1) = mean(P(:,4));

%Stopping criteria will be based on variance
varPopFitness = 10;
popNum=2;
while(popNum <= 30)
    %Mating functions
    P = selectPop(P, N, nElite);
    P = breed(P, N, Pc, Pm, SRange, LRange, RRange, arduinoObj);
    %Do load cell business with new population
    [P,arduinoObj.currentPos,evalMat, arduinoObj.offsets] = evalNewChildren(P, N, SRange, LRange,RRange, arduinoObj,popNum);
    dlmwrite('offsets',arduinoObj.offsets,'-append');

    P = fitness(P,evalMat);
    %Checking how good we are in population terms
    popFitness(popNum) = mean(P(:,4));
    if mod(popNum,1)==0 %Skip data points maybe
        figure(2)
        hold on
        plot(ones(1,N).*popNum,P(:,4),'*k');
        plot(popFitness,'or');
        title('Population fitness');
        pause(0.01)
        figure(3)
        hold on
        plot3((P(:,1)),(P(:,2)),(P(:,3)),'*k')
        view([1 1 1])
        pause(0.01)
    end
    dlmwrite('fitness',P(:,4),'-append')
%     %Exit condition
%     if(i>10) %Wait for 10 populations to start evaluating
%         varPopFitness = var(popFitness(i:i-9));
%         varFitnessVec(i) = varPopFitness;
%     end
    popFitness(popNum)
    popNum=popNum+1;
end
P
popFitness
end


%%
function [P, currentPos, evalMat, offsets] = firstPop(N, SRange, LRange, RRange, arduinoObj)

currentPos = arduinoObj.currentPos;

% % Randomized
% P(:,1) = rand(N,1)*(LRange(2)-LRange(1)) + LRange(1)*ones(N,1); %cm
% P(:,2) = rand(N,1)*(SRange(2)-SRange(1)) + SRange(1)*ones(N,1); %cm
% P(:,3) = arduinoObj.angle*ones(N,1); %deg

% Search grid - deterministic
% Shape is R1 - 3, R2 - 4, R3 - 3, where R is S actuator
LVec1 = linspace(0,1,3);
LVec2 = linspace(0,1,4);
SVec = linspace(0,1,3);

LVec = [LVec1 LVec2 LVec1]';
SVec = [SVec(1).*ones(1,3) SVec(2).*ones(1,4) SVec(3).*ones(1,3)]';

P(:,1) = LVec*(LRange(2)-LRange(1)) + LRange(1)*ones(N,1); %cm
P(:,2) = SVec*(SRange(2)-SRange(1)) + SRange(1)*ones(N,1); %cm
P(:,3) = arduinoObj.angle*ones(N,1); %deg

P(:,1:2) = arduinoObj.Kbp.*P(:,1:2);
%Sort by descending for faster actuation
[~,indices] = sort(P(:,1),'descend');
P = P(indices,:);

for i = 1:N
    %Check which direction to go
    dir = getDirection(currentPos,P(i,:));
    
    %Subtract to get relative increment to absolute position
    Lstr = strcat('L',dir{1},string(abs(P(i,1)-currentPos(1))));
    Sstr = strcat('S',dir{2},string(abs(P(i,2)-currentPos(2))));
    Rstr = strcat('R',string(P(i,3)));
    
    %return to update globalValue
    currentPos = [P(i,1),P(i,2),P(i,3)]
    
    %Actuate to new position
    fprintf(arduinoObj.ard,Lstr);
    exitLoop(arduinoObj.ard);
    fprintf(arduinoObj.ard,Sstr);
    exitLoop(arduinoObj.ard);
    fprintf(arduinoObj.ard,Rstr);
    exitLoop(arduinoObj.ard);
    
    
    %Writingfile - characteristics
    %Read load cell
    
    fileLoc = 'E:\Mark\Spring 2019\03182019_geneticAlgorithm/optimizeDrag/';
    dataFileName = strcat('Population_1_Gene_',num2str(i));
    avgForceVals = readLoadCell(i,P(i,3),arduinoObj,dataFileName,N);
    dnaFileName = strcat(fileLoc,dataFileName,'_DNA');
    dlmwrite(dnaFileName,[P(i,:) avgForceVals],'-append');
    
    %Camera snapshot
    frameFileName = strcat(fileLoc,dataFileName,'_Frame');
    frame = getsnapshot(arduinoObj.c);
    figure(4)
    imshow(frame)
    dlmwrite(frameFileName,frame,'-append');
    
    %Store values in a matrix for easy access
    evalMat(i,:) = [P(i,:) avgForceVals];
    
end

%Delete after debugging
offsets = returnHome(currentPos,arduinoObj);
currentPos = [0 0 arduinoObj.angle];

end

function dir = getDirection(currentPos,P)
for q = 1:2
    if (currentPos(q) > P(q))
        dir{q} = 'R';
    elseif (currentPos(q) < P(q))
        dir{q} = 'F';
    else
        dir{q} = '0';
    end
end
end

%Checking to determine when the motor stops running
%"Fin" is sent when this occurs
function exitLoop(ard)
while 1
    fin = fscanf(ard);
    if strcmp(fin,'Fin')
        return
    end
end
end

%%
% %We want the minimizer of the function so fitness = 1/f(x)
% %Also want to normalize contribution: so divide by magnitude
function P_new = fitness(P,evalMat)

%Selecting objective function:
ObjDrag = evalMat(:,5);
ObjLift = evalMat(:,6);
ObjTorque = (evalMat(:,7).^2+evalMat(:,8).^2+evalMat(:,9).^2)./3;

P_new = [P, ObjDrag+abs(min(ObjDrag))+0.01]; %Drag is +/- the offset value.
                                               %0.02 give small chance to
                                               %the weaklings

end

%%
function M = selectPop(P, N, nElite)
%Sorting in order of descending fitness
[~,ind] = sort(P(:,4),'descend');
P = P(ind,:);

%Normalize the probability based on fitness, where small drag/lift =
%greater fitness -> gets greater share of possible numbers
prob = P(:,4)./sum(P(:,4));
prob = cumsum(prob);

%Preselect elites
M = P(1:nElite,:);

%Choose N-elites for the next population
%Because the population is ordered from worst fitness to best, it will
%choose the first best case, obvs repeats can occur, there demonstrating
%the need for diversity
for i = 1:N-nElite
    pick = rand;
    for j = 1:N
        if pick <= prob(j)
            M = [M ; P(j,:)];
            break
        end
    end
end
end

%%
function CNew = breed(M, N, Pc, Pm, SRange, LRange, RRange, arduinoObj)
prob = 1/N;
probVec = prob*ones(N,1);
probVec = cumsum(probVec);

LRange = LRange.*arduinoObj.Kbp; 
SRange = SRange.*arduinoObj.Kbp; 
CNew = [];
%Generate N children
for i = 1:N/2
    %Pick 2 Parents
    pick = rand;
    for j = 1:N
        if pick<= probVec(j)
            P1 = M(j,:);
            break
        end
    end
    pick = rand;
    for j = 1:N
        if pick<= probVec(j)
            P2 = M(j,:);
            break
        end
    end
    
   

P1 = encoding(P1,LRange,SRange);
P2 = encoding(P2,LRange,SRange);
    
pick = rand;
if pick < Pc
    [C] = crossover(P1 ,P2);
else
    C = [P1;P2];
end
 C = mutation(C,Pm);   
 C = decoding(C,LRange,SRange);
 CNew = [CNew;C];
end
CNew = [CNew , M(:,3)];
end

function C = crossover(P1,P2)
%Next step: inject noise in encoding
%Uniform crossover
for i = 1:16
    if rand < 0.5
        C1(i) = P2(i);
        C2(i) = P1(i);
    else
        C1(i) = P1(i);
        C2(i) = P2(i);
    end
C = [C1; C2];
end

% %single point crossover for each coded int  
% %through trial and error found this to generate diverse solutions
% r1 = round(rand*7);
% r2 = round(rand*7);
% 
% newP11 = [P1(1:r1) P2(r1+1:8)];
% newP21 = [P2(1:r1) P1(r1+1:8)];
% 
% newP12 = [P1(9:8+r2) P2(9+r2:16)];
% newP22 = [P2(9:8+r2) P1(9+r2:16)];
% 
% C = [newP11 newP12; newP21 newP22];

% %single point crossover for whole chromosome
% r1 = round(rand*15);
% newP1 = [P1(1:r1) P2(r1+1:16)];
% newP2 = [P2(1:r1) P1(r1+1:16)];
% C = [newP1; newP2];

% % two-point crossover
% r1 = round(rand*16);%crossoversite
% r2 = round(rand*16);%crossoversite
% if(r1>r2)
%     pick1 = r2;
%     pick2 = r1;
% else
%     pick1 = r1;
%     pick2 = r2;
% end
% newP1 = [P1(1:pick1) P2(pick1+1:pick2) P1(pick2+1:end)];
% newP2 = [P2(1:pick1) P1(pick1+1:pick2) P2(pick2+1:end)];
% C = [newP1;newP2];

end

function C = mutation(C,Pm)

for i = 1:numel(C(1,:))
    
    if(rand<= Pm)
        if(strcmp(C(1,i),'1'))
            C(1,i)='0';
        else
            C(1,i)='1';
        end
    end
        
    if(rand<= Pm)
        if(strcmp(C(2,i),'1'))
            C(2,i)='0';
        else
            C(2,i)='1';
        end
    end
end
end

function P =  encoding(P,LRange,SRange)

Pnum1 = (P(1)-LRange(1))*(2^(8)-1)/(LRange(2)-LRange(1));
Pnum2 = (P(2)-SRange(1))*(2^(8)-1)/(SRange(2)-SRange(1));
P1 = dec2bin(Pnum1,8);
P2 = dec2bin(Pnum2,8);
P = [P1 P2];

end

function C = decoding(C,LRange,SRange)

Cdec11 = bin2dec(C(1,1:8));
Cdec12 = bin2dec(C(1,9:16));
Cdec21 = bin2dec(C(2,1:8));
Cdec22 = bin2dec(C(2,9:16));

C11 = Cdec11/(2^8-1)*(LRange(2)-LRange(1)) + LRange(1);
C12 = Cdec12/(2^8-1)*(SRange(2)-SRange(1)) + SRange(1);
C21 = Cdec21/(2^8-1)*(LRange(2)-LRange(1)) + LRange(1);
C22 = Cdec22/(2^8-1)*(SRange(2)-SRange(1)) + SRange(1);
C = [C11 C12; C21 C22];

end

%%
function [P, currentPos, evalMat, offsets] = evalNewChildren(P, N, SRange, LRange, RRange, arduinoObj,popNum)

currentPos = arduinoObj.currentPos;
P(:,3) = arduinoObj.angle*ones(N,1); %deg

for i = 1:N
    %Check which direction to go
    dir = getDirection(currentPos,P(i,:));
    
    %Sort by descending for faster actuation
    [~,indices] = sort(P(:,1),'descend');
    P = P(indices,:);
    
    %Subtract to get relative increment to absolute position
    Lstr = strcat('L',dir{1},string(abs(P(i,1)-currentPos(1))));
    Sstr = strcat('S',dir{2},string(abs(P(i,2)-currentPos(2))));
    Rstr = strcat('R',string(P(i,3)));
    
    %return to update globalValue
    currentPos = [P(i,1),P(i,2),P(i,3)]
    
    %Actuate to new position
    fprintf(arduinoObj.ard,Lstr);
    exitLoop(arduinoObj.ard);
    fprintf(arduinoObj.ard,Sstr);
    exitLoop(arduinoObj.ard);
    fprintf(arduinoObj.ard,Rstr);
    exitLoop(arduinoObj.ard);
    
    
    %Writingfile - characteristics
    %Read load cell
    fileLoc = 'E:\Mark\Spring 2019\03182019_geneticAlgorithm/optimizeDrag/';
    dataFileName = strcat('Population_',num2str(popNum),'_Gene_',num2str(i));
    avgForceVals = readLoadCell(i,P(i,3),arduinoObj,dataFileName,N);
    dnaFileName = strcat(fileLoc,dataFileName,'_DNA');
    dlmwrite(dnaFileName,[P(i,1:2), P(i,3)+arduinoObj.thetaOff, avgForceVals],'-append');
    
    %Camera snapshot
    frameFileName = strcat(fileLoc,dataFileName,'_Frame');
    frame = getsnapshot(arduinoObj.c);
    figure(4)
    imshow(frame)
    dlmwrite(frameFileName,frame,'-append');
    
    %Store values in a matrix for easy access
    evalMat(i,:) = [P(i,:) avgForceVals];
    
end

%Delete after debugging
offsets = returnHome(currentPos,arduinoObj);
currentPos = [0 0 arduinoObj.angle];

end


%%
function offsets = returnHome(currentPos,arduinoObj)

"returning home"


%Subtract to get relative increment to absolute position
Lstr = strcat('L','R',string(currentPos(1)));
Sstr = strcat('S','R',string(currentPos(2)));
Rstr = strcat('R',string(arduinoObj.angle));

%Actuate to new position
fprintf(arduinoObj.ard,Lstr);
exitLoop(arduinoObj.ard);
fprintf(arduinoObj.ard,Sstr);
exitLoop(arduinoObj.ard);
fprintf(arduinoObj.ard,Rstr);
exitLoop(arduinoObj.ard);

offsets = getOffset(arduinoObj);

pause(10);
end

%%
function offsets = getOffset(arduinoObj)

arduinoObj.s.Rate = 1000;
arduinoObj.s.DurationInSeconds = 30;
[offSetMat,time] = arduinoObj.s.startForeground; %30000 x 6 matrix
offsets = mean(offSetMat);

end

%%
function avgVals = readLoadCell(q,angle,arduinoObj,dataFileName,N)

load matrixVals;
timeLength = 1;
arduinoObj.s.Rate = 1000;
arduinoObj.s.DurationInSeconds = timeLength;
offsets = arduinoObj.offsets;



for i = 1:30 %%sec of acq
    
    [voltVals,time] = arduinoObj.s.startForeground;
    %Offset the data once finished and multiply by weights
    
    voltVals = voltVals(:,1:6) - ones(timeLength*arduinoObj.s.Rate,1)*offsets;
    forceVals = matrixVals*voltVals';
    forceVals = forceVals';
    avgVoltVals = mean(voltVals);
    avgForceVals = matrixVals*avgVoltVals';
    
    %Plotting: had to reverse x and y 
    hold on;
    theta = (angle)*pi/180;
    plotVals = [cos(theta) sin(theta);...
        -sin(theta) cos(theta)] * [avgForceVals(1);avgForceVals(2)];
    
    %Writingfiles
    fileLoc = 'E:\Mark\Spring 2019\03182019_geneticAlgorithm/optimizeDrag/';
    dataLoadCell = strcat(fileLoc,dataFileName,'_loadCell');
    dlmwrite(dataLoadCell,forceVals,'-append');
    dlmwrite(dataLoadCell,[plotVals' avgForceVals(3) 0 0 0],'-append');
    
    figure(1)
    hold on
    plot(i,plotVals(1),'*k');
    plot(i,plotVals(2),'*r');
    plot(i,avgForceVals(3),'*g');
    ylim([-1,1]);
    title('Current population force values');
    pause(0.1);
        
    %Build matrix of averaged values to average once again
    data(i,:) = [plotVals' avgForceVals(3:6)'];
end
clf
avgVals = mean(data);

end