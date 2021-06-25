%% Grid Search
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

%Create daq session,
arduinoObj.s = daq.createSession('ni');
addAnalogInputChannel(arduinoObj.s,'Dev1',0,'Voltage');
addAnalogInputChannel(arduinoObj.s,'Dev1',1,'Voltage');
addAnalogInputChannel(arduinoObj.s,'Dev1',2,'Voltage');
addAnalogInputChannel(arduinoObj.s,'Dev1',3,'Voltage');
addAnalogInputChannel(arduinoObj.s,'Dev1',4,'Voltage');
addAnalogInputChannel(arduinoObj.s,'Dev1',5,'Voltage');

%% Parameters
numGrid = 4;
arduinoObj.thetaOff = -8;
Pm = 0.2;%proability of mutation (small allows faster convergence)
SRange = [0 arduinoObj.Slimit];%height range corresponding to vertical actuator
LRange = [0 arduinoObj.Llimit];%volume range corresponding to volume actuator
RRange = [0 36] + arduinoObj.thetaOff; %So that the range is 10 deg from arm straight in flow
nElite = 2;%number of "bests" selected for each iteration
threshold = 0.05;%convergence criteria - amount of variance in fitness


%% Go to first position

arduinoObj.start = 1;
currentPos = arduinoObj.currentPos;

LSpace = linspace(LRange(1),LRange(2),numGrid);
SSpace = linspace(SRange(1),SRange(2),numGrid);
RSpace = linspace(RRange(1),RRange(2),numGrid);

for j = 1:numGrid
    for i = 1:numGrid
        startPos = (j-1)*numGrid^2 + (i-1)*numGrid + 1;
        positions = startPos:startPos+numGrid-1;
        P(positions,1) = LSpace;
        P(positions,2) = SSpace(i)*ones(numGrid,1);
        P(positions,3) = RSpace(j)*ones(numGrid,1);
    end
end

P(:,1:2) = arduinoObj.Kbp.*P(:,1:2);

Rstr = strcat('R',string(P(arduinoObj.start,3)));
    
%return to update globalValue
arduinoObj.currentPos = [0,0,P(arduinoObj.start,3)]
    
%Actuate to new position
fprintf(arduinoObj.ard,'%s',Rstr);
exitLoop(arduinoObj.ard);


%% Get offsets
arduinoObj.s.Rate = 1000;
arduinoObj.s.DurationInSeconds = 30;
[offSetMat,time] = arduinoObj.s.startForeground; %30000 x 6 matrix
arduinoObj.offSets = mean(offSetMat);

'Turn On Da Channel'

pause(180); %Wait 3 mins



%% Setup
pause(5); %Pause allow arduino to begin
[P,popFitness,varFitnessVec] = mainLoop(numGrid, Pm, SRange, LRange, RRange, nElite, threshold, arduinoObj);
xlabel('Number of loops for N=10');
ylabel('Fitness');

%% Main
function [P,popFitness,varFitnessVec] = mainLoop(N, Pm, SRange, LRange,RRange, nElite, threshold, arduinoObj)

[P,arduinoObj.currentPos,evalMat] = gridSearch(N, SRange, LRange,RRange, arduinoObj);
%Fitness function is used to select which objective function we are
%interested
% P = fitness(P,evalMat);
% popFitness(1) = mean(P(:,4));
% 
% %Stopping criteria will be based on variance
% varPopFitness = 10;
% popNum=2;
% while(varPopFitness >= threshold)
%     %Mating functions
%     P = selectPop(P, N, nElite);
%     P = breed(P, N, Pm, SRange, LRange, RRange);
%     %Do load cell business with new population
%     [P,arduinoObj.currentPos,evalMat] = evalNewChildren(P, N, SRange, LRange,RRange, arduinoObj,popNum);
%     P = fitness(P,evalMat);
%     %Checking how good we are in population terms
%     popFitness(popNum) = mean(P(:,4));
%     if mod(popNum,1)==0 %Skip data points maybe
%         figure(2)
%         hold on
%         plot(ones(1,N).*popNum,P(:,4),'*k');
%         plot(popFitness,'or');
%         title('Population fitness');
%         pause(0.01)
%         figure(3)
%         hold on
%         plot3(mean(P(:,1)),mean(P(:,2)),mean(P(:,3)),'*k')
%         view([1 1 1])
%     end
%     dlmwrite('fitness',P(:,4),'-append')
%     %     %Exit condition
%     %     if(i>10) %Wait for 10 populations to start evaluating
%     %         varPopFitness = var(popFitness(i:i-9));
%     %         varFitnessVec(i) = varPopFitness;
%     %     end
%     popFitness(popNum)
%     popNum=popNum+1;
% end
% P
% popFitness
% end

end

%%
function [P, currentPos, evalMat] = gridSearch(numGrid, SRange, LRange, RRange, arduinoObj)

currentPos = arduinoObj.currentPos;

LSpace = linspace(LRange(1),LRange(2),numGrid);
SSpace = linspace(SRange(1),SRange(2),numGrid);
RSpace = linspace(RRange(1),RRange(2),numGrid);

for j = 1:numGrid
    for i = 1:numGrid
        startPos = (j-1)*numGrid^2 + (i-1)*numGrid + 1;
        positions = startPos:startPos+numGrid-1;
        P(positions,1) = LSpace;
        P(positions,2) = SSpace(i)*ones(numGrid,1);
        P(positions,3) = RSpace(j)*ones(numGrid,1);
    end
end

P(:,1:2) = arduinoObj.Kbp.*P(:,1:2);

for i = arduinoObj.start:arduinoObj.start+15
    %Check which direction to go
    dir = getDirection(currentPos,P(i,:));
    
    %Subtract to get relative increment to absolute position
    Lstr = strcat('L',dir{1},string(abs(P(i,1)-currentPos(1))));
    Sstr = strcat('S',dir{2},string(abs(P(i,2)-currentPos(2))));
    Rstr = strcat('R',string(P(i,3)));
    
    %return to update globalValue
    currentPos = [P(i,1),P(i,2),P(i,3)]
    
    %Actuate to new position
    fprintf(arduinoObj.ard,'%s',Lstr);
    exitLoop(arduinoObj.ard);
    fprintf(arduinoObj.ard,'%s',Sstr);
    exitLoop(arduinoObj.ard);
    fprintf(arduinoObj.ard,'%s',Rstr);
    exitLoop(arduinoObj.ard);
    
    
    %Writingfile - characteristics
    %Read load cell
    fileLoc = 'E:\Mark\Spring 2019\03182019_geneticAlgorithm/gridSearch/';
    dataFileName = strcat(num2str(i));
    avgForceVals = readLoadCell(i,P(i,3),arduinoObj,dataFileName);
    dnaFileName = strcat(fileLoc,dataFileName,'_DNA');
    dlmwrite(dnaFileName,[P(i,:) avgForceVals],'-append');
    
    %Store values in a matrix for easy access
    evalMat(i,:) = [P(i,:) avgForceVals];
    
    if mod(i,8) == 0
        %Delete after debugging
        returnHome(currentPos,arduinoObj);
        currentPos = [0 0 0];
    end
    
end



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
ObjDrag = ((evalMat(:,4).^2 + evalMat(:,5).^2));
ObjLift = evalMat(:,6);
ObjTorque = (evalMat(:,7).^2+evalMat(:,8).^2+evalMat(:,9).^2)./3;

P_new = [P, 1./ObjDrag];

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
function C = breed(M, N, Pm, SRange, LRange, RRange)
prob = 1/N;
probVec = prob*ones(N,1);
probVec = cumsum(probVec);

%Generate N children
for i = 1:N
    %Pick 2 Parents
    pick = rand;
    for j = 1:N
        if pick<= probVec(j)
            P1 = M(i,:);
            break
        end
    end
    pick = rand;
    for j = 1:N
        if pick<= probVec(j)
            P2 = M(i,:);
            break
        end
    end
    
    %With only averaging, the numbers become too consistent,
    %need a mutation to create diversity, want +/- random
    %or could just increase precision...
    %for sure adding the noise makes convergence slower
    
    alpha1 = (rand-0.5)*4000/10;
    alpha2 = (rand-0.5)*800/10;
    alpha3 = (rand-0.5)*abs(RRange(2)-RRange(1))/10;
    C(i,1) = (P1(1)+P2(1))/2 + alpha1;
    C(i,2) = (P1(2)+P2(2))/2 + alpha2;
    C(i,3) = (P1(3)+P2(3))/2 + alpha3;
    
    %Is the child weird and mutated?
    if rand <= Pm
        
        h = rand*(800)+0;
        v = rand*(4000)+0;
        r = rand*abs(RRange(2)-RRange(1))+RRange(1);
        alpha = rand;
        C(i,1) = alpha*C(i,1)+(1-alpha)*v;
        C(i,2) = alpha*C(i,2)+(1-alpha)*h;
        C(i,3) = alpha*C(i,3)+(1-alpha)*r;
        
    end
    
    
end

end

%%
function [P, currentPos, evalMat] = evalNewChildren(P, N, SRange, LRange, RRange, arduinoObj,popNum)

currentPos = arduinoObj.currentPos;

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
    fprintf(arduinoObj.ard,'%s',Lstr);
    exitLoop(arduinoObj.ard);
    fprintf(arduinoObj.ard,'%s',Sstr);
    exitLoop(arduinoObj.ard);
    fprintf(arduinoObj.ard,'%s',Rstr);
    exitLoop(arduinoObj.ard);
    
    
    %Writingfile - characteristics
    %Read load cell
    fileLoc = 'E:\Mark\Spring 2019\03182019_geneticAlgorithm/optimizeDrag/';
    dataFileName = strcat('Population_',num2str(popNum),'_Gene_',num2str(i));
    avgForceVals = readLoadCell(i,P(i,3),arduinoObj,dataFileName,N);
    dnaFileName = strcat(fileLoc,dataFileName,'_DNA');
    dlmwrite(dnaFileName,[P(i,1:2), P(i,3)+arduinoObj.thetaOff, avgForceVals],'-append');
    
    %Store values in a matrix for easy access
    evalMat(i,:) = [P(i,:) avgForceVals];
    
end

%Delete after debugging
returnHome(currentPos,arduinoObj);
currentPos = [0 0 0];

end
%%
function returnHome(currentPos,arduinoObj)

"returning home"

%Subtract to get relative increment to absolute position
Lstr = strcat('L','R',string(currentPos(1)));
Sstr = strcat('S','R',string(currentPos(2)));
Rstr = strcat('R',0);

%Actuate to new position
fprintf(arduinoObj.ard,'%s',Lstr);
exitLoop(arduinoObj.ard);
fprintf(arduinoObj.ard,'%s',Sstr);
exitLoop(arduinoObj.ard);
fprintf(arduinoObj.ard,'%s',Rstr);
exitLoop(arduinoObj.ard);

pause(10);
end

%%
function avgVals = readLoadCell(q,angle,arduinoObj,dataFileName)

load matrixVals;
timeLength = 1;
arduinoObj.s.Rate = 1000;
arduinoObj.s.DurationInSeconds = timeLength;
offSets = arduinoObj.offSets;

for i = 1:30 %%sec of acq
    
    [voltVals,time] = arduinoObj.s.startForeground;
    %Offset the data once finished and multiply by weights
    
    voltVals = voltVals(:,1:6) - ones(timeLength*arduinoObj.s.Rate,1)*offSets;
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
    fileLoc = 'E:\Mark\Spring 2019\03182019_geneticAlgorithm/gridSearch/';
    dataLoadCell = strcat(fileLoc,dataFileName,'_loadCell');
    dlmwrite(dataLoadCell,forceVals,'-append');
    dlmwrite(dataLoadCell,[plotVals' avgForceVals(3) 0 0 0],'-append');
    
    figure(1)
    hold on
    plot(i,plotVals(1),'*k');
    plot(i,plotVals(2),'*r');
    plot(i,avgForceVals(3),'*g');
    plot(i,norm(avgForceVals(1:2)),'^b')
    ylim([-1,1]);
    title('Current population force values');
    pause(0.1);
    
    %Build matrix of averaged values to average once again
    data(i,:) = [plotVals' avgForceVals(3:6)'];
end
clf
avgVals = mean(data);

end