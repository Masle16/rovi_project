%% Simulated depth sensor analysis
clc; clear;

n = 30;

%% Table, world and scanner
load('transforms.txt');
tablePos = transforms(1,1:3);
tableRpy = transforms(1,4:6);
tableT = [rpy2r(tableRpy) tablePos'; 0 0 0 1];

worldPos = transforms(2,1:3);
worldRpy = transforms(2,4:6);
worldT = [rpy2r(worldRpy) worldPos'; 0 0 0 1];

scannerPos = transforms(3,1:3);
scannerRpy = transforms(3,4:6);
scannerT = [rpy2r(scannerRpy) scannerPos'; 0 0 0 1];

%% Real poses
realPoses = load('../scanner25D_point_clouds/random_picking_position/data.txt');
realPos = realPoses(:,1:3);
realRpy = realPoses(:,4:6);

%% Estimates poses
% load data
estimatedPoses = load('data.txt');
noise = estimatedPoses(:,1);
time = estimatedPoses(:,2);
globalPos = estimatedPoses(:,3:5);
globalRpy = estimatedPoses(:,6:8);
localPos = estimatedPoses(:,9:11);
localRpy = estimatedPoses(:,12:14);

%% Plot
% time
%boxplot(time, noise)

% position
realT = [rpy2r(realRpy(1,:)) realPos(1,:)'; 0 0 0 1];
globalT = [rpy2r(globalRpy(1,:)) globalPos(1,:)'; 0 0 0 1];
localT = [rpy2r(localRpy(1,:)) localPos(1,:)'; 0 0 0 1];
realT = realT * tableT * worldT * scannerT
estimateT = globalT * localT

% for i = 1:n
%     
% end
