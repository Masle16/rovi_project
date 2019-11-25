clc; clear;

%% load data
n=30;
% Table, world and scanner
load('transforms.txt');
tablePos = transforms(1,1:3);
tableRpy = transforms(1,4:6);
tableT = [eul2rotm(tableRpy) tablePos'; 0 0 0 1];

worldPos = transforms(2,1:3);
worldRpy = transforms(2,4:6);
worldT = [eul2rotm(worldRpy) worldPos'; 0 0 0 1];

scannerPos = transforms(3,1:3);
scannerRpy = transforms(3,4:6);
scannerT = [eul2rotm(scannerRpy) scannerPos'; 0 0 0 1];

% Real poses
realPoses = load('../scanner25D_point_clouds/random_picking_position/data.txt');
realPos = realPoses(:,1:3);
realRpy = realPoses(:,4:6);

% Estimates poses
estimatedPoses = load('data.txt');
time = estimatedPoses(:,1);
globalPos = estimatedPoses(:,2:4);
globalRpy = estimatedPoses(:,5:7);
localPos = estimatedPoses(:,8:10);
localRpy = estimatedPoses(:,11:13);

%% Plot data
figure('Name', 'ANALYSIS')
% Time
subplot(3,1,1)
boxplot(time)

% Position and angle
diff_pos = zeros(n,1);
diff_angle = zeros(n,1);
for i = 1:n
    % real pose
    realT = [eul2rotm(realRpy(i,:)) realPos(i,:)'; 0 0 0 1];
    
    % estimated pose
    globalT = [eul2rotm(globalRpy(i,:)) globalPos(i,:)'; 0 0 0 1];
    localT = [eul2rotm(localRpy(i,:)) localPos(i,:)'; 0 0 0 1];
    pose = globalT * localT;
    
    % worldT * tableT * realT = worldT * scannerT * globalT * localT
    % realT = tableT^(-1) * scannerT * globalT * localT
    estimateT = inv(tableT) * scannerT * pose;
    
    % difference in position
    estimatePos = estimateT(1:3,4)';
    diff = abs(realPos(i,:) - estimatePos);
    diff_pos(i,:) = diff(1) + diff(2) + diff(3);
    
    % difference in angle
    P = eul2rotm(realRpy(i,:));
    Q = estimateT(1:3,1:3);
    R = P * Q';
    diff_angle(i,:) = acos( (trace(R)-1)/2 );
end

subplot(3,1,2)
boxplot(diff_pos)

subplot(3,1,3)
boxplot(diff_angle)