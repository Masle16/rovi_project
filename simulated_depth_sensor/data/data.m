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

% Estimates poses
estimatedPoses = load('data.txt');
time = estimatedPoses(:,1);
poses = estimatedPoses(:,2:end);

%% Plot data
figure('Name', 'ANALYSIS')
% Time
subplot(3,1,1)
boxplot(time)

% Position and angle
diff_pos = zeros(n,1);
diff_angle = zeros(n,1);
for i = 1:30
    % real pose
    real = [
        eul2rotm(realPoses(i,4:6)) realPoses(i,1:3)';
        0 0 0 1
    ]
    
    % estimated pose
    pose = [
        poses(i,1:4);
        poses(i,5) poses(i,6) poses(i,7) poses(i,8);
        poses(i,9) poses(i,10) poses(i,11) poses(i,12);
        poses(i,13) poses(i,14) poses(i,15) poses(i,16)
    ];
    
    % worldT * tableT * realT = worldT * scannerT * globalT * localT
    % realT = tableT^(-1) * scannerT * globalT * localT
    estimateT = inv(tableT) * scannerT * pose;
    
    
    % difference in position
    estimatePos = estimateT(1:3,4);
    realPos = real(1:3,4);
    diff = sum(abs(realPos - estimatePos));
    diff_pos(i,:) = diff;
    
    % difference in angle
    P = [
        real(1,1) real(1,2) real(1,3);
        real(2,1) real(2,2) real(2,3);
        real(3,1) real(3,2) real(3,3);
    ];
    Q = [
        estimateT(1,1) estimateT(1,2) estimateT(1,3);
        estimateT(2,1) estimateT(2,2) estimateT(2,3);
        estimateT(3,1) estimateT(3,2) estimateT(3,3);
    ];
    R = P * Q';
    diff_angle(i,:) = acos( (trace(R)-1)/2 );
end

subplot(3,1,2)
boxplot(diff_pos)

subplot(3,1,3)
boxplot(diff_angle)