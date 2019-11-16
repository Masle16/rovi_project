%% Simulated depth sensor analysis
clc; clear;

% load data
n = 30;
time = load('data/times.txt');
estimations = load('data/pose_estimations.txt');
real_poses = load('data/real_poses.txt');

% get positions
position = real_poses(1:3,4)';
position_hat = zeros(n,3);
j = 1:4:120;
for i = 1:n
    position_hat(i,:) = estimations(j(i):j(i)+2,4)';
end

% get rotations
rotation = real_poses(1:3,1:3);
rotation_hat = zeros(n*3,3);
k = 1:3:90;
for i = 1:n
    rotation_hat(k(i):k(i)+2,1:3) = estimations(j(i):j(i)+2,1:3);
end

%% time
time_std_dev = std(time)
time_mean = mean(time)

%% position
diff_position = zeros(n,3);
for i = 1:n
    diff_position(i,:) = abs(position_hat(i,:) - position);
end
position_mean = mean(diff_position)
position_std_dev = std(diff_position)

%% rotation
angle = zeros(n,3);
for i = 1:n
    R = rotation * rotation_hat(k(i):k(i)+2,1:3)';
    angle(i) = acos((trace(R)-1)/2);
end
angle_mean = mean(angle)
angle_std_dev = std(angle)