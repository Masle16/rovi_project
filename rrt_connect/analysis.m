%% Cylinder (0.25 0.474 0.15)
clc; clear;
load('cylinder_0.25/data.txt')
stepSize = data(:,1);
time = data(:,2);
distance = data(:,3);
nodes = data(:,4);

figure('Name', 'Cylinder at (0.25, 0.474, 0.15)')
subplot(3, 1, 1)
% boxplot(time, stepSize, 'Labels', labels)
boxplot(time, stepSize)
xlabel('Step size [rad]')
ylabel('Time [s]')
title('Time as a function of step size.')
subplot(3, 1, 2)
% boxplot(nodes, stepSize, 'Labels', labels)
boxplot(nodes, stepSize)
xlabel('Step size [rad]')
ylabel('Nodes')
title('Nodes as a function of step size.')
subplot(3, 1, 3)
% boxplot(distance, stepSize, 'Labels', labels)
boxplot(distance, stepSize)
xlabel('Step size [rad]')
ylabel('Path length [rad]')
title('Path length as a function of step size.')

disp('Descriptive statistics for cylinder at 0.25 with stepsize of 0.6')
time_mu = mean(time(551:600,:))
time_std = std(time(551:600,:))

dist_mu = mean(distance(551:600,:))
dist_std = std(distance(551:600,:))

nodes_mu = mean(nodes(551:600,:))
nodes_std = std(nodes(551:600,:))

%% Cylinder (-0.25 0.474 0.15)
clc; clear;
load('cylinder_-0.25/data.txt')
stepSize = data(:,1);
time = data(:,2);
distance = data(:,3);
nodes = data(:,4);

figure('Name', 'Cylinder at (-0.25, 0.474, 0.15)')
subplot(3, 1, 1)
% boxplot(time, stepSize, 'Labels', labels)
boxplot(time, stepSize)
xlabel('Step size [rad]')
ylabel('Time [s]')
title('Time as a function of step size.')
subplot(3, 1, 2)
% boxplot(nodes, stepSize, 'Labels', labels)
boxplot(nodes, stepSize)
xlabel('Step size [rad]')
ylabel('Nodes')
title('Nodes as a function of step size.')
subplot(3, 1, 3)
% boxplot(distance, stepSize, 'Labels', labels)
boxplot(distance, stepSize)
xlabel('Step size [rad]')
ylabel('Path length [rad]')
title('Path length as a function of step size.')

disp('Descriptive statistics for cylinder at 0.25 with stepsize of 0.6')
time_mu = mean(time(551:600,:))
time_std = std(time(551:600,:))

dist_mu = mean(distance(551:600,:))
dist_std = std(distance(551:600,:))

nodes_mu = mean(nodes(551:600,:))
nodes_std = std(nodes(551:600,:))

%% Cylinder (0.0 0.474 0.15)
clc; clear;
load('cylinder_0.0/data.txt')
stepSize = data(:,1);
time = data(:,2);
distance = data(:,3);
nodes = data(:,4);

figure('Name', 'Cylinder at (0.0, 0.474, 0.15)')
subplot(3, 1, 1)
% boxplot(time, stepSize, 'Labels', labels)
boxplot(time, stepSize)
xlabel('Step size [rad]')
ylabel('Time [s]')
title('Time as a function of step size.')
subplot(3, 1, 2)
% boxplot(nodes, stepSize, 'Labels', labels)
boxplot(nodes, stepSize)
xlabel('Step size [rad]')
ylabel('Nodes')
title('Nodes as a function of step size.')
subplot(3, 1, 3)
% boxplot(distance, stepSize, 'Labels', labels)
boxplot(distance, stepSize)
xlabel('Step size [rad]')
ylabel('Path length [rad]')
title('Path length as a function of step size.')

disp('Descriptive statistics for cylinder at 0.25 with stepsize of 0.6')
time_mu = mean(time(551:600,:))
time_std = std(time(551:600,:))

dist_mu = mean(distance(551:600,:))
dist_std = std(distance(551:600,:))

nodes_mu = mean(nodes(551:600,:))
nodes_std = std(nodes(551:600,:))

%% Trajectory plot
clc; clear;

% load data
data = load('forward_kinematics.txt');
[time, xyz, eul] = split_data(data);
eul = avoid_singularities(eul);

figure('Name', 'X, y and z.')
plot(time, xyz(:,1))
hold on
plot(time, xyz(:,2))
plot(time, xyz(:,3))
ylabel('Position [m]')
xlabel('Time [s]')
legend('x', 'y', 'z')
hold off

figure('name', 'Roll, pitch and yaw.')
plot(time, eul(:,1))
hold on
plot(time, eul(:,2))
plot(time, eul(:,3))
ylabel('Orientation [rad]')
xlabel('Time [s]')
legend('Roll', 'Pitch', 'Yaw')
hold off

%% velocity plot
plot(time(1:end-1), abs(diff(xyz(:,1))./1))

% function
function [t, xyz, eul] = split_data(input)
t = input(:,1);
xyz = input(:,2:4);

rows1 = input(:,5:7);
rows2 = input(:,8:10);
rows3 = input(:,11:13);

rot_vec = reshape([rows1(:) rows2(:) rows3(:)]', [], 3);
rot_matrices = reshape(rot_vec', 3, 3, []);
rot = permute(rot_matrices, [2 1 3]);

eul = rotm2eul(rot);
end

function eul = avoid_singularities(input)
    for i = 1:(length(input)-1)
        
        % yaw
        cur_yaw = input(i,3);
        next_yaw = input(i+1,3);
        if sign(cur_yaw) ~= sign(next_yaw)
            diff = abs(cur_yaw - next_yaw);
            if diff > 1
                if sign(cur_yaw) == -1
                    input(i,3) = (2*pi) + cur_roll;
                elseif sign(next_yaw) == -1
                    input(i+1,3) = (2*pi) + next_yaw;
                end
            end
        end
    end
    eul = input;
end