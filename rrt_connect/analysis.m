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
time = data(:,1);
x = data(:,2);
y = data(:,3);
z = data(:,4);
roll = data(:,5);
pitch = data(:,6);
yaw = data(:,7);

figure('Name', 'X, y and z.')
plot(time,x)
hold on
plot(time,y)
plot(time,z)
ylabel('Position [m]')
xlabel('Time [s]')
legend('x', 'y', 'z')
hold off

figure('name', 'Roll, pitch and yaw.')
plot(time,roll)
hold on
plot(time,pitch)
plot(time,yaw)
ylabel('Angle [rad]')
xlabel('Time [s]')
legend('Roll', 'Pitch', 'Yaw')
hold off
