%% Cylinder (0.25 0.474 0.15)
clc; clear;
load('cylinder_0.25_0.474_0.15/data.txt')
stepSize = data(:,1);
time = data(:,2);
distance = data(:,3);
nodes = data(:,4);
labels = ["0.1","","","","0.5","","","","","1.0","","","","","1.5","","","","","2.0","","","","","2.5","","","","","3.0","","","","","3.5","","","","","4.0"];

% figure('Name', 'Cylinder (0.25, 0.474, 0.15): time as a function of step size.')
% boxplot(time, stepSize, 'Labels', labels)
% xlabel('Step size [rad]')
% ylabel('Time [s]')
% figure('Name', 'Cylinder (0.25, 0.474, 0.15): nodes as a function of step size.')
% boxplot(nodes, stepSize)
% xtickformat
% xlabel('Step size [rad]')
% ylabel('Nodes')
% figure('Name', 'Cylinder (0.25, 0.474, 0.15): path length as a function of step size.')
% boxplot(distance, stepSize)
% xlabel('Step size [rad]')
% ylabel('Path length [rad]')

figure('Name', 'Cylinder at (0.25, 0.474, 0.15)')
subplot(3, 1, 1)
boxplot(time, stepSize, 'Labels', labels)
xlabel('Step size [rad]')
ylabel('Time [s]')
title('Time as a function of step size.')
subplot(3, 1, 2)
boxplot(nodes, stepSize, 'Labels', labels)
xlabel('Step size [rad]')
ylabel('Nodes')
title('Nodes as a function of step size.')
subplot(3, 1, 3)
boxplot(distance, stepSize, 'Labels', labels)
xlabel('Step size [rad]')
ylabel('Path length [rad]')
title('Path length as a function of step size.')

%% Cylinder (-0.25 0.474 0.15)
clc; clear;
load('cylinder_-0.25_0.474_0.15/data.txt')
stepSize = data(:,1);
time = data(:,2);
distance = data(:,3);
nodes = data(:,4);
labels = ["0.1","","","","0.5","","","","","1.0","","","","","1.5","","","","","2.0","","","","","2.5","","","","","3.0","","","","","3.5","","","","","4.0"];

% figure('Name', 'Cylinder (-0.25, 0.474, 0.15): time as a function of step size.')
% boxplot(time, stepSize)
% xlabel('Step size [rad]')
% ylabel('Time [s]')
% figure('Name', 'Cylinder (-0.25, 0.474, 0.15): nodes as a function of step size.')
% boxplot(nodes, stepSize)
% xlabel('Step size [rad]')
% ylabel('Nodes')
% figure('Name', 'Cylinder (-0.25, 0.474, 0.15): path length as a function of step size.')
% boxplot(distance, stepSize)
% xlabel('Step size [rad]')
% ylabel('Path length [rad]')

figure('Name', 'Cylinder at (-0.25, 0.474, 0.15)')
subplot(3, 1, 1)
boxplot(time, stepSize, 'Labels', labels)
xlabel('Step size [rad]')
ylabel('Time [s]')
title('Time as a function of step size.')
subplot(3, 1, 2)
boxplot(nodes, stepSize, 'Labels', labels)
xlabel('Step size [rad]')
ylabel('Nodes')
title('Nodes as a function of step size.')
subplot(3, 1, 3)
boxplot(distance, stepSize, 'Labels', labels)
xlabel('Step size [rad]')
ylabel('Path length [rad]')
title('Path length as a function of step size.')

%% Cylinder (0.0 0.474 0.15)
clc; clear;
load('cylinder_0.0_0.474_0.15/data.txt')
stepSize = data(:,1);
time = data(:,2);
distance = data(:,3);
nodes = data(:,4);
labels = ["0.1","","","","0.5","","","","","1.0","","","","","1.5","","","","","2.0","","","","","2.5","","","","","3.0","","","","","3.5","","","","","4.0"];

% figure('Name', 'Cylinder (0.0, 0.474, 0.15): time as a function of step size.')
% boxplot(time, stepSize)
% xlabel('Step size [rad]')
% ylabel('Time [s]')
% figure('Name', 'Cylinder (0.0, 0.474, 0.15): nodes as a function of step size.')
% boxplot(nodes, stepSize)
% xlabel('Step size [rad]')
% ylabel('Nodes')
% figure('Name', 'Cylinder (0.0, 0.474, 0.15): path length as a function of step size.')
% boxplot(distance, stepSize)
% xlabel('Step size [rad]')
% ylabel('Path length [rad]')

figure('Name', 'Cylinder at (0.0, 0.474, 0.15)')
subplot(3, 1, 1)
boxplot(time, stepSize, 'Labels', labels)
xlabel('Step size [rad]')
ylabel('Time [s]')
title('Time as a function of step size.')
subplot(3, 1, 2)
boxplot(nodes, stepSize, 'Labels', labels)
xlabel('Step size [rad]')
ylabel('Nodes')
title('Nodes as a function of step size.')
subplot(3, 1, 3)
boxplot(distance, stepSize, 'Labels', labels)
xlabel('Step size [rad]')
ylabel('Path length [rad]')
title('Path length as a function of step size.')