%% Trajectory plot
clc; clear;

% load data
data = load('forward_kinematics.txt');
x = data(:,1);
y = data(:,2);
z = data(:,3);
roll = data(:,4);
pitch = data(:,5);
yaw = data(:,6);

figure('Name', 'Trajectory plots')
h1=subplot(6,1,1)
plot(x)
title('x')
ylabel('[m]')
xlabel('Configuration')

h2=subplot(6,1,2)
plot(y)
title('y')
ylabel('[m]')
xlabel('Configuration')

h3=subplot(6,1,3)
plot(z)
title('z')
ylabel('[m]')
xlabel('Configuration')

h4=subplot(6,1,4)
plot(roll)
title('Roll')
ylabel('[rad]')
xlabel('Configuration')

h5=subplot(6,1,5)
plot(pitch)
title('Pitch')
ylabel('[rad]')
xlabel('Configuration')

h6=subplot(6,1,6)
plot(yaw)
title('Yaw')
ylabel('[rad]')
xlabel('Configuration')