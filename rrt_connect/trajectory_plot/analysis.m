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

figure('Name', 'X, y and z.')
plot(x)
hold on
plot(y)
plot(z)
ylabel('Position [m]')
xlabel('Configurations')
legend('x', 'y', 'z')
hold off

figure('name', 'Roll, pitch and yaw.')
plot(roll)
hold on
plot(pitch)
plot(yaw)
ylabel('Angle [rad]')
xlabel('Configurations')
legend('Roll', 'Pitch', 'Yaw')
hold off