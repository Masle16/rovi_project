%% Iteration analysis
clc; clear;
format long
x = 50000:10000:90000;

% load
load('iterations.txt');

iter_mu = zeros(5,4);
iter_mu(1,:) = mean(iterations(1:30,:));
iter_mu(2,:) = mean(iterations(31:60,:));
iter_mu(3,:) = mean(iterations(61:90,:));
iter_mu(4,:) = mean(iterations(91:120,:));
iter_mu(5,:) = mean(iterations(121:150,:));

iter_std = zeros(5,4);
iter_std(1,:) = std(iterations(1:30,:));
iter_std(2,:) = std(iterations(31:60,:));
iter_std(3,:) = std(iterations(61:90,:));
iter_std(4,:) = std(iterations(91:120,:));
iter_std(5,:) = std(iterations(121:150,:));

% plot
figure('name', 'Iteration analysis')

subplot(3,1,1)
y = iter_mu(:,2);
err = iter_std(:,2);
errorbar(x,y,err);
title('Time as a function of iterations.')
ylabel('Execution time [ms]')
xlabel('Iterations')

subplot(3,1,2);
y = iter_mu(:,3);
err = iter_std(:,3);
errorbar(x,y,err);
title('Angle difference as a function of iterations.')
ylabel('Angle difference [m]')
xlabel('Iterations')

subplot(3,1,3)
y = iter_mu(:,4);
err = iter_std(:,4);
errorbar(x,y,err);
title('Position difference as a function of iterations')
ylabel('Position difference [m]')
xlabel('Iterations')

%% similarity analysis
clc; clear;
format long
x = 0.9:-0.05:0.65;
% iterations
load('similarities.txt');

sim_mu = zeros(6,4);
sim_mu(1,:) = mean(similarities(1:30,:));
sim_mu(2,:) = mean(similarities(31:60,:));
sim_mu(3,:) = mean(similarities(61:90,:));
sim_mu(4,:) = mean(similarities(91:120,:));
sim_mu(5,:) = mean(similarities(121:150,:));
sim_mu(6,:) = mean(similarities(151:180,:));

sim_std = zeros(6,4);
sim_std(1,:) = std(similarities(1:30,:));
sim_std(2,:) = std(similarities(31:60,:));
sim_std(3,:) = std(similarities(61:90,:));
sim_std(4,:) = std(similarities(91:120,:));
sim_std(5,:) = std(similarities(121:150,:));
sim_std(6,:) = std(similarities(151:180,:));

% plot
figure('name', 'Similarity analysis')

subplot(3,1,1)
y = sim_mu(:,2);
err = sim_std(:,2);
errorbar(x,y,err);
title('Time as a function of similarities.')
ylabel('Execution time [ms]')
xlabel('similarities')

subplot(3,1,2);
y = sim_mu(:,3);
err = sim_std(:,3);
errorbar(x,y,err);
title('Angle difference as a function of similarities.')
ylabel('Angle difference [m]')
xlabel('similarities')

subplot(3,1,3)
y = sim_mu(:,4);
err = sim_std(:,4);
errorbar(x,y,err);
title('Position difference as a function of similarities')
ylabel('Position difference [m]')
xlabel('similarities')

%% noise analysis
clc; clear;
format long

% load data
load('data.txt');

noise = data(:,1);
times = data(:,2);
angles = data(:,3);
positions = data(:,4);

% plot data
figure('name', 'Noise analysis')
subplot(3,1,1)
boxplot(times, noise)
title('Noise as a function of time.')
ylabel('Execution time[ms].')
xlabel('Standard deviation of normal distribution.')

subplot(3,1,2)
boxplot(angles, noise)
title('Noise as a function of angle difference.')
ylabel('Angle difference[Deg].')
xlabel('Standard deviation of normal distribution.')

subplot(3,1,3)
boxplot(positions, noise)
title('Noise as a function of position difference.')
ylabel('Position difference[m].')
xlabel('Standard deviation of normal distribution.')

%% Descriptive statistics
times_mu = mean(times(1:180,:))
times_std = std(times(1:180,:))

angles_mu = mean(angles(1:180,:))
angles_std = std(angles(1:180,:))

positions_mu = mean(positions(1:180,:))
positions_std = std(positions(1:180,:))