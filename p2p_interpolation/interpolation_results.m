clc
clear
close all

js_blend = importdata('jointspace_trajectory.txt');
js_no_blend = importdata('jointspace_trajectory_nb.txt');
% ts_no_blend = importdata('toolspace_trajectory_nb.txt');
time_steps_all = importdata('time_steps.txt');
time_steps = time_steps_all(1:end-1);

% The path to follow:
on_pick = [-0.45 0.474 0.19 0 0 3.138];
above_pick = [-0.2 0.474 0.39 0 0 3.138];
intermediate1 = [0.3 0.17 0.5 -1.5 -1 4];
intermediate2 = [0.55 -0.25 0.7 -1 -0.5 3.5];
above_place = [0.075 -0.5 0.39 0 0 3.138];
on_place = [0.075 -0.5 0.19 0 0 3.138];

p_xyz = [on_pick(1:3); above_pick(1:3); intermediate1(1:3); intermediate2(1:3); above_place(1:3); on_place(1:3)];
p_rot = [on_pick(4:6); above_pick(4:6); intermediate1(4:6); intermediate2(4:6); above_place(4:6); on_place(4:6)];

%% Joint space with blend
[js_t, js_xyz, js_eul] = split_data(js_blend);
js_eul = avoid_singularities(js_eul);

figure('NumberTitle', 'off', 'Name', 'Position - Joint space interpolation with blend.');
hold on
plot(js_t, js_xyz(:,1))
plot(js_t, js_xyz(:,2))
plot(js_t, js_xyz(:,3))
% The linear path between the points
%scatter([0; time_steps_all], p_xyz(:,1),'MarkerEdgeColor',[0 0.4470 0.7410])
%scatter([0; time_steps_all], p_xyz(:,2),'MarkerEdgeColor',[0.8500 0.3250 0.0980])
%scatter([0; time_steps_all], p_xyz(:,3),'MarkerEdgeColor',[0.9290 0.6940 0.1250])

% Markers for points
%scatter(time_steps, js_xyz(time_steps.*10+1,1),'k')
%scatter(time_steps, js_xyz(time_steps.*10+1,2),'k')
%scatter(time_steps, js_xyz(time_steps.*10+1,3),'k')
%title('Position - Joint space interpolation with blend.')
legend('X', 'Y', 'Z', 'Location', 'northeast')
xlabel('Time [s]')
ylabel('Position [m]')
box on

figure('NumberTitle', 'off', 'Name', 'Rotation - Joint space interpolation with blend.');
hold on
plot(js_t, js_eul(:,1))
plot(js_t, js_eul(:,2))
plot(js_t, js_eul(:,3))
% The linear path between the points
%scatter([0; time_steps_all], p_rot(:,1),'MarkerEdgeColor',[0 0.4470 0.7410])
%scatter([0; time_steps_all], p_rot(:,2),'MarkerEdgeColor',[0.8500 0.3250 0.0980])
%scatter([0; time_steps_all], p_rot(:,3),'MarkerEdgeColor',[0.9290 0.6940 0.1250])

%scatter(time_steps, js_eul(time_steps.*10+1,1),'k')
%scatter(time_steps, js_eul(time_steps.*10+1,2),'k')
%scatter(time_steps, js_eul(time_steps.*10+1,3),'k')
%title('Rotation - Joint space interpolation with blend.')
legend('R', 'P', 'Y', 'Location', 'northeast')
xlabel('Time [s]')
ylabel('Rotation [rad]')
box on

%% Joint space without blend
[js_t, js_xyz, js_eul] = split_data(js_no_blend);
js_eul = avoid_singularities(js_eul);

figure('NumberTitle', 'off', 'Name', 'Position - Joint space interpolation without blend.');
hold on
plot(js_t, js_xyz(:,1))
plot(js_t, js_xyz(:,2))
plot(js_t, js_xyz(:,3))
% The linear path between the points
%scatter([0; time_steps_all], p_xyz(:,1),'MarkerEdgeColor',[0 0.4470 0.7410])
%scatter([0; time_steps_all], p_xyz(:,2),'MarkerEdgeColor',[0.8500 0.3250 0.0980])
%scatter([0; time_steps_all], p_xyz(:,3),'MarkerEdgeColor',[0.9290 0.6940 0.1250])

%scatter(time_steps, js_xyz(time_steps.*10+1,1),'k')
%scatter(time_steps, js_xyz(time_steps.*10+1,2),'k')
%scatter(time_steps, js_xyz(time_steps.*10+1,3),'k')
%title('Position - Joint space interpolation without blend.')
legend('X', 'Y', 'Z', 'Location', 'northeast')
xlabel('Time [s]')
ylabel('Position [m]')
box on

figure('NumberTitle', 'off', 'Name', 'Rotation - Joint space interpolation without blend.');
hold on
plot(js_t, js_eul(:,1))
plot(js_t, js_eul(:,2))
plot(js_t, js_eul(:,3))
% The linear path between the points
%scatter([0; time_steps_all], p_rot(:,1),'MarkerEdgeColor',[0 0.4470 0.7410])
%scatter([0; time_steps_all], p_rot(:,2),'MarkerEdgeColor',[0.8500 0.3250 0.0980])
%scatter([0; time_steps_all], p_rot(:,3),'MarkerEdgeColor',[0.9290 0.6940 0.1250])

%scatter(time_steps, js_eul(time_steps.*10+1,1),'k')
%scatter(time_steps, js_eul(time_steps.*10+1,2),'k')
%scatter(time_steps, js_eul(time_steps.*10+1,3),'k')
%title('Rotation - Joint space interpolation without blend.')
legend('R', 'P', 'Y', 'Location', 'northeast')
xlabel('Time [s]')
ylabel('Rotation [rad]')
box on



%% Tool space without blend
[ts_t, ts_xyz, ts_eul] = split_data(ts_no_blend);
ts_eul = avoid_singularities(ts_eul);

figure('NumberTitle', 'off', 'Name', 'Position - Tool space interpolation without blend.');
hold on
plot(ts_t, ts_xyz(:,1))
plot(ts_t, ts_xyz(:,2))
plot(ts_t, ts_xyz(:,3))
% The linear path between the points
scatter([0; time_steps_all], p_xyz(:,1),'MarkerEdgeColor',[0 0.4470 0.7410])
scatter([0; time_steps_all], p_xyz(:,2),'MarkerEdgeColor',[0.8500 0.3250 0.0980])
scatter([0; time_steps_all], p_xyz(:,3),'MarkerEdgeColor',[0.9290 0.6940 0.1250])

%scatter(time_steps, js_xyz(time_steps.*10+1,1),'k')
%scatter(time_steps, js_xyz(time_steps.*10+1,2),'k')
%scatter(time_steps, js_xyz(time_steps.*10+1,3),'k')
%title('Position - Joint space interpolation without blend.')
legend('X', 'Y', 'Z', 'Location', 'northeast')
xlabel('Time [s]')
ylabel('Position [m]')
box on

figure('NumberTitle', 'off', 'Name', 'Rotation - Tool space interpolation without blend.');
hold on
plot(ts_t, ts_eul(:,1))
plot(ts_t, ts_eul(:,2))
plot(ts_t, ts_eul(:,3))
% The linear path between the points
scatter([0; time_steps_all], p_rot(:,1),'MarkerEdgeColor',[0 0.4470 0.7410])
scatter([0; time_steps_all], p_rot(:,2),'MarkerEdgeColor',[0.8500 0.3250 0.0980])
scatter([0; time_steps_all], p_rot(:,3),'MarkerEdgeColor',[0.9290 0.6940 0.1250])

%scatter(time_steps, js_eul(time_steps.*10+1,1),'k')
%scatter(time_steps, js_eul(time_steps.*10+1,2),'k')
%scatter(time_steps, js_eul(time_steps.*10+1,3),'k')
%title('Rotation - Joint space interpolation without blend.')
legend('R', 'P', 'Y', 'Location', 'northeast')
xlabel('Time [s]')
ylabel('Rotation [rad]')
box on

function [t, xyz, eul] = split_data(input)
t = input(:,1);
xyz = input(:,2:4);

rows1 = input(:,5:7);
rows2 = input(:,8:10);
rows3 = input(:,11:13);

rot_vec = reshape([rows1(:) rows2(:) rows3(:)]', [], 3);
rot_matrices = reshape(rot_vec', 3, 3, []);
rot = permute(rot_matrices, [2 1 3]);

eul = tr2eul(rot);
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
                input(i,3) = -1* cur_roll;
            elseif sign(next_yaw) == -1
                input(i+1,3) = -1 * next_yaw;
            end
        end
    end
end
eul = input;
end