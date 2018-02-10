clear all,clc
close all
load('97 11-8-2017 7-06-15 PM.bin-22032.mat')

range = length(OF(:,1));
us_pos = OF(:,4:6)';
us_new = OF(:,7)';
time_us = OF(:,2)';

opt_flow = OF(:,8:9)';
opt_gyro = OF(:,10:11)';
yaw_angle = OF(:,12)';
% k_pos = OF(:,13:15)';
% plot(us_pos(:,1),'g');
% hold on
% grid on
% plot(k_pos(:,1),'r');
eval_pos = zeros(3,range);
eval_vel = zeros(3,range);
dx = zeros(1,range);
dy = zeros(1,range);
v_opt_rot = zeros(2,1);
v_opt = zeros(2,1);
scaleX = 1;
scaleY = 1;
dx(1) = 0;
for i = 2: range-100
    [eval_pos(:,i),eval_vel(:,i)] = multirate_kalman_v4(us_pos(:,i),us_new(i),opt_flow(:,i),opt_gyro(:,i),-yaw_angle(i));
%     eval_pos(:,i) = multirate_kalman_3state(us_pos(:,i),us_new(i+1),opt_flow(:,i),opt_gyro(:,i),-yaw_angle(i));     
    v_opt(1) = (opt_flow(2,i)*scaleX-opt_gyro(2,i))*(us_pos(3,i));%[m/s]
    v_opt(2) = (opt_flow(1,i)*scaleY-opt_gyro(1,i))*(us_pos(3,i));%[m/s]
    v_opt_rot(1) = v_opt(1)*cos(yaw_angle(i))-v_opt(2)*sin(yaw_angle(i));
    v_opt_rot(2) = v_opt(1)*sin(yaw_angle(i))+v_opt(2)*cos(yaw_angle(i));
    dx(i) = dx(i-1) + v_opt_rot(1)*(1/100);
    dy(i) = dy(i-1) + v_opt_rot(2)*(1/100);
    
%     [eval_pos(:,i),eval_vel(:,i)] = LPF_pos(us_pos(:,i),us_new(i),0,[0.8;0.8;0.5],0);
end


%%
figure(1)
plot(us_pos(1,1:range),'LineWidth', 1.5)
hold on
grid on
plot(eval_pos(1,1:range),'LineWidth', 1.5)
plot(eval_vel(1,1:range),'LineWidth', 1.5)
plot(dx(1:range),'LineWidth', 1.5)
% plot(eval_pos(3,1:range),'LineWidth', 1.5)
% plot(yaw_angle(1:range), 'LineWidth', 1.5)

figure(2)
plot(us_pos(2,1:range),'LineWidth', 1.5)
hold on
grid on
plot(eval_pos(2,1:range),'LineWidth', 1.5)
plot(eval_vel(2,1:range),'LineWidth', 1.5)
plot(dy(1:range),'LineWidth', 1.5)
% plot(eval_pos(3,1:range),'LineWidth', 1.5)
% % plot(yaw_angle(1:range), 'LineWidth', 1.5)
% 
figure(3)
plot(us_pos(3,1:range),'LineWidth', 1.5)
hold on
grid on
plot(eval_pos(3,1:range),'LineWidth', 1.5)


%% Plotxy

% figure(4)
% plot(eval_pos(1,6000:9400)*100, eval_pos(2,6000:9400)*100)
