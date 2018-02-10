% load('fusion_raw_full.mat')
clear all, clc
ips_pos = zeros(3,1);
ips_flag= int16(0);
opt_flow = zeros(2,1);
opt_gyro = zeros(2,1);
yaw_angle = 1.23;
delay_ms = 20;
max_inno = zeros(3,1); 
last_timeout = 0.2;
codegen  -config:lib -report -c multirate_kalman_v4.m -args {ips_pos,ips_flag,opt_flow,opt_gyro,yaw_angle}
% codegen  -config:lib -report -c multirate_kalman_v3.m -args {ips_pos,ips_flag}
% LPF_pos(ips_pos,ips_flag,delay_ms,max_inno,last_timeout)
codegen  -config:lib -report -c LPF_pos.m -args {ips_pos,ips_flag,delay_ms,max_inno,last_timeout}