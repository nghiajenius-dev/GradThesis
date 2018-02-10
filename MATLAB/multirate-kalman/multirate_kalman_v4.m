function [k_pos, k_vel] = multirate_kalman_v4(ips_pos,ips_flag,opt_flow,opt_gyro,yaw_angle)
%% Parameters
T1 = 1/100;                     % 100Hz -> 10ms
T2 = 1/20;                      % 20Hz  -> 50ms
scaleX = 1;
scaleY = 1;
KALMAN_L1 = 0.5;                % x
KALMAN_L2 = 4;                  % v
MAX_TIMEOUT = 0.2;              % [second]
optical_z_offset = 0.0;         % offset z

LPF_vel_k = 0.09516;    %10hz

A = [1  0  0  T1 0  0;
    0  1  0  0  T1 0;
    0  0  1  0  0  T1;
    0  0  0  1  0  0;
    0  0  0  0  1  0;
    0  0  0  0  0  1];

H = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];
r1p = 10^-2;
r1v = 10^-12;


q1p = 10^-2;
q1v = 10^-12;
q2p = 10^-4;
q2v = 10^-12;

rzp = 10^-2;
rzv = 10^-5;
qzp = 10^-2;
qzv = 10^-5;

R1 = [r1p 0  0  0  0  0;
    0  r1p 0  0  0  0;
    0  0  rzp 0  0  0;
    0  0  0  r1v 0  0;
    0  0  0  0  r1v 0;
    0  0  0  0  0  rzv];

R2 = R1;

Q1 = [q1p 0  0  0  0  0;
    0  q1p 0  0  0  0;
    0  0 qzp  0  0  0;
    0  0  0  q1v 0  0;
    0  0  0  0  q1v 0;
    0  0  0  0  0  qzv];

Q2 = [q2p 0  0  0  0  0;
    0  q2p 0  0  0  0;
    0  0 qzp  0  0  0;
    0  0  0  q2v 0  0;
    0  0  0  0  q2v 0;
    0  0  0  0  0  qzv];

%% KALMAN
v_optical = zeros(2,1);

persistent x_est p_est Vx_ Vy_ Vz_ 
% Init value
if isempty(x_est)
    x_est = zeros(6, 1);        % x_est=[Sx,Sy,Sz,Vx,Vy,Vz]'
%     x_est_ = zeros(6, 1);        % x_est=[Sx,Sy,Sz,Vx,Vy,Vz]'
    
    x_est(1:3) = ips_pos;
    p_est = Q1;
    Vx_ = 0;
    Vy_ = 0;
    Vz_ = 0;
end

% KALMAN
if(ips_flag == 1)
    x_pred = A * x_est;
    p_pred = A * p_est * A' + Q2;
    
    v_optical(1) = (opt_flow(2)*scaleX-opt_gyro(2))*(x_pred(3)-optical_z_offset);%[m/s]
    v_optical(2) = (opt_flow(1)*scaleY-opt_gyro(1))*(x_pred(3)-optical_z_offset);%[m/s]
    Vx = v_optical(1)*cos(yaw_angle)-v_optical(2)*sin(yaw_angle);
    Vy = v_optical(1)*sin(yaw_angle)+v_optical(2)*cos(yaw_angle);
    Vz = (x_pred(3) - x_est(3))/T1;
    
    % Apply LPF for raw vel
    Vx = Vx_ +  LPF_vel_k * (Vx - Vx_);
    Vy = Vy_ +  LPF_vel_k * (Vy - Vy_);   
    Vz = Vz_ +  LPF_vel_k * (Vz - Vz_);
    Vx_ = Vx; 
    Vy_ = Vy;
    Vz_ = Vz;
    
    inno =  [ips_pos;Vx;Vy;Vz]- H*x_pred;
    S = H * p_pred * H'+ R2;
    K = p_pred * H' / S;         %% Kalman gain
    x_est = x_pred + K*inno;    %% new state
    
    p_est = p_pred - K * S * K';     %% new covariance
else    % Measurement update 1 @100Hz
    %   Time update / Predict @ time k
    % x = Ax + Bu + w
    % z = Hx + v
    x_pred = A * x_est;
    p_pred = A * p_est * A' + Q1;
    
    v_optical(1) = (opt_flow(2)-opt_gyro(2))*(x_pred(3)-optical_z_offset);%[m/s]
    v_optical(2) = (opt_flow(1)-opt_gyro(1))*(x_pred(3)-optical_z_offset);%[m/s]
    Vx = v_optical(1)*cos(yaw_angle)-v_optical(2)*sin(yaw_angle);
    Vy = v_optical(1)*sin(yaw_angle)+v_optical(2)*cos(yaw_angle);
    Vz = (x_pred(3) - x_est(3))/T1;
    
    % Apply LPF for raw vel
    Vx = Vx_ +  LPF_vel_k * (Vx - Vx_);
    Vy = Vy_ +  LPF_vel_k * (Vy - Vy_);   
    Vz = Vz_ +  LPF_vel_k * (Vz - Vz_);
    Vx_ = Vx; 
    Vy_ = Vy;
    Vz_ = Vz;
    
    inno =[x_pred(1:3);Vx;Vy;Vz]- H*x_pred;
    S = H * p_pred * H'+ R1;
    K = p_pred * H' / S;         %% Kalman gain
    x_est = x_pred + K*inno;    %% new state
    
    p_est = p_pred - K * S * K';     %% new covariance   
end  
k_pos = x_est(1:3);
k_vel = x_est(4:6);
end