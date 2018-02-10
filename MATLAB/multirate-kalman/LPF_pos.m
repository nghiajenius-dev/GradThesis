% function [k_pos, inno] = LPF_pos(ips_pos,ips_flag,delay_ms,max_inno,last_timeout)
function [k_pos, k_vel] = LPF_pos(ips_pos,ips_flag,delay_ms,max_inno,last_timeout)
%%Init parameter
KALMAN_L1 = 0.5;
KALMAN_L2 = 4;
MAX_TIMEOUT = 0.2;  %[second]
dt = 0.01;
% LPF_vel_k = 0.09516;    %10hz

% max_inno = 0.1; %m
persistent x_pred v_pred x_post v_post v_post_
% Init value
if isempty(x_pred)
    % x_est=[Sx,Sy,Sz,Vx,Vy,Vz]'
    x_post = ips_pos;
    v_post = zeros(3, 1);
%     v_post_ = zeros(3, 1);
    v_pred = zeros(3, 1);
end

%% LFP
if(ips_flag == 1)
    
    x_pred = x_post + dt * v_pred;
    x_new = ips_pos + v_pred*delay_ms/1000;
    v_pred = v_post;
    inno = x_new - x_pred;
    for i = 1:1:3
        if abs(inno(i)) < max_inno(i)
            x_post(i) = x_pred(i) + KALMAN_L1 * inno(i);
            v_post(i) = v_pred(i) + KALMAN_L2 * inno(i);
        else
            x_post(i) = x_pred(i);
            v_post(i) = 0;
        end
    end
%     % Apply LPF for raw vel
%     v_post = v_post_ +  LPF_vel_k * (v_post - v_post_);  
%     v_post_ = v_post;
  
    k_pos = x_post;
    k_vel = v_post;
    
else
    inno = zeros(3,1);
    if(last_timeout > MAX_TIMEOUT)
        v_post = zeros(3,1);
    end
    x_pred = x_post + dt * v_pred;
    v_pred = v_post;
    x_post = x_pred;
    v_post = v_pred;
%     % Apply LPF for raw vel
%     v_post = v_post_ +  LPF_vel_k * (v_post - v_post_);  
%     v_post_ = v_post;
    
    k_pos = x_post;  
    k_vel = v_post;  
end
end