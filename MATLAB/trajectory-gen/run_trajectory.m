clear,clc
close all
r = 45;     %cm
T = 12;      %s
interval = 1/10;        %update interval
w = 2*pi/T;
t = 0;
center = [105 105];     %cm
init_phase = 0;        %rad

figure(1)
plot(center(1), center(2),'*');
hold on
grid minor
traj_line = animatedline('Color','b','LineWidth',4);
pause(3);
for t = 0:interval:T
    axis([0 210 0 210]);
%     [x, y, vx, vy, ax, ay] = circle_trajectory(center,r,w,t,init_phase);    
    [x, y, vx, vy, ax, ay] = infinity_trajectory(center,r,w,t,init_phase,2);
    addpoints(traj_line,x,y);
    quiver(x,y,vx,vy,'Color','g','LineWidth',0.5);
    quiver(x,y,ax,ay,'Color','r','LineWidth',0.5);
    pause(0.01);
%     delete(pts)
end

