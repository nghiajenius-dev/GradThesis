function    [x,y,vx,vy,ax,ay]  = circle_trajectory(center,r,w,t,init_phase)
alpha = w*t-init_phase;
x = center(1) + r*cos(alpha);
y = center(2) + r*sin(alpha);
vx = -r*w*sin(alpha);
vy = r*w*cos(alpha);
ax = -r*w^2*cos(alpha);
ay = -r*w^2*sin(alpha);
end