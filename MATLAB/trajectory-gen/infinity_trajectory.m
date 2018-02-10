% REF
% https://gamedev.stackexchange.com/questions/43691/how-can-i-move-an-object-in-an-infinity-or-figure-8-trajectory
function    [x,y,vx,vy,ax,ay]  = infinity_trajectory(center,r,w,t,init_phase,type)
alpha = w*t-init_phase;
%     Wrap 0->2pi
if alpha > 2*pi
    alpha = alpha - 2*pi;
elseif alpha < 0
    alpha = alpha + 2*pi;
end
if type == 1        %  lemniscate of Bernoulli
    x = r * cos(alpha) / (1+sin(alpha)^2);
    y = x * sin(alpha);
    vx = (r*w*sin(alpha)*(sin(alpha)^2 - 3))/(sin(alpha)^2 + 1)^2;
    vy = -(r*w*(3*sin(alpha)^2 - 1))/(sin(alpha)^2 + 1)^2;
    ax = (r*w^2*cos(alpha)*(10*cos(alpha)^2 + cos(alpha)^4 - 8))/(cos(alpha)^2 - 2)^3;
    ay = (r*w^2*(14*sin(2*alpha) + 3*sin(4*alpha)))/(4*(cos(alpha)^2 - 2)^3);
elseif type == 2    %  lemniscate of Gerono
    x = r*cos(alpha);
    y = x*sin(alpha);
    vx = -r*w*sin(alpha);
    vy = r*w*cos(2*alpha);
    ax = -r*w^2*cos(alpha);
    ay = -2*r*w^2*sin(2*alpha);
elseif type == 3
    r = r/2;
    if(alpha < pi)
        x = -r + r*cos(alpha*2);
        y = r*sin(alpha*2);
        vx = -r*w*sin(alpha*2);
        vy = r*w*cos(alpha*2);
        ax = -r*w^2*cos(alpha*2);
        ay = -r*w^2*sin(alpha*2);
    else
        x = r - r*cos(alpha*2);
        y = r*sin(alpha*2);
        vx = r*w*sin(alpha*2);
        vy = r*w*cos(alpha*2);
        ax = r*w^2*cos(alpha*2);
        ay = -r*w^2*sin(alpha*2);
    end
end
% Center offset
x = x + center(1);
y = y + center(2);
end