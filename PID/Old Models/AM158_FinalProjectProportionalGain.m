close all

slow = 10;

[t,y] = ode45(@odefun,[0 30],[-1; 0; -pi/2]);

plot(y(:,1),y(:,2))
hold on
plot(-cos(t/slow),sin(t/slow),'-')
hold off

function dydt = odefun(t,y)

slow = 10;

xr = -cos(t/slow);
yr = sin(t/slow);
thetar = atan2((1/slow)*cos(t/slow),(1/slow)*sin(t/slow));
kp_v = [.8 .8 0];
if xr <= 0
    wxfeed = -1;
else
    wxfeed = 1;
end
if yr >= 0
    wyfeed = 1;
else
    wyfeed = -1;
end
kp_w = [wxfeed wyfeed .5];
y(3) = wrapToPi(y(3));
v = kp_v*[abs(xr-y(1)); abs(yr-y(2)); thetar-y(3)];
w = kp_w*[xr-y(1); yr-y(2); thetar-y(3)];
dydt = [v*cos(y(3)); v*sin(y(3)); w];
end