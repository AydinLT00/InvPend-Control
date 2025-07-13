m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

b = 1; % pendulum up (b=1)

A = [0 1 0 0;
    0 -d/M b*m*g/M 0;
    0 0 0 1;
    0 -b*d/(M*L) -b*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; b*1/(M*L)];

x = out.u_out.Data;
t = out.u_out.Time;
for k=1:2:length(t)
    drawpend(x(k,:),m,M,L);
end