Ts = 0.01; 
ITER = 1e3;
t=linspace(-pi,pi,ITER); 
x = 8*sin(t).^3; y = 8*sin((2*t)).^3;
vx  = gradient(x, Ts); vy = gradient(y, Ts);
ax  = gradient(vx, Ts); ay = gradient(vy, Ts);
phi = atan2(vy, vx); 

%resulting forward kinematic velocities
x_f = zeros(1, ITER); y_f = zeros(1, ITER); 

delta_f = 0; 

for n=1:ITER-1
    J = [cos(delta_f)*cos(phi(n)) 0; 
         cos(delta_f)*sin(phi(n)) 0;
         0 1];
    J_inv = pinv(J);
    
    X = [vx(n); vy(n); phi(n)];
    B = J_inv*X; %chassis velocities
    
    %forward kinematic
    
    x_f(n+1) = x_f(n) + Ts*B(1)*cos(phi(n)+ B(2)*Ts);
    y_f(n+1) = y_f(n) + Ts*B(1)*sin(phi(n)+B(2)*Ts); 


end
hold on 
plot(x, y, 'linewidth', 2)
plot(x_f, y_f, 'g-','linewidth', 4)
hold off
