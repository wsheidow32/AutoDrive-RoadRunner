function dxdt = StateFunctionVehicle(x,u)

 m   = 2000;   
 J  = 4000;   
 lf  = 1.4;   
 lr  = 1.6;    
 Cf  = 12e3;  
 Cr  = 11e3;   
 
X = x(1);
Y = x(2);
psi = x(3);
vx = x(4);
vy = x(5);
w = x(6);

ax = u(1);
delta = u(2);

betaf = atan2(vy + lf*w, vx) - delta;
betar = atan2(vy - lr*w, vx);

Fyf = -Cf * betaf * cos(delta);
Fyr = -Cr * betar;

X_dot = vx * cos(psi) - vy * sin(psi);
Y_dot = vx * sin(psi) + vy * cos(psi);
psi_dot = w;
vx_dot = vy * w + ax;
vy_dot = -vx * w + (2/m) * (Fyf + Fyr);
w_dot = (2/J) * (lf * Fyf - lr * Fyr);

dxdt = [X_dot; Y_dot; psi_dot; vx_dot; vy_dot; w_dot];

end


