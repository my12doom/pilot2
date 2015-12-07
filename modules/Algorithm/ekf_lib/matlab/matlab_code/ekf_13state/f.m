function Xresult = f(X,U)
ax = U(4);
ay = U(5);
az = U(6); % NO BIAS STATES ON ACCELS
wx = U(1) - X(11);
wy = U(2) - X(12);
wz = U(3) - X(13); % subtract the biases on gyros
q0 = X(7);
q1 = X(8);
q2 = X(9);
q3 = X(10);
Xresult=zeros(13,1);
Xresult(1) = X(4);
Xresult(2) = X(5);
Xresult(3) = X(6);

% Vdot = Reb*a
Xresult(4) =(q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * ax + 2.0 * (q1 * q2 -q0 * q3) *ay + 2.0 * (q1 * q3 + q0 * q2) * az;
Xresult(5) =2.0 * (q1 * q2 + q0 * q3) * ax + (q0 * q0 - q1 * q1 + q2 * q2 -q3 * q3) * ay + 2 * (q2 * q3 -q0 * q1) *az;
Xresult(6) =2.0 * (q1 * q3 - q0 * q2) * ax + 2 * (q2 * q3 + q0 * q1) * ay +(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az + 9.89;%earth gravity

% qdot = Q*w
Xresult(7)  = (-q1 * wx - q2 * wy - q3 * wz) / 2.0;
Xresult(8)  = (q0 * wx - q3 * wy + q2 * wz) / 2.0;
Xresult(9)  = (q3 * wx + q0 * wy - q1 * wz) / 2.0;
Xresult(10)  = (-q2 * wx + q1 * wy + q0 * wz) / 2.0;

% best guess is that bias stays constant
Xresult(11) =0;
Xresult(12) =0;
Xresult(13) = 0;


