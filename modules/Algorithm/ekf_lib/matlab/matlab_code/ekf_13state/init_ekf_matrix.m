function [Be,P,X,Q,R]=init_ekf_matrix()
Be=[1;0;0];%local mageetic unit vector
P=diag([100;100;100;100;100;100;10;10;10;10;100;100;100]);%initial position variance (m^2),initial2 velocity variance (m/s)^2,initial quaternion variance,initial gyro bias variance (rad/s)^2
X=zeros(13,1);
X(6)=1;%earth gravity
Q=diag([1e-17;1e-17;1e-17;3.5e-2;3.5e-2;5e-1;6e-6;6e-6;6e-6])%gyro noise variance,accelerometer noise variance,gyro bias random walk variance 
R=diag([1e-3;1e-3;3.5e-3;0.08;0.08;5e-1;5e-1;5e-1])%noise:Pgps_x,Pgps_y,Pbaro_z,Vgps_x,Vgps_y,Mx,My,Mz