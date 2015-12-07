%read raw sensor data
M = csvread('Z:\fusing_gps.csv',2);
[length,line]=size(M);
[gx_zerobias,gy_zerobias,gz_zerobias]=gyro_correction(M);
M(:,2)=M(:,2)-gx_zerobias;M(:,3)=M(:,3)-gy_zerobias;M(:,4)=M(:,4)-gz_zerobias;
[q0,q1,q2,q3]=init_quaternion_by_euler(0,0,-50);
[Be,P,X,Q,R]=init_ekf_matrix();
dT=0.003;
%X = INS_SetState(p_x,p_y,p_z,v_x,v_y,v_z,q0,q1,q2,q3,gyro_x_bias,gyro_y_bias,gyro_z_bias);
X = INS_SetState(M(2,11),M(2,12),M(2,13),M(2,14),M(2,15),0,q0,q1,q2,q3,0,0,0);
%for log result
result = zeros([length,8]);

 for i=1:length-1
    %U:gx gy gz;ax,ay,az
    U=[M(i,2);M(i,3);M(i,4);-M(i,5);-M(i,6);-M(i,7)];
    [F,G]=LinearFG(X,U);
    X=RungeKutta(X,U,dT);
    P=INS_CovariancePrediction(F,G,Q,dT,P);
    Mag_data=[-M(i,8);-M(i,9);M(i,10)];
    Pos=[M(i,11);M(i,12);M(i,16)];
    %[0,0,M(i,16)];%
    Vel=[M(i,14);M(i,15)];

    [X,P]=INS_Correction(Mag_data,Pos,Vel,X,R,P,Be);
    [roll,pitch,yaw]=quaternion_to_euler(0,X(7),X(8),X(9),X(10));
    result(i,1)=roll;
    result(i,2)=pitch;
    result(i,3)=yaw;
    result(i,4)=X(1);
    result(i,5)=X(2);
    result(i,6)=X(13);
    result(i,7)=X(3);
    result(i,8)=X(6);
 end
%plot curve
x_aixs=1:length;
%  plot(M(:,11),M(:,12),'r')
figure(1)
subplot(3,2,1);
plot(x_aixs,result(:,1),'b',x_aixs,M(:,17)*180/pi,'r')
subplot(3,2,2);
plot(x_aixs,result(:,2),'b',x_aixs,M(:,18)*180/pi,'r')
subplot(4,1,3);
plot(x_aixs,result(:,3),'b',x_aixs,M(:,19)*180/pi,'r',x_aixs,result(:,6),'g')
subplot(4,1,4);
%plot(x_aixs,result(:,6),'g')
plot(x_aixs,result(i,8),'y')
figure(2)%x-y position
plot(M(:,11),M(:,12),'b',result(:,4),result(:,5),'r')
figure(3)%altitude
plot(x_aixs,M(:,16),'b',x_aixs,result(:,7),'r')
figure(4)%plot3d track data
plot3(result(:,4),result(:,5),result(:,7),'x')
