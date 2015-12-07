function [F G]=LinearFG(X,U)
ax=U(4);ay=U(5);az=U(6);wx=U(1);wy=U(2);wz=U(3);
q0=X(7);q1=X(8);q2=X(9);q3=X(10);
F=zeros(13,13);
F(1:3,4:6)=eye(3);
Fvq=[2.0 * (q0 * ax - q3 * ay + q2 * az),2.0 * (q1 * ax + q2 * ay + q3 * az),2.0 * (-q2 * ax + q1 * ay + q0 * az),2.0 * (-q3 * ax - q0 * ay + q1 * az);
     2.0 * (q3 * ax + q0 * ay - q1 * az),2.0 * (q2 * ax - q1 * ay - q0 * az),2.0 * (q1 * ax + q2 * ay + q3 * az),2.0 * (q0 * ax - q3 * ay + q2 * az);
     2.0 * (-q2 * ax + q1 * ay + q0 * az),2.0* (q3 * ax + q0 * ay - q1 * az),2.0 * (-q0 * ax + q3 * ay - q2 * az),2.0 * (q1 * ax + q2 * ay + q3 * az)];
F(4:6,7:10)=Fvq;
Fqq=[0,-wx / 2.0,-wy / 2.0,-wz / 2.0;
    wx / 2.0,0,wz / 2.0,-wy / 2.0;
    wy / 2.0,-wz / 2.0,0,wx / 2.0;
    wz / 2.0,wy / 2.0,-wx / 2.0,0];
F(7:10,7:10)=Fqq;
Fqb=[q1 / 2.0,q2 / 2.0,q3 / 2.0;
    -q0 / 2.0,q3 / 2.0,-q2 / 2.0;
    -q3 / 2.0,-q0 / 2.0,q1 / 2.0;
     q2 / 2.0,-q1 / 2.0,-q0 / 2.0];
F(7:10,11:13)=Fqb;
%caculate G
Reb=[
    % dVdot/dna  - NO BIAS STATES ON ACCELS - S0 REPEAT FOR G HERE
    -q0 * q0 - q1 * q1 + q2 * q2 + q3 * q3,2.0 * (-q1 * q2 + q0 * q3),-2.0 * (q1 * q3 + q0 * q2);
    -2.0 * (q1 * q2 + q0 * q3),-q0 * q0 + q1 * q1 - q2 * q2 + q3 * q3,2.0 * (-q2 * q3 + q0 * q1);
    2.0 * (-q1 * q3 + q0 * q2),-2.0 * (q2 * q3 + q0 * q1),-q0 * q0 + q1 * q1 + q2 * q2 - q3 * q3];
Rw=[
    % dqdot/dnw
    q1 / 2.0,q2 / 2.0,q3 / 2.0;
    -q0 / 2.0,q3 / 2.0,-q2 / 2.0;
    -q3 / 2.0,-q0 / 2.0,q1 / 2.0;
    q2 / 2.0, -q1 / 2.0,-q0 / 2.0];
G=zeros(13,9);
G(4:6,4:6)=Reb;
G(7:10,1:3)=Rw;
G(11:13,7:9)=eye(3);

