function [X P]=INS_Correction(Mag_data,Pos,Vel,X,R,P,Be)
q0=X(7);
q1=X(8);
q2=X(9);
q3=X(10);
Z=zeros(8,1);

Z(1)=Pos(1);
Z(2)=Pos(2);
Z(3)=Pos(3);
Z(4)=Vel(1);
Z(5)=Vel(2);

% %% do lots of things to remove megnetic Z value

q_now=[q0;q1;q2;q3];
Me=body2ned(q_now,Mag_data);
Bnorm=sqrt(Me(1)^2+Me(2)^2);
Me_x=Me(1)/Bnorm;
Me_y=Me(2)/Bnorm;
Me=[Me_x;Me_y;0];
Mb=ned2body(q_now,Me);

%%






Z(6)=Mb(1);
Z(7)=Mb(2);
Z(8)=Mb(3);

H=LinearizeH(X,Be);
Y=h(X,Be);
[X P]=SerialUpdate(H,R,Z,Y,P,X);
X=normlise_quaternion(X);




