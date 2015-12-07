function X=RungeKutta(X,U,dT)
%2nd order runge-kutta implement
Xlast=X;
k1=f(X,U);
X=Xlast+0.5*dT*k1;
k2=f(X+0.5*dT*k1,U);
X=Xlast+0.5*dT*k2;
k3=f(X+0.5*dT*k2,U);
X=Xlast+0.5*dT*k3;
k4=f(X+dT*k3,U);
X=Xlast+dT*(k1+2*k2+2*k3+k4)/6;
% X=Xlast+dT*f(X,U);
X=normlise_quaternion(X);

