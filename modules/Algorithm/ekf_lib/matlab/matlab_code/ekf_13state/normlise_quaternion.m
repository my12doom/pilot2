function X=normlise_quaternion(X)
    norm = sqrt(X(7)^2+X(8)^2+X(9)^2+X(10)^2);
    X(7)=X(7)/norm;
    X(8)=X(8)/norm;
    X(9)=X(9)/norm;
    X(10)=X(10)/norm;
    