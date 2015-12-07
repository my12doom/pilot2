% // *************  CovariancePrediction *************
% // Does the prediction step of the Kalman filter for the covariance matrix
% // Output, Pnew, overwrites P, the input covariance
% // Pnew = (I+F*T)*P*(I+F*T)' + T^2*G*Q*G'
% // Q is the discrete time covariance of process noise
% // Q is vector of the diagonal for a square matrix with
% // dimensions equal to the number of disturbance noise variables
% // The General Method is very inefficient,not taking advantage of the sparse F and G
% // The first Method is very specific to this implementation
% // ************************************************
function P = INS_CovariancePrediction(F,G,Q,dT,P)
I=eye(13);
P=(I+F*dT)*P*(I+F*dT)'+dT^2*G*Q*G';