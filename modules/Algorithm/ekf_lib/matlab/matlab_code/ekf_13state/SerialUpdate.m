function [X P]=SerialUpdate(H,R,Z,Y,P,X)
% // *************  SerialUpdate *******************
% // Does the update step of the Kalman filter for the covariance and estimate
% // Outputs are Xnew & Pnew, and are written over P and X
% // Z is actual measurement, Y is predicted measurement
% // Xnew = X + K*(Z-Y), Pnew=(I-K*H)*P,
% // where K=P*H'*inv[H*P*H'+R]
% // NOTE the algorithm assumes R (measurement covariance matrix) is diagonal
% // i.e. the measurment noises are uncorrelated.
% // It therefore uses a serial update that requires no matrix inversion by
% // processing the measurements one at a time.
% // Algorithm - see Grewal and Andrews, "Kalman Filtering,2nd Ed" p.121 & p.253
% // - or see Simon, "Optimal State Estimation," 1st Ed, p.150
% // The SensorsUsed variable is a bitwise mask indicating which sensors
% // should be used in the update.
% // ************************************************
K=P*H'*inv(H*P*H'+R);
X=X+K*(Z-Y);
P=P-K*H*P;