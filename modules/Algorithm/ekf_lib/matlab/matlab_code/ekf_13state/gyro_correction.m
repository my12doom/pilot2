function [gx_zerobias,gy_zerobias,gz_zerobias]=gyro_correction(M)
gx_zerobias=M(2,2);
gy_zerobias=M(2,3);
gz_zerobias=M(2,4);