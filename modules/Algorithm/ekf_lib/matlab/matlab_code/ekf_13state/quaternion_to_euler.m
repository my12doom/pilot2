function [roll,pitch,yaw]=quaternion_to_euler(is_radian,q0,q1,q2,q3)
roll = atan2(2*(q2*q3+q0*q1),(q0^2-q1^2-q2^2+q3^2));
pitch = (-1)*asin(2*(q1*q3-q0*q2));
yaw = atan2(2*(q1*q2+q0*q3),(q0^2+q1^2-q2^2-q3^2));
if is_radian == 0
   roll = roll*180/pi;
   pitch=pitch*180/pi;
   yaw=yaw*180/pi;
else
    roll=roll;
    pitch=pitch;
    yaw=yaw;
end


