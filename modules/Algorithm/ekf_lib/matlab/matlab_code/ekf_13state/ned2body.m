function [vector_body]=ned2body(q_now,vector_ned)
Rbe=[q_now(1)^2+q_now(2)^2-q_now(3)^2-q_now(4)^2,2*(q_now(2)*q_now(3)+q_now(1)*q_now(4)),2*(q_now(2)*q_now(4)-q_now(1)*q_now(3));
    2*(q_now(2)*q_now(3)-q_now(1)*q_now(4)),q_now(1)^2-q_now(2)^2+q_now(3)^2-q_now(4)^2,2*(q_now(3)*q_now(4)+q_now(1)*q_now(2));
    2*(q_now(2)*q_now(4)+q_now(1)*q_now(3)),2*(q_now(3)*q_now(4)-q_now(1)*q_now(2)),q_now(1)^2-q_now(2)^2-q_now(3)^2+q_now(4)^2];
vector_body=Rbe*vector_ned;
