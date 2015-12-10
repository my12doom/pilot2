function [vector_ned]=body2ned(q_now,vector_body)
Reb=[q_now(1)^2+q_now(2)^2-q_now(3)^2-q_now(4)^2,2*(q_now(2)*q_now(3)+q_now(1)*q_now(4)),2*(q_now(2)*q_now(4)-q_now(1)*q_now(3));
    2*(q_now(2)*q_now(3)-q_now(1)*q_now(4)),q_now(1)^2-q_now(2)^2+q_now(3)^2-q_now(4)^2,2*(q_now(3)*q_now(4)+q_now(1)*q_now(2));
    2*(q_now(2)*q_now(4)+q_now(1)*q_now(3)),2*(q_now(3)*q_now(4)-q_now(1)*q_now(2)),q_now(1)^2-q_now(2)^2-q_now(3)^2+q_now(4)^2]';
vector_ned=Reb*vector_body;
