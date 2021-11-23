function J2 = J_2(p,J1)

p_hat = [0,-1*p(3),p(2);p(3),0,-1*p(1);-1*p(2),p(1),0];
I = eye(3);
J2 = [I,2*p_hat*J1;zeros(3),2*J1];


end