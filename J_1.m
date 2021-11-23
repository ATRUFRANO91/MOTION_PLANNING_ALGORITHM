function J1 = J_1(Q)

q0=Q(1);
q1=Q(2);
q2=Q(3);
q3=Q(4);



J1 = [-q1,q0,q3,-q2;-q2,-q3,q0,q1;-q3,q2,-q1,q0];

end