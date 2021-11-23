function Ar = Ar(R)
%%PROBLEM 5.b

r11 = R(1,1);
r12 = R(1,2);
r13 = R(1,3);
r21 = R(2,1);
r22 = R(2,2);
r23 = R(2,3);
r31 = R(3,1);
r32 = R(3,2);
r33 = R(3,3);
B = [r11;r22;r33;1];
A = [1,1,-1,-1;1,-1,1,-1;1,-1,-1,1;1,1,1,1];
x = A^(-1)*B;
q0 = sqrt(x(1));
q1 = sqrt(x(2));
q2 = sqrt(x(3));
q3 = sqrt(x(4));

Q1 = [q0,q1,q2,q3];

Max = max(Q1);
if q0 == Max
qmax=q0;
q1 = (r32-r23)/(4*q0);
q2 = (r13-r31)/(4*q0);
q3 = (r21-r12)/(4*q0);    
elseif q1 == Max
qmax=q1;
q0 = (r32-r23)/(4*q1);
q2 = (r12+r21)/(4*q1);
q3 = (r13+r31)/(4*q1);
elseif q2 == Max
qmax=q2;
q0 = (r13-r31)/(4*q2);
q1 = (r12+r21)/(4*q2);
q3 = (r23+r32)/(4*q2);
elseif q3 == Max
qmax=q3;
q0 = (r21-r12)/(4*q3);    
q1 = (r13+r31)/(4*q3);    
q2 = (r23+r32)/(4*q3);
else 
    disp("SOMETHING IS WRONG");
end

sumsquare = sqrt(q0^2+q1^2+q2^2+q3^2);

Ar = [q0;q1;q2;q3];

end
