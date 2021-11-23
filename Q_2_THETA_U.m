%%MEC 529 MOTION PLANNING ALGORITHM
%%BY ALEXANDER RUFRANO
%%111270310

function [theta, U1,U2,U3] = Q_2_THETA_U(Q,p)
uref = p/norm(p); %%USED ONLY WHEN IT IS PURELY TRANSLATIONAL
theta = acos(Q(1))*2; %%FIND THETA BASED FROM QUARTIRION
if theta == 0 %%WHEN END EFFECTOR POSITION DOES NOT CHANGE AND PURELY TRANSLATIONAL
    U1 = uref(1);%%UNIT VECTOR, X
    U2 = uref(2);%%UNIT VECTOR, Y
    U3 = uref(3);%%UNIT VECTOR, Z
else %%WHEN IT IS ROTATIONAL AND TRANSLATIONAL OR PURELY ROTATIONAL
    U1 = Q(2)/sin(theta/2); %%UNIT VECTOR, X
    U2 = Q(3)/sin(theta/2); %%UNIT VECTOR, Y
    U3 = Q(4)/sin(theta/2); %%UNIT VECTOR, Z
end

end