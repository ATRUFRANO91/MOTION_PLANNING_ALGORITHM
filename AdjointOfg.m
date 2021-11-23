function Adjointg = AdjointOfg(g)
%%Let g be a 4x4 matrix
R = g(1:3,1:3); %%rotational matrix
p = g(1:3,4); %%position vector
p_hat = [0, -1*p(3),p(2);p(3),0,-1*p(1);-1*p(2),p(1),0]; %%position skew symmetric
zero = zeros(3,3); %%zero matrix
Adjointg = [R,p_hat*R;zero,R]; %%adjoint definition
end