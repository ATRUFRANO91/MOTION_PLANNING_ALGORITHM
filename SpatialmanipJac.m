function Js = SpatialmanipJac(w,q,type,theta)

i = 1;
[row, numoftwists] = size(w);

%%obtain all twists coordinates and exponential twists

while i <= numoftwists %%find all joint twists for g1,i-1
    if type(i) == 1 %%For revolute
        xi(1:6,1,i) = [-1*cross(w(1:3,i),q(1:3,i));w(1:3,i)]; %%6x1 matrix for revolute
        exp_xitheta(1:4,1:4,i) = exp_twist(xi(1:6,1,i), theta(i)); %%exp_twist made from Professor Nilanjan Chakraborty
        g1(1:4,1:4,i) = exp_xitheta(1:4,1:4,i);
    elseif type(i) == 0 %%for prismatic
        xi(1:6,1,i) = [w(1:3,i);0;0;0];
        exp_xitheta(1:4,1:4,i) = exp_twist(xi(1:6,1,i), theta(i));
        g1(1:4,1:4,i) = exp_xitheta(1:4,1:4,i);
    end
    
    
    i=i+1;
    
end

Js(1:6,1) = xi(1:6,1,1);%%xi,i for first column of spatial jacobian
g1i(1:4,1:4) = g1(1:4,1:4,1); %%g1,i-1
Adg1(1:6,1:6,1) = AdjointOfg((g1i(1:4,1:4)));
for j = 2:(numoftwists)
    g1i(1:4,1:4) = g1i(1:4,1:4)*g1(1:4,1:4,j);%%g1,i-1
    Adg1(1:6,1:6,j) = AdjointOfg((g1i(1:4,1:4)));%%adjointg1,i-1
    Js(1:6,j) = Adg1(1:6,1:6,j-1)*xi(1:6,1,j);%%spatial jacobian
end

Adg1(1:6,1:6) = Adg1(1:6,1:6,6);%%set last adjoint computed as final one. 




end




