function QuartProduct = QuartProduct(p,q) 

a1 = p(1);
b1 = p(2);
c1 = p(3);
d1 = p(4);

a2 = q(1);
b2 = q(2);
c2 = q(3);
d2 = q(4);

a3 = a1*a2-b1*b2-c1*c2-d1*d2;
b3 = a1*b2+a2*b1+c1*d2-c2*d1;
c3 = a1*c2+a2*c1+b2*d1-b1*d2;
d3 = a1*d2+a2*d1+b1*c2-b2*c1;

QuartProduct = [a3,b3,c3,d3]';


p0 = p(1);
p = p(2:end);
q0 = q(1);
q = q(2:end);

QuartProduct = [q0*p0 - dot(q,p);q0*p+p0*q+cross(q,p)];

end