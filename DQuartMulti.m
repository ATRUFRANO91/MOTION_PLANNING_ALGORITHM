function DQuartMulti = DQuartMulti(A,B)

P = A(:,1);
Q = A(:,2);
U = B(:,1);
V = B(:,2);

real= QuartProduct(P,U); 
dual = QuartProduct(Q,U) + QuartProduct(P,V);
DQuartMulti = [real,dual];

end