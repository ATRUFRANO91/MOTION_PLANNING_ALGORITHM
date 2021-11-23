function Astar = Astar(A)

Ar = A(1:end,1);
Ad = A(1:end,2);
Arstar = [Ar(1);-1*Ar(2:end)];
Adstar = [Ad(1);-1*Ad(2:end)];
Astar = [Arstar,Adstar];


end