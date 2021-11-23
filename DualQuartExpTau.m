function DualQuartExpTau = DualQuartExpTau(theta, tau, u,p)
d = dot(p,u);
if theta == 0
    m = zeros(3,1);
else
    m = 0.5*(cross(p,u)+(p-d*u)*cot(theta/2));
end
ubar = [u,m];
sinthetabartau = [sin(tau*theta/2),tau*d/2*cos(tau*theta/2)];
DualQuartExpTau = [cos(tau*theta/2),-1*tau*d/2*sin(tau*theta/2);DualVectorNumberMulti(sinthetabartau,ubar)];
end