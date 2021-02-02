function Jdq= JacobianDerivative(u) %u is [q,qd]
global R n_links
q=u(1:n_links,1)';
qd=u(n_links+1:end, 1)';
Jd=R.jacob_dot(q,qd);

if n_links==1
    J=Jd(1,1);
elseif n_links ==2 
    J=[Jd(1,1);Jd(6,1)];
elseif n_links ==3
    J=[Jd(1,1);Jd(2,1);Jd(6,1)];
elseif n_links ==4
    J=[Jd(1,1);Jd(2,1);Jd(3,1);Jd(6,1)];
elseif n_links == 5
    J=[Jd(1,1);Jd(2,1);Jd(3,1); Jd(4,1);Jd(6,1)];
else
    J=Jd;
end

Jdq=J;