function x=fwdKine(q)
global R n_links

T=R.fkine(q');  % input requires 1xN vector
x=T(1:3,4);