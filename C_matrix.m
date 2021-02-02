function C=C_matrix(u)
global R  n_links
q=u(1:n_links);
qd=u(n_links+1:end);

C=R.coriolis(u');
