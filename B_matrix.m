function B=B_matrix(q)
global R  n_links
B=R.inertia(q(1:n_links,1)');
end