% Minimizes ||q - (Rp + t)||^2
function [R, t] = arun(q, p)

% ASSUME: q, p are dxn (d: 2D or 3D)
d = size(q,1);

% shift point clouds by centroids
mu_q = mean(q,2); % (rowwise)
mu_p = mean(p,2); % (rowwise)
Q = q - mu_q;
P = p - mu_p;

% construct H matrix (dxd)
H = Q * P';

% perform SVD of H
[U,~,V] = svd(H);
D = eye(size(H));
D(d,d) = det(U*V');

% solve rotation-only problem
R = U*D*V';

% solve translation
t = mu_q - R*mu_p;
end