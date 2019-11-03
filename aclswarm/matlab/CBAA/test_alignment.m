p = [[ 0  0  0];
     [ 1  1  0];
     [-1  1  0];
     [ 0 -1  0];
     [ 0 -2  0]];

rng(1);
R = eul2rotm([pi/2 0 pi/4], 'ZYX')
a = -2; b = 2;
t = a + (b-a).*rand(3,1)
 
q = (R*p' + t)';

figure(1), clf;
scatter3(p(:,1),p(:,2),p(:,3));
hold on;
scatter3(q(:,1),q(:,2),q(:,3));
axis([-3 3 -3 3 -3 3]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(0,90);

%% Arun's Method (3d)
% Minimizes ||q - (Rp + t)||^2

% ASSUME: q, p are 3xN
q = q';
p = p';

% shift point clouds by centroids
mu_q = mean(q,2); % (colwise)
mu_p = mean(p,2); % (colwise)
Q = q - mu_q;
P = p - mu_p;

% construct H matrix (3x3)
H = Q * P';

% perform SVD of H
[U,~,V] = svd(H);
d = [1, 1, det(U*V')];

% solve rotation-only problem
Rhat = U*diag(d)*V'

% solve translation
that = mu_q - R*mu_p

paligned = (Rhat*p + that)';
scatter3(paligned(:,1),paligned(:,2),paligned(:,3));
