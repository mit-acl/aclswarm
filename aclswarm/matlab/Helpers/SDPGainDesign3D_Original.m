%% SDP for computing Hermitian gain matrix.
%
% Inputs:
%
%       - Qs :  Desired formation coordinates (3*n matrix, each column representing coordinate of formation point)
%       - adj:  Graph adjacency matrix (n*n logical matrix)
%
% Outputs:
%
%       - A : Aggregate gain matrix
%
%     See K. Fathian et al., "Robust 3D Distributed Formation Control with
%     Collision Avoidance and Application to Multirotor Aerial Vehicles," ICRA'18
%
% (C) Tyler Summers and Kaveh Fathian, 2017-2018.
%     tyler.summers@utdallas.edu, kaveh.fathian@gmail.com

function A = SDPGainDesign3D_Original(Qs, adj)

% Number of agents
n  = size(adj,1);

qs = Qs(:); % Desired formation coordinates in vector form

% Vectors of one entries in each dimension
oneX = kron(ones(n,1), [1; 0; 0]);
oneY = kron(ones(n,1), [0; 1; 0]);
oneZ = kron(ones(n,1), [0; 0; 1]);

% Vectors for desired formation coordinates
qs1 = zeros(size(qs));
qs2 = zeros(size(qs));
qs3 = zeros(size(qs));

R1 = [1  0  0;
      0  1  0;
      0  0  0];
R2 = [0 -1  0;
      1  0  0;
      0  0  0];
R3 = [0  0  0;
      0  0  0;
      0  0  1];
for i = 1 : n
    qs1((i-1)*3+1: i*3) = R1 * qs((i-1)*3+1: i*3);
    qs2((i-1)*3+1: i*3) = R2 * qs((i-1)*3+1: i*3);
    qs3((i-1)*3+1: i*3) = R3 * qs((i-1)*3+1: i*3);
end


qz = Qs(3,:)'; % Vector of z-coordinates
xyform = (std(qz) < 1e-2); % Determine if desired formation is planar

% Set problem dimension
if xyform % Planar formation
    dimkerAz = 5; % Dimension of kernel
    N = [qs1, qs2, oneX, oneY, oneZ];  % Kernel of gain matrix
else
    dimkerAz = 6; % Dimension of kernel
    N = [qs1, qs2, qs3, oneX, oneY, oneZ]; % Kernel of gain matrix
end

m = 3*n - dimkerAz; % Reduced dimension

% Get orthogonal complement of N
[U,~,~] = svd(N);
Q = U(:,(dimkerAz+1):3*n);

% Subspace constraints for the given sensing topology
S = not( kron( adj+diag(ones(1,n)) , ones(3,3) )  );



%% CVX solver
% NOTE: CVX must be downloaded and installed. See http://cvxr.com/cvx/

cvx_begin sdp quiet
    cvx_precision low
    variable A(3*n,3*n) symmetric
    maximize( lambda_min( Q'*A*Q ) )
    subject to
        A * N == 0;   % Kernel of A
        A .* S == 0;  % For agents that are not neighbors set Aij blocks to zero
        trace(A) == m; % Fixed trace
        % Enforce structure of A matrix
        for i = 1 : n % Loop over row blocks
            for j = 1 : n % Loop over column blocks
                if adj(i,j) % If agents are neighbors enforce the special structure of Aij blocks
                    A(3*i-2, 3*j-2)     ==  A(3*i-1, 3*j-1);
                    A(3*i-2, 3*j-1)     == -A(3*i-1, 3*j-2);
                    A(3*i-2:3*i-1, 3*j) == 0;
                    A(3*i, 3*j-2:3*j-1) == 0;
                end
            end
        end
cvx_end

% Return gain matrix
A = - full(A);

% maxAr = max(abs(Ar(:)));
% Ar = Ar ./ maxAr;

%% Check answer
evec = sort(abs(eig(A)),'ascend');
Kd = evec(7);

% eig(A)
