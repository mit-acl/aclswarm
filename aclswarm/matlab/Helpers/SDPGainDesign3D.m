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

function A = SDPGainDesign3D(Qs, adj) 

% Number of agents
n  = size(adj,1);   


%% Formation gains corresponding to the 2D formation 
% (2D formation is defined as the projection of the 3D formation on the x-y plane)
Axy = SDPGainDesign2D(Qs(1:2,:), adj);


%% Formation gains corresponding to altitude

oneZ = ones(n,1); % Vector of ones
qz = Qs(3,:)'; % Vector of z-coordinates

% Kernel space
N = [qz, oneZ];

% Determine if desired formation is actually 2D
xyform = (std(qz) < 1e-2);

% Reduced problem dimension
if xyform, dimkerAz = 1;
else, dimkerAz = 2; end
m = n - dimkerAz;

% Get orthogonal complement of N
[U,~,~] = svd(N);
% Qbar = U(:,1:dimkerAz);
Q = U(:,(dimkerAz+1):n);

% Subspace constraints for the given sensing topology
S = not( adj + diag(ones(1,n)) );

% Solve via CVX
% NOTE: CVX must be downloaded and installed. See http://cvxr.com/cvx/
cvx_begin sdp quiet
    cvx_precision low
    variable X(m,m) symmetric
    maximize( lambda_min( X ) )
    subject to
        (Q*X*Q') .* S == 0; % For agents that are not neighbors
        trace(X) == m; % Fixed trace
cvx_end

Az = - full(Q * X * Q'); % Formation gain matrix


%% 3D formation gain matrix

A = zeros(3*n, 3*n);
for i = 1 : n
    for j = 1 : n
        % Component corresponding to 2D formation
        A(3*i-2:3*i-1, 3*j-2:3*j-1) = Axy(2*i-1:2*i, 2*j-1:2*j);
        % Component corresponding to altitude
        A(3*i, 3*j) = Az(i,j);
    end
end
