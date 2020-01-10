%% SDP for computing Hermitian gain matrix.  
% 
% Inputs:
%
%       - Qs :  Desired formation coordinates (2*n matrix, each column representing coordinate of formation point)
%       - adj:  Graph adjacency matrix (n*n logical matrix)
%
% Outputs:
%
%       - Ar : Laplacina gain matrix
%       - Ac : Complex representation of the Laplacian matrix
%
% (C) Tyler Summers and Kaveh Fathian, 2017-2018.
%  tyler.summers@utdallas.edu, kaveh.fathian@gmail.com
%
function [Ar, Ac] = SDPGainDesign2D(Qs, adj) 

% Make a vector out of formation points
qs = Qs(:);

% Number of agents
n = size(adj,1);

% Complex representation of desired formation coordinates
p = qs(1 : 2 : end-1);
q = qs(2 : 2 : end);
z = p + 1i * q;

% Get orthogonal complement of [z ones(n,1)]
[U,~,~] = svd([z ones(n,1)]);
Q = U(:,3:n);

% Subspace constraint for the given graph
S = not(adj);
S = S - diag(diag(S));

m = n - 2;

% Solve via CVX
% NOTE: CVX must be downloaded and installed. See http://cvxr.com/cvx/
cvx_begin sdp
    cvx_precision low
    variable X(n-2,n-2) hermitian
    maximize( lambda_min( X ) )
    subject to
        (Q*X*Q') .* S == 0; % For agents that are not neighbors
        trace(X) == m;
cvx_end


A = Q * X * Q';

Ac = -full(A);    % Complex gain matrix
Ar = A_C2R(Ac);   % Real represnetation of gain matrix



%% Check answer 

% eig(Ar)

% Normalize A 
% Ar = Ar ./ max(abs(Ar(:)));

% eigVec = abs(eig(Ar));
% Ar = Ar ./ min(eigVec);

% e = eig(Ac)
% [Vl,Dl] = eig(Ac)
% Ac * z




