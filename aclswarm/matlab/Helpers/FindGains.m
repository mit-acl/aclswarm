%% SDP for computing Hermitian gain matrix.  
% 
% Inputs:
%
%       - qs :  Desired formation coordinates (vector with 2*n elements)
%       - adj:  Graph adjacency matrix (n*n logical matrix)
%
% Outputs:
%
%       - Lr : Laplacian gain matrix
%       - Lf : Complex representation of the Laplacian matrix
%
% (C) Tyler Summers and Kaveh Fathian, 2017-2018.
%  tyler.summers@utdallas.edu, kaveh.fathian@gmail.com
%
function [Ar, Ac] = FindGains(qs, adj) 

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

% Solve via CVX
% NOTE: CVX must be downloaded and installed. See http://cvxr.com/cvx/
cvx_begin sdp
    variable A(n,n) hermitian
%     minimize( trace_inv( Q'*A*Q ) )
    maximize( lambda_min( Q'*A*Q ) )
%     maximize( log_det( Q'*A*Q) )
    subject to
        A*[z ones(n,1)] == 0+ 1i*0;
        norm(A) <= 10;
        A.*S == 0;
cvx_end

Ac = -full(A);    % Complex gain matrix
Ar = A_C2R(Ac);   % Real represnetation of gain matrix

% Normalize A
Ar = Ar ./ max(abs(Ar(:)));

% e = eig(Ac)
% [Vl,Dl] = eig(Ac)
% Ac * z
