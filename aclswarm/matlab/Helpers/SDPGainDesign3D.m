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
function A = SDPGainDesign3D(Qs, adj) 

% Make a vector out of formation points
qs = Qs(:);

% Number of agents
n  = size(adj,1);   

oneX = kron(ones(n,1), [1; 0; 0]);
oneY = kron(ones(n,1), [0; 1; 0]);
oneZ = kron(ones(n,1), [0; 0; 1]);

% Rotate coordinates
qsBar = zeros(size(qs)); 
R = [0 -1  0;
     1  0  0;
     0  0  1];  % Rotation of 90 degrees about the z-axis 
for i = 1 : n
    qsBar((i-1)*3+1: i*3) = R * qs((i-1)*3+1: i*3);  
end

% Project on plane
qsp = qs;
for i = 1 : n
    qsp(3*i) = 0;
end

% Kernel space
N = [qs, qsBar, qsp, oneX, oneY, oneZ];

% Get orthogonal complement of kernel of A
[U,~,~] = svd(N);
Q = U(:,7:3*n);

% Subspace constraints for the given sensing topology
S = not( kron( adj+diag(ones(1,n)) , ones(3,3) )  );

m = n - 2;

% % Solve via CVX
% % NOTE: CVX must be downloaded and installed. See http://cvxr.com/cvx/
% cvx_begin sdp
%     cvx_precision low
%     variable X(3*m,3*m) symmetric
%     minimize( lambda_max( X ) )
%     subject to
%         % For agents that are not neighbors
%         (Q*X*Q') .* S == 0;
%         for i = 1 : m % Loop over rows
%             for j = 1 : m % Loop over columns
%                 X(3*i-2, 3*j-2)     ==  X(3*i-1, 3*j-1);
%                 X(3*i-2, 3*j-1)     == -X(3*i-1, 3*j-2);
%                 X(3*i-2:3*i-1, 3*j) == 0;
%                 X(3*i, 3*j-2:3*j-1) == 0;
%             end
%         end
%         % Fixed trace
%         trace(X) == m;
% cvx_end
% A = Q * (-X) * Q'; % Formation gain matrix


% % Solve via CVX
% % NOTE: CVX must be downloaded and installed. See http://cvxr.com/cvx/
% cvx_begin sdp
%     cvx_precision low
%     variable X(3*m,3*m) symmetric
%     minimize( lambda_max( X ) )
%     subject to
%         % For agents that are not neighbors
%         AA = (Q*X*Q');
%         AA .* S == 0;
%         for i = 1 : m % Loop over rows
%             for j = 1 : m % Loop over columns
%                 AA(3*i-2, 3*j-2)     ==  AA(3*i-1, 3*j-1);
%                 AA(3*i-2, 3*j-1)     == -AA(3*i-1, 3*j-2);
%                 AA(3*i-2:3*i-1, 3*j) == 0;
%                 AA(3*i, 3*j-2:3*j-1) == 0;
%             end
%         end
%         % Fixed trace
%         trace(X) == m;
% cvx_end
% A = Q * (-X) * Q'; % Formation gain matrix


% NOTE: CVX must be downloaded and installed. See http://cvxr.com/cvx/
cvx_begin sdp
    variable A(3*n,3*n) symmetric
    maximize( lambda_min( Q'*A*Q ) )
    subject to
        A * N == 0;   % Kernel of A
        A .* S == 0;  % For agents that are not neighbors
        for i = 1 : n % Loop over rows
            for j = 1 : n % Loop over columns
                if adj(i,j) % If agents are neighbors
                    A(3*i-2, 3*j-2)     ==  A(3*i-1, 3*j-1);
                    A(3*i-2, 3*j-1)     == -A(3*i-1, 3*j-2);
                    A(3*i-2:3*i-1, 3*j) == 0;
                    A(3*i, 3*j-2:3*j-1) == 0;
                end
            end
        end
        trace(A) == m;
cvx_end
A = full(-A); % Make matrix negative semi-definite



%% Check answer 

% eig(Ar)



% % NOTE: CVX must be downloaded and installed. See http://cvxr.com/cvx/
% cvx_begin sdp
%     variable A(3*n,3*n) symmetric
% %     minimize( trace_inv( Q'*A*Q ) )
%     maximize( lambda_min( Q'*A*Q ) )
% %     maximize( log_det( Q'*A*Q) )
%     subject to
%         A * N == 0;   % Kernel of A     
%         A .* S == 0;  % For agents that are not neighbors
%         norm(A) <= 10;        
%         for i = 1 : n % Loop over rows
%             for j = 1 : n % Loop over columns
%                 if adj(i,j) % If agents are neighbors
%                     A(3*i-2, 3*j-2)     ==  A(3*i-1, 3*j-1);
%                     A(3*i-2, 3*j-1)     == -A(3*i-1, 3*j-2);
%                     A(3*i-2:3*i-1, 3*j) == 0;
%                     A(3*i, 3*j-2:3*j-1) == 0;
%                 end
%             end
%         end                            
% cvx_end


% Ar = - A; % Make matrix negative semi-definite
% 
% Ar = full(Ar);
% 
% maxAr = max(abs(Ar(:)));
% Ar = Ar ./ maxAr;
% 
% evec = sort(abs(eig(Ar)),'ascend');
% Kd = evec(7);
