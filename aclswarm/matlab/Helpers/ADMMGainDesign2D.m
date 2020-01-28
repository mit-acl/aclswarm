%% ADMM for computing gain matrix.  
% --> Speeded up by using sparse matrix representation
% --> Set trace to a fixed value
%
% 
% Inputs:
%
%       - Qs :  Desired formation coordinates (2*n matrix, each column representing coordinate of formation point)
%       - adj:  Graph adjacency matrix (n*n logical matrix)
%
% Outputs:
%
%       - Aopt : Gain matrix
%
%
function Aopt = ADMMGainDesign2D(Qs, adj) 

% Make a vector out of formation points
qs = Qs(:);

% Number of agents
n = size(adj,1);
m = n - 2; % Reduced number

% 90-degree rotated desired formation coordinates
qsbar = zeros(2*n,1);
qsbar(1: 2: end-1) = - qs(2 : 2 : end);
qsbar(2: 2: end) = qs(1 : 2 : end-1);

% One vectors
one1 = zeros(2*n,1);
one1(1 : 2 : end-1) = 1;
one2 = zeros(2*n,1);
one2(2 : 2 : end) = 1;

% Get orthogonal complement of [z ones(n,1)]
N = [qs, qsbar, one1, one2]; % Desired kernel bases
[U,~,~] = svd(N);
Q = U(:,5:2*n); % Bases for the range space
Qt = Q'; % Q transpose


%% Preallocate variables

I0 = sparse(eye(2*m));
Z0 = sparse(2*m, 2*m);
Z = sparse(4*m, 4*m);

% Cost function's coefficient matrix: f = <C,X>
C = [I0, Z0;
     Z0, Z0];

% Trace of gain materix must be the specified value in 'trVal'
trVal = 2*m; % fixed value for trace


%%%%%%%%%%%%%%%%%%%%%% Find total number of nonzero elements in A and b matrices

% Number of elements in block [X]_11
numElmA1 = 1 + (2*m-1)*2 + m*(2*m-1); 
numElmB1 = 1;

% Number of elements in block [X]_12
numElmA2 = 2*m + (2*m)^2 - 2*m;
numElmB2 = 2*m;

% Zero-gain constraints for the given adjacency graph
S = not(adj);
S = S - diag(diag(S));
Su = triu(S); % Upper triangular part
[idxRow, idxCol] = find(Su); % Find location of nonzero entries
numConAdj = length(idxRow); % Number of constraints

% (upper bound for) Number of elements in block [X]_22
numElmA3 = 2*m*(m+1) + 2*(8^2)*numConAdj;
numElmB3 = 0;

% Number of elements for symmetry
numElmA4 = 2* 2*m * (4*m-1);
numElmB4 = 0;

% Number of elements for trace
numElmA5 = 2*m;
numElmB5 = 1;

% Total number of elements
numElmAtot = numElmA1 + numElmA2 + numElmA3 + numElmA4 + numElmA5;
numElmBtot = numElmB1 + numElmB2 + numElmB3 + numElmB4 + numElmB5;


%%%%%%%%%%%%%%%%%%%%%% Preallocate sparse matrices A & b
%
% Constraint: A * vec(X) = b
%

% Indices of A: entry [Ai(k), Aj(k)] takes value of Av(k) 
% Each row of matrix A will represent a constraint
Ai = zeros(numElmAtot,1);
Aj = zeros(numElmAtot,1);
Av = zeros(numElmAtot,1);

bi = zeros(numElmBtot,1);
bv = zeros(numElmBtot,1);


%% Generate the constraint matrix
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%   Constraints: A . vec(X) = b  %%%%%%%%%%%%%%%%%%%%%%%%%

itrr = 0; % Counter for rows in A constraint
itra = 0; % Counter for entries in A constraint
itrb = 0; % Counter for entries in b constraint


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_11

% The diagonal entries of X should be equal to the first diagonal entry
for i = 2 : 2*m
    itrr = itrr + 1;
    
    itra = itra + 1;    
    idxR = 1;
    idxC = 1;

    Ai(itra) = itrr;
    Aj(itra) = (idxR-1)*(4*m) + idxC;
    Av(itra) = 1;
    
    itra = itra + 1;
    idxR = i;
    idxC = i;
    
    Ai(itra) = itrr;
    Aj(itra) = (idxR-1)*(4*m) + idxC;
    Av(itra) = -1;
end

% Off-diagonal entries should be zero
for i = 1 : 2*m-1
    for j = i+1 : 2*m
        itrr = itrr + 1;
        
        itra = itra + 1;
        idxR = i;
        idxC = j;

        Ai(itra) = itrr;
        Aj(itra) = (idxR-1)*(4*m) + idxC;
        Av(itra) = 1;
    
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_12

% Diagonal entries should be 1
for i = 1 : 2*m
    itrr = itrr + 1;
    
    itra = itra + 1;
    idxR = i;
    idxC = i + 2*m;
    
    Ai(itra) = itrr;
    Aj(itra) = (idxR-1)*(4*m) + idxC;
    Av(itra) = 1;
    
    itrb = itrb + 1;
    bi(itrb) = itrr;
    bv(itrb) = 1;
end

% Other entries should be 0
for i = 1 : 2*m
    for j = 1 : 2*m
        if i ~= j
            itrr = itrr + 1;
            
            itra = itra + 1;
            idxR = i;
            idxC = j + 2*m;

            Ai(itra) = itrr;
            Aj(itra) = (idxR-1)*(4*m) + idxC;
            Av(itra) = 1;
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Block [X]_22

% Scaled rotation matrix structure constraints
for i = 1 : m
    for j = i : m 
        % Diagonal entries should be equal
        itrr = itrr + 1;
        
        itra = itra + 1;
        idxR = 2*m + 2*i-1;
        idxC = 2*m + 2*j-1;

        Ai(itra) = itrr;
        Aj(itra) = (idxR-1)*(4*m) + idxC;
        Av(itra) = 1;
        
        itra = itra + 1;
        idxR = 2*m + 2*i;
        idxC = 2*m + 2*j;

        Ai(itra) = itrr;
        Aj(itra) = (idxR-1)*(4*m) + idxC;
        Av(itra) = -1;
        
        
        % Off-diagonal entries have same value with different sign  
        if i == j
            itrr = itrr + 1;
            
            itra = itra + 1;
            idxR = 2*m + 2*i-1;
            idxC = 2*m + 2*j;

            Ai(itra) = itrr;
            Aj(itra) = (idxR-1)*(4*m) + idxC;
            Av(itra) = 1;
        else            
            itrr = itrr + 1;
            
            itra = itra + 1;
            idxR = 2*m + 2*i-1;
            idxC = 2*m + 2*j;

            Ai(itra) = itrr;
            Aj(itra) = (idxR-1)*(4*m) + idxC;
            Av(itra) = 1;
            
            itra = itra + 1;
            idxR = 2*m + 2*i;
            idxC = 2*m + 2*j-1;

            Ai(itra) = itrr;
            Aj(itra) = (idxR-1)*(4*m) + idxC;
            Av(itra) = 1;
        end
        
    end
end

% Zero constraints due to the adjacency matrix
for i = 1 : numConAdj
    ii = idxRow(i);
    jj = idxCol(i);
    
    % Diagonal terms, just do first column/row since a == a
    QQ = Qt(:,2*jj-1) * Q(2*ii-1,:);

    itrr = itrr + 1;
        
    for ki = 1 : 2*m
        for kj = 1 : 2*m
            itra = itra + 1;
            idxR = 2*m + ki;
            idxC = 2*m + kj;
            
            Ai(itra) = itrr;
            Aj(itra) = (idxR-1)*(4*m) + idxC;
            Av(itra) = QQ(ki, kj);
        end        
    end
        
    % Off-diagonal terms
    QQ = Qt(:,2*jj-1) * Q(2*ii,:);
    
    itrr = itrr + 1;
    
    for ki = 1 : 2*m
        for kj = 1 : 2*m
            itra = itra + 1;
            idxR = 2*m + ki;
            idxC = 2*m + kj;
            
            Ai(itra) = itrr;
            Aj(itra) = (idxR-1)*(4*m) + idxC;
            Av(itra) = QQ(ki, kj);
        end        
    end
end


% Trace of gain materix must be the specified value in 'trVal'
itrr = itrr + 1;

for i = 1 : 2*m
    itra = itra + 1;
    idxR = 2*m + i;
    idxC = idxR;
    
    Ai(itra) = itrr;
    Aj(itra) = (idxR-1)*(4*m) + idxC;
    Av(itra) = 1;
end

itrb = itrb + 1;
bi(itrb) = itrr;
bv(itrb) = trVal;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Symmetry

for i = 1 : (4*m-1)
    for j = (i+1) : 4*m 
        % Symmetric entries should be equal
        itrr = itrr + 1;
        
        itra = itra + 1;
        idxR = i;
        idxC = j;

        Ai(itra) = itrr;
        Aj(itra) = (idxR-1)*(4*m) + idxC;
        Av(itra) = 1;
        
        itra = itra + 1;
        idxR = j;
        idxC = i;

        Ai(itra) = itrr;
        Aj(itra) = (idxR-1)*(4*m) + idxC;
        Av(itra) = -1;        
    end
end

% Last element set to fix the size of b
itrb = itrb + 1;
bi(itrb) = itrr; 
bv(itrb) = 0;

% Remove any additional entries
Ai(itra+1:end) = [];
Aj(itra+1:end) = [];
Av(itra+1:end) = [];

bi(itrb+1:end) = [];
bv(itrb+1:end) = [];


% Make sparse matrices
A = sparse(Ai, Aj, Av);
b = sparse(bi, ones(length(bi), 1), bv);

% Size of optimization variable
sizX = 4 * m;



%% ADMM algorithm--full eigendecomposition

As = A.'; % Dual operator
AAs = A * As;
mu = 1; % Penalty    
epsEig = 1e-5;  % Precision for positive eig vals

% Stop criteria
thresh = 1e-4; % Threshold based on change in X updates
threshTr = 10; % Percentage threshold based on the trace value. If trace of X2 reaches within the specified percentage, the algorithm stops.
maxItr = 10; % Maximum # of iterations

% Initialize:
X = [I0 I0; I0 I0];
S = Z;
y = sparse(length(b),1);

for i = 1 : maxItr 
    
    %%%%%%% Update for y
    y = AAs \ (A * vec(C - S - mu * X) + mu * b);
    
    %%%%%%% Update for S
    W = C - reshape(As*y, [sizX,sizX]) - mu * X;
    W = (W + W.') ./ 2;
    
    [V, D] = eig(full(W));
    d = diag(D);
    posEig = d > epsEig;
    S = sparse(real(V(:,posEig) * diag(d(posEig)) * V(:,posEig).'));
    
    %%%%%%% Update for X
    Xold = X;
    X = (S - W) ./ mu;
    
    %%%%%%% Stop criteria
    difX = sum(abs(Xold(:) - X(:)));
    if difX < thresh % change in X is small
        break 
    end
    
    % trace of sparse matrix for MATLAB Coder
    trPerc = abs(full(sum(diag( X(2*m+1:end, 2*m+1:end) ))) - trVal) / trVal * 100;
    if trPerc < threshTr % trace of X is close to the desired value
        break
    end
    
end

% Set S=0 to project the final solution and ensure that it satisfies the linear constraints given by the adjacency matrix
S = 0 * S;
y = AAs \ (A * vec(C - S - mu * X) + mu * b);
W = C - reshape(As*y, [sizX,sizX]) - mu * X;
W = (W + W.') ./ 2;
X = (S - W) ./ mu;

X = full(X); % Transform X from sparse to full representation 

X2 = X(2*m+1:end,2*m+1:end); % The componenet of X corresponding to the gain matrix
Aopt = Q * (-X2) * Qt; % The formation gain matrix

% i
% eig(X2)
% trace(X2)


%% ADMM algorithm--sparse eigendecomposition (use for very large-size problems n>1000)

% As = A'; % Dual operator
% AAs = A * As;
% 
% mu = 1; % Penalty    
% epsEig = 1e-5;  % Precision for positive eig vals
% 
% % Stop criteria
% thresh = 1e-4; % Threshold based on change in X updates
% threshTr = 10; % Percentage threshold based on the trace value. If trace of X2 reaches within the specified percentage, the algorithm stops.
% maxItr = 10; % Maximum # of iterations
% 
% % Initialize:
% X = [I0 I0; I0 I0];
% S = Z;
% y = sparse(length(b),1);
% 
% for i = 1 : maxItr
%     
%     %%%%%%% Update for y
%     y = AAs \ (A * reshape(C - S - mu * X, [sizX^2,1]) + mu * b);
%     
%     
%     %%%%%%% Update for S
%     W = C - reshape(As*y, [sizX,sizX]) - mu * X;
%     W = (W + W')/2;
%     
%     % Find positive eigenvalues of W using an iterative technique
%     V = zeros(sizX);
%     d = zeros(sizX, 1);
%     
%     % Eigenvalues are repeated in paris. We find the corresponding eigvector, 
%     % then find the other eigenvector by rotating the first one 90 degrees
%     [vw1, dw1] = eigs(W,1,'largestreal', 'Tolerance',epsEig, 'FailureTreatment','keep', 'Display', false); 
%     dw2 = dw1; % The second eigenvalue
%     % The second eigenvector
%     vw2 = zeros(4*m,1);
%     for j = 1 : 2*m
%         vw2(j*2-1:j*2) = [0 -1; 1 0] * vw1(j*2-1:j*2);
%     end
%     
%     numPos = 0; % Number of positive eigenvalues
%     Wold = W;
%     while dw1 > 10*epsEig %dw(1) > epsEig
%         % Save positive eigenvalues and eigenvectors
%         numPos = numPos + 1;
%         d(numPos*2-1:numPos*2) = [dw1; dw2];
%         V(:, numPos*2-1:numPos*2) = [vw1, vw2];
%         
%         % Remove positive eigenvalue components from matrix W        
%         Wnew = Wold - dw1*vw1*vw1' - dw2*vw2*vw2.';
%         
%         % Find most positive eigenvalues of the new W matrix
%         [vw1, dw1] = eigs(Wnew,1,'largestreal', 'Tolerance',epsEig, 'FailureTreatment','keep', 'Display', false); 
%         dw2 = dw1; % The second eigenvalue
%         % The second eigenvector
%         vw2 = zeros(4*m,1);
%         for j = 1 : 2*m
%             vw2(j*2-1:j*2) = [0 -1; 1 0] * vw1(j*2-1:j*2);
%         end
%         
%         Wold = Wnew;
%     end
%     
%     S =  V(:,1:numPos*2) * diag(d(1:numPos*2)) * V(:,1:numPos*2).';
%     
%     %%%%%%% Update for X
%     Xold = X;
%     X = sparse((S - W) / mu);
%     
% 
%     %%%%%%% Stop criteria
%     difX = sum(abs(Xold(:) - X(:)));
%     if difX < thresh % change in X is small
%         break 
%     end
%     
%     trPerc = abs(trace( X(2*m+1:end, 2*m+1:end) ) - trVal) / trVal * 100;
%     if trPerc < threshTr % trace of X is close to the desired value
%         break
%     end
%     
% end
% 
% % Set S=0 to project the final solution and ensure that it satisfies the linear constraints given by the adjacency matrix
% S = 0 * S;
% y = AAs \ (A * reshape(C - S - mu * X, [sizX^2,1]) + mu * b);
% W = C - reshape(As*y, [sizX,sizX]) - mu * X;
% W = (W + W') / 2;
% X = (S - W) / mu;
% 
% X = full(X); % Transform X from sparse to full representation 
% 
% X2 = X(2*m+1:end,2*m+1:end); % The componenet of X corresponding to the gain matrix
% Aopt = Q * (-X2) * Qt; % The formation gain matrix
% 
% % i
% % eig(X2)
% % trace(X2)


%% Test solution

% eig(X2)
% eig(Aopt)

% % Enforce zero-gain constraint for non-neighbor agents
% Atrim = Aopt;
% for i = 1 : n
%     for j = 1 : n
%         if (i~=j) && ~adj(i,j)
%             Atrim(2*i-1:2*i, 2*j-1:2*j) = zeros(2);            
%         end
%     end
% end
% 
% eig(Atrim)
