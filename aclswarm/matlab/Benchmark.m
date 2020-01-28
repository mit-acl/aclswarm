%% Benchmark SDP and ADMM gain design methods
%
%
addpath('cvx');
addpath('Helpers');
cvx_startup;

% Reset the random number generator (for repeatability of the results)
rng(0,'twister'); 


% Number of agents to test
% numAgt = [5 10 30 60 150 300];

numAgt = [3 10 50 100 150 200];
% numAgt = [3 5 10];
numAgt = 4;

% Number of tests
numTst = length(numAgt);

% Vector of execution times 
Tsdp = zeros(1,numTst);
Tadmm = zeros(1,numTst);


%% Benchmark


for i = 1 : numTst

fprintf('Iteration %i of %i ...\n', i, numTst);
    
n = numAgt(i); % Number of agents   

% Random desired formation cooridnates
Qs = rand(3,n) * 5; 

% Random adjacency matrix
adj = ones(n) - diag(ones(n,1));  % Complete graph

% Choose (at most) k edges to remove:
k = 0;
rowIdx = randi(n, 1,k);
colIdx = randi(n, 1,k);    
for j = 1 : k
    adj(rowIdx(j),colIdx(j)) = 0;
    adj(colIdx(j),rowIdx(j)) = 0;
end

% SDP approach via CVX
% fprintf('Running SDP solver for n = %i ...\n', i);
% tic
% Asdp = SDPGainDesign3D(Qs, adj);
% Tsdp(i) = toc

% % Eigenvalues of the gain matrix
% eigSdp = sort( abs(eig(Asdp)) )'
% trace(Asdp)


% Customized ADMM approach 
fprintf('Running ADMM solver for n = %i ...\n', i);
tic
Aadmm = ADMMGainDesign3D(Qs, adj);
Tadmm(i) = toc

% % Eigenvalues of  the gain matrix
% eigAdmm = sort( abs(eig(Aadmm)) )
% trace(Aadmm)


end




%% test

% 
% % Desired formation as regular N-gon
% n = 6;
% qAng = linspace(0,360,n+1);        % Desired locations of agents on the unit circle given in angle
% qAng(end) = [];
% Qs = [cosd(qAng); sind(qAng)] * 3; % Desired locations in x-y coordinates
% 
% % % Line formation
% % Qs = [(1:n); zeros(1,n)];
% 
% % Graph adjacency matrix
% adj = [ 0     1     1     0     0     0
%         1     0     1     1     1     0
%         1     1     0     0     1     1
%         0     1     0     0     1     0
%         0     1     1     1     0     1
%         0     0     1     0     1     0];
% 
% % SDP approach via CVX
% fprintf('Running SDP solver for n = %i ...\n', i);
% tic
% Asdp = SDPGainDesign2D(Qs, adj);
% toc
% 
% % Normalized the gain matrix
% eigSdp = sort( abs(eig(Asdp)) )
% trace(Asdp)
% 
% % Customized ADMM approach 
% fprintf('Running ADMM solver for n = %i ...\n', i);
% tic
% Aadmm = ADMMGainDesign2D(Qs, adj);
% toc
% 
% % Normalized the gain matrix
% eigAdmm = sort( abs(eig(Aadmm)) )
% trace(Aadmm)
