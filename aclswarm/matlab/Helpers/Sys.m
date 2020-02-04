%% Closed-loop dynamics of a quadrotor with nonlinear model under the 
%  proposed control strategy.
%
% (C) Kaveh Fathian, 2017-2018.  Email: kaveh.fathian@gmail.com
%
function state_dot = Sys(t,state,par)

A = par.A;                      % Control gain matrix
n = par.n;                      % Number of agents
Dd = par.Dd;                    % Desired distance matrix
qs = par.qs;                    % Desired formation points
adj = par.adj;                  % Adjacency matrix
vSat = par.vSat;                % Saturation velocity
runAssign = par.runAssign;      % Boolean to run assignment
T = par.T;                      % Reassignment period
method = par.method;            % Assignment method
runColAvoid = par.runColAvoid;  % Boolean to run collision avoidance



persistent P                 % Assignment matrix
if isempty(P)
    P = eye(n);              % Set initial assignment as identity
end

persistent tAssign           % Timer for running assignment 
if isempty(tAssign)
    tAssign = -1e10;         % Set to a large negative number to ensure assignment is executed in the first iteration
end

% disp(t); % time

q = state;
A0 = A;
Dd0 = Dd;
qs0 = qs;
adj0 = adj;

% x-y-z position of agents in matrix
qm = reshape(q, [3,n]);

%% Assignment

if runAssign % If assignment should be used
    
% Run assignment at determined time intervals
if t - tAssign > T % Run assignment    
    disp('Assignment is being executed');
    
    % Formation points at the latest assignment
    qs = qs0 * P;
    adj = P' * adj0 * P;
    
    if strcmp(method, 'cbaa')
        % CBAA algorithm
        [assign, ~] = CBAA_aclswarm(adj, qs, qm)
    elseif strcmp(method, 'hungarian')
        % Hungarian assignment for comparison
        assign = Hungarian_aclswarm(qs, qm)
    end
    
    % Assignment matrix (maps formpt to vehidx)
    Pnew = zeros(n);
    for ii = 1 : length(assign)
        if (assign(ii) ~= 0)
            Pnew(ii,assign(ii)) = 1;
        end
    end

    P = Pnew * P; % Update the assignment
    
    % Reset assignment time
    tAssign = t
    
%     keyboard
    
end

% Permute the state and the desired distance matrix
P2 = kron(P,eye(3));

% Wrong permutation:
% A = P2 * A0 * P2';
% Dd = P * Dd0 * P';

% Correct permutation
A = P2' * A0 * P2;
Dd = P' * Dd0 * P;
adj = P' * adj0 * P;


end


%% Relative distance
% inter-agent distances in current formation
Dc = squareform(pdist(qm'));

Dcxy = squareform(pdist(qm(1:2,:)'));
Dcz = squareform(pdist(qm(3,:)'));

%% Desired relative distance

Ddxy = squareform(pdist(qs(1:2,:)'));
Ddz = squareform(pdist(qs(3,:)'));

%% Control to fix the scale

% xy scale
K = 8; % gain
eps = 1 ./ (K * Dcxy);
eps(eps == Inf) = 0;
Fxy = eps .* atan( adj.*(Dcxy-Ddxy) );
Fxy = Fxy + diag(-sum(Fxy,2));

% z scale
K = 4; % gain
eps = 1 ./ (K * Dcz);
eps(eps == Inf) = 0;
Fz = eps .* atan( adj.*(Dcz-Ddz) );
Fz = Fz + diag(-sum(Fz,2));

F = kron(Fxy, eye(3));
F(3:3:end, 3:3:end) = Fz;

% all scale
% K = 8; % gain
% eps = 1 ./ (K * Dc);
% eps(eps == Inf) = 0;
% F = eps .* atan( adj.*(Dc-Dd) );
% F = F + diag(-sum(F,2));
% F = kron(F, eye(3));


%% Full control 

u0 = A * q + F * q;
u = u0;


%% Collision avoidance

if runColAvoid
    % Control in matrix form
    um = reshape(u0, [3,n]);

    u = ColAvoid(um, qm, par);
end

% pin down agent 1's altitude
% u(3) = 0;

%% Dynamics

% Speed saturation
for i = 1 : n
    ui = u(3*(i-1)+1 : 3*(i-1)+3);
    vi = norm(ui);
    if vi > vSat
        u(3*(i-1)+1 : 3*(i-1)+3) = ui ./ vi .* vSat;
    end
end


% Derivative of state
state_dot = u;
