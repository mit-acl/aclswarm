%% Closed-loop dynamics of a quadrotor with nonlinear model under the 
%  proposed control strategy.
%
% (C) Kaveh Fathian, 2017-2018.  Email: kaveh.fathian@gmail.com
%
function state_dot = SysDynam(t,state,par)

A = par.A;                      % Control gain matrix
n = par.n;                      % Number of agents
Dd = par.Dd;                    % Desired distance matrix
qs = par.qs;                    % Desired formation points
adj = par.adj;                  % Adjacency matrix
vSat = par.vSat;                % Saturation velocity
dcoll = par.dcoll;              % Collision avaoidance distance 
rcoll = par.rcoll;              % Collision avaoidance circle radius
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

q = state(1:2*n);
v = state(2*n+1:end);
A0 = A;
Dd0 = Dd;
qs0 = qs;
adj0 = adj;


%% Assignment

if runAssign % If assignment should be used
    
% Run assignment at determined time intervals
if t - tAssign > T % Run assignment    
    disp('Assignment is being executed');
    
    % x-y position of agents in matrix
    qm = zeros(2,n);
    for i = 1 : n
        qm(:,i) = [q(2*i-1); q(2*i)];
    end
    
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
    
    % Assignment matrix
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
P2 = kron(P,eye(2));

% Wrong permutation:
% A = P2 * A0 * P2';
% Dd = P * Dd0 * P';

% Correct permutation
A = P2' * A0 * P2;
Dd = P' * Dd0 * P;
adj = P' * adj0 * P;


end


%% Dynamics

% Current distances
Dc = zeros(n,n); % inter-agent distances in current formation
for i = 1 : n
    for j = i+1 : n
        Dc(i,j) = norm(q(2*i-1:2*i)-q(2*j-1:2*j), 2);
    end
end
Dc = Dc + Dc';


%% Control to fix the scale

g = 2; % Gain
F = g * atan( adj.*(Dc-Dd) );
F = F + diag(-sum(F,2));
F = kron(F, eye(2));


%% Full control 

u0 = A * q  + F * q;
u = u0;


%% Collision avoidacne

if runColAvoid

% Positions in matrix form
qm = zeros(2,n);
for i = 1 : n
    qm(:,i) = [q(2*i-1); q(2*i)];
end

% Control in matrix form
um = zeros(2,n);
for i = 1 : n
    um(:,i) = [u0(2*i-1); u0(2*i)];
end

u = ColAvoid(um, qm, par);

end


%% Dynamics

% Speed saturation
for i = 1 : n
    ui = u(2*i-1:2*i);
    vi = norm(ui);
    if vi > vSat
        u(2*i-1: 2*i) = ui ./ vi .* vSat;
    end
end

% Derivative of state
I = eye(2*n);
q_dot = I * v;
v_dot = u - I * v;
state_dot = [q_dot; v_dot];
