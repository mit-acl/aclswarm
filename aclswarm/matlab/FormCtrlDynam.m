% This script simulates formation control of a group of quadrotors.
%
% -------> Scale of the formation is NOT controlled in this demo!
%
% -------> IMPORTANT:  CVX must be installed before running! (see README)
%
%
% (C) Kaveh Fathian, 2018.  Email: kaveh.fathian@gmail.com
%
% This program is a free software: you can redistribute it and/or modify it
% under the terms of the GNU lesser General Public License, either version 
% 3, or any later version.
%
% This program is distributed in the hope that it will be useful, but 
% WITHOUT ANY WARRANTY. See the GNU Lesser General Public License for more 
% details <http://www.gnu.org/licenses/>.
%
addpath('cvx');
addpath('Helpers');
addpath('CBAA');
addpath('Hungarian');


%% Simulation parameters for triangle formation

% Desired formation as regular N-gon
n = 10;
% qAng = linspace(0,360,n+1);      % Desired locations of agents on the unit circle given in angle
% qAng(end) = [];
% qs = [cosd(qAng); sind(qAng)] * 3; % Desired locations in x-y coordinates

% Line formation
qs = [(1:n); zeros(1,n)];

n = size(qs,2);       % Number of agents

% Random initial positions (can replace with any other value)
% rng(1)
rng(2)
q0 = rand(2,n) * 5;


% % Graph adjacency matrix
% adj = [ 0     1     1     0     0     0
%         1     0     1     1     1     0
%         1     1     0     0     1     1
%         0     1     0     0     1     0
%         0     1     1     1     0     1
%         0     0     1     0     1     0];

adj = ones(n) - diag(ones(n,1));  % Complete graph

     
%% Parameters

Tassign     = 5e-0;              % Reassignment period
runAssign   = true;              % Boolean to run assignment
% method      = 'cbaa';            % CBAA assignment
method      = 'hungarian';       % Hungarian assignment
runColAvoid = false;             % Boolean to run collision avoidance

T           = [0, 70];           % Simulation time interval 
numT        = 1000;              % Number of time samples in the simulation
vSat        = 3;                 % Max speed
dcoll       = 2;                 % Collision avaoidance distance
rcoll       = 1;                 % Collision avaoidance circle radius



%% Desired distance matrix

% Element (i,j) in matrix Dd describes the distance between agents i and j 
% in the formation. The diagonals are zero by definition.
Dd = zeros(n,n); % inter-agent distances in desired formation
for i = 1 : n
    for j = i+1 : n
        Dd(i,j) = norm(qs(:,i)-qs(:,j), 2);
    end
end
Dd = Dd + Dd';


%% Computing formation control gains

% Run CVX setup:
% cvx_setup
cvx_startup

% Find stabilizing control gains (Needs CVX)
A = FindGains(qs(:), adj);


%% Simulate the model

Tvec = linspace(T(1), T(2), numT);

% Initial state
state0 = [q0(:); zeros(2*n,1)];                % Initial condition


% Parameters passed down to the ODE solver
par.n = n;                      % Number of agents
par.A = A;                      % Control gain matrix
par.qs = qs;                    % Desired formation points
par.adj = adj;                  % Adjacency matrix
par.Dd = Dd;                    % Desired distances
par.vSat = vSat;                % Saturation velocity
par.dcoll = dcoll;              % Collision avaoidance distance
par.rcoll = rcoll;              % Collision avaoidance circle radius
par.T = Tassign;                % Reassignment period
par.runAssign = runAssign;      % Boolean to run assignment
par.method = method;            % Assignment method
par.runColAvoid = runColAvoid;  % Boolean to run collision avoidance


% Simulate the dynamic system
opt = odeset('AbsTol', 1.0e-4, 'RelTol', 1.0e-4);

clear SysDynam
[t,stateMat] = ode45(@SysDynam, Tvec, state0, opt, par);




%% Show results

% A rough plot of the trajectories
figure;
hold on
for i = 1 : n
    state = stateMat(:,2*i-1:2*i);
    scatter(state(:,1), state(:,2), 20, 'filled');
end
grid on
hold off
axis equal
title('Trajectories')



% A rough plot of the final position
figure;
hold on
for i = 1 : n
    state = stateMat(:,2*i-1:2*i);
    scatter(state(end,1), state(end,2),50, 'filled');
end
grid on 
hold off
axis equal
title('Final position')
