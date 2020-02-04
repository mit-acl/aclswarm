% This script simulates formation control of a swarm of single-integrator
% agents.
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


%% Formation design

% Desired formation as regular N-gon
n = 4;
qAng = linspace(0,360,n+1);      % Desired locations of agents on the unit circle given in angle
qAng(end) = [];
r = 3;
% qs = [r*cosd(qAng); r*sind(qAng); ones(size(qAng))]; % 2d formation
% qs = [r*cosd(qAng); r*sind(qAng); 1:n]; % 3d formation

% Line formation
% qs = [(1:n); zeros(1,n); zeros(1,n)]; % 2d formation
qs = [(1:n); zeros(1,n); 1:n]; % 3d formation

% Random initial positions (can replace with any other value)
% rng(1)
rng(2)
q0 = rand(3,n) * 5;

% if formation starts 2d, it will stay 2d (invariant to scaling along z)
q0(3,:) = 1*ones(1,n);
q0(3,1) = 0;

% Graph adjacency matrix
adj = ones(n) - diag(ones(n,1));  % Complete graph


%% Parameters

Tassign     = 1e-0;              % Reassignment period
runAssign   = false;              % Boolean to run assignment
method      = 'cbaa';            % CBAA assignment
% method      = 'hungarian';       % Hungarian assignment
runColAvoid = false;             % Boolean to run collision avoidance

% n.b.: when colAvoid is on, ode45 may not progress if in gridlock.
% Gridlock is likely when initial conditions are random (i.e., agents start
% within each other's keep out regions).

T           = [0, 30];           % Simulation time interval 
numT        = diff(T)/0.01;      % Number of time samples in the simulation
vSat        = 6;                 % Max speed
dcoll       = 2;                 % Collision avoidance distance
rcoll       = 1;                 % Collision avoidance circle radius



%% Desired distance matrix

% Element (i,j) in matrix Dd describes the distance between agents i and j 
% in the formation. The diagonals are zero by definition.
Dd = squareform(pdist(qs')); % inter-agent distances in desired formation

%% Computing formation control gains

% Run CVX setup:
% cvx_setup
cvx_startup

% Find stabilizing control gains (Needs CVX)
A = ADMMGainDesign3D(qs, adj);
% A = SDPGainDesign3D(qs, adj);


%% Simulate the model

Tvec = linspace(T(1), T(2), numT);

% Initial state
state0 = q0(:);                % Initial condition

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
opt = odeset('AbsTol', 1.0e-6, 'RelTol', 1.0e-6);

clear sys
[t,stateMat] = ode45(@Sys, Tvec, state0, opt, par);

%% Show results

qf = zeros(size(q0));

% A rough plot of the trajectories
figure;
hold on
for i = 1 : n
    state = stateMat(:, 3*(i-1)+1 : 3*(i-1)+3);
    scatter3(state(:,1), state(:,2), state(:,3), 20, 'filled');
    scatter3(state(1,1), state(1,2), state(1,3), 50, 'rx');
    
    % extract final position
    qf(:,i) = state(end,:);
end
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on
hold off
axis equal
title('Trajectories')



% A rough plot of the final position
figure;
hold on
for i = 1 : n
    scatter3(qf(1,i), qf(2,i), qf(3,i), 50, 'filled');
end
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on 
hold off
axis equal
title('Final position')

if ~runAssign
    % The following scale control analysis may not be as clear when agents
    % are reordered due to reassignment.
    disp('-------------- xy --------------')
    Df = pdist(qf(1:2,:)')
    Ds = pdist(qs(1:2,:)')
    ratf = Df(1) ./ Df
    rats = Ds(1) ./ Ds

    disp('-------------- z --------------')
    Df = pdist(qf(3,:)')
    Ds = pdist(qs(3,:)')
    ratf = Df(1) ./ Df
    rats = Ds(1) ./ Ds
end