% Use this script to precalculate gains
clear; clc;

addpath('cvx');
addpath('Helpers');
cvx_startup;
format short;

%% swarm4
n = 4;
adj = ones(n) - eye(n);  % Complete graph

name = 'Square';
qs = [[0.0, 0.0, 0.0];
      [2.5, 0.0, 0.0];
      [0.0, 2.5, 0.0];
      [2.5, 2.5, 0.0]];
A = SDPGainDesign3D(qs', adj);
D = squareform(pdist(qs));
disp(name)
disp(A)
disp(D);

name = 'Line';
qs = [[0.0, 0.0, 0.0];
      [3.0, 0.0, 0.0];
      [1.5, 0.0, 0.0];
      [4.5, 0.0, 0.0]];
A = SDPGainDesign3D(qs', adj);
D = squareform(pdist(qs));
disp(name)
disp(A)
disp(D);

name = 'Diamond';
qs = [[1.25, 2.165063509, 0.0];
      [2.5, 0.0, 0.0];
      [0.0, 0.0, 0.0];
      [1.25, -2.165063509, 0.0]];
A = SDPGainDesign3D(qs', adj);
D = squareform(pdist(qs));
disp(name)
disp(A)
disp(D);

%% swarm5
n = 5;
adj = ones(n) - eye(n);  % Complete graph
adj = [[0, 1, 0, 1, 1];
       [1, 0, 1, 1, 0];
       [0, 1, 0, 1, 1];
       [1, 1, 1, 0, 1];
       [1, 0, 1, 1, 0]];
adj = [[0, 1, 0, 1, 1];
       [1, 0, 1, 1, 1];
       [0, 1, 0, 1, 1];
       [1, 1, 1, 0, 1];
       [1, 1, 1, 1, 0]];

% name = 'Line';
% qs = [[-2.0, 0.0, 0.0];
%       [-4.0, 0.0, 0.0];
%       [+2.0, 0.0, 0.0];
%       [+4.0, 0.0, 0.0];
%       [+0.0, 0.0, 0.0]];
% A = SDPGainDesign3D(qs', adj);
% D = squareform(pdist(qs));
% disp(name)
% disp(A)
% disp(D);
clc
name = 'Pentagon';
qs = [[2.0, 0.0, 0.0];
      [+0.618, +1.902, 1.0];
      [-1.618, +1.176, 0.0];
      [-1.618, -1.176, 2.0];
      [+0.618, -1.902, 0.0]];
A = SDPGainDesign3D(qs', adj);
D = squareform(pdist(qs));
% disp(name)
A(abs(A)<1e-6) = 0;
disp(A)
% disp(D);
disp(eig(A)');
disp(trace(A));

% name = 'Square+1';
% qs = [[-2., 0.0, 0.0];
%       [+2., 0.0, 0.0];
%       [0.0, +2., 0.0];
%       [0.0, -2., 0.0];
%       [0.0, 0.0, 0.0]];
% A = SDPGainDesign3D(qs', adj);
% D = squareform(pdist(qs));
% disp(name)
% disp(A)
% disp(D);

%% swarm6

name = 'Pentagon';
qs = [[2.0, 0.0];
      [+0.618, +1.902];
      [-1.618, +1.176];
      [-1.618, -1.176];
      [+0.618, -1.902]];
A = SDPGainDesign2D(qs', adj);
A(abs(A)<1e-6) = 0;
disp(A)
disp(eig(A)');
disp(trace(A));

%% mitacl15
n = 15;

name = 'MIT'
adj = ones(n) - eye(n);
qs = [[0.0, 0.0, 1.0];
      [0.0, 1.5, 2.0];
      [0.0, 3.0, 1.0];
      [1.5, 1.5, 2.0];
      [3.0, 1.5, 1.0];
      [1.5, 3.0, 2.0];
      [4.5, 1.5, 1.0];
      [3.0, 0.0, 2.0];
      [3.0, 3.0, 1.0];
      [4.5, 0.0, 2.0];
      [4.5, 3.0, 1.0];
      [6.0, 1.5, 2.0];
      [7.5, 3.0, 1.0];
      [6.0, 3.0, 2.0];
      [6.0, 0.0, 1.0]]
A = ADMMGainDesign3D(qs', adj);
D = squareform(pdist(qs));
disp(name)
disp(A)
disp(D);