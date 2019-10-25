% Use this script to precalculate gains
clear; clc;

addpath('cvx');
addpath('Helpers');
cvx_startup;

%% swarm4
n = 4;
adj = ones(n) - eye(n);  % Complete graph

name = 'Square';
qs = [[0.0, 0.0, 0.0];
      [2.5, 0.0, 0.0];
      [0.0, 2.5, 0.0];
      [2.5, 2.5, 0.0]];
A = FindGains3D(qs', adj);
D = squareform(pdist(qs));
disp(name)
disp(A)
disp(D);

name = 'Line';
qs = [[0.0, 0.0, 0.0];
      [3.0, 0.0, 0.0];
      [1.5, 0.0, 0.0];
      [4.5, 0.0, 0.0]];
A = FindGains3D(qs', adj);
D = squareform(pdist(qs));
disp(name)
disp(A)
disp(D);

name = 'Diamond';
qs = [[1.25, 2.165063509, 0.0];
      [2.5, 0.0, 0.0];
      [0.0, 0.0, 0.0];
      [1.25, -2.165063509, 0.0]];
A = FindGains3D(qs', adj);
D = squareform(pdist(qs));
disp(name)
disp(A)
disp(D);

%% swarm5

%% swarm6

%% mitacl15