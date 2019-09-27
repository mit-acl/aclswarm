clear;clc;close all

% folder = fileparts(which(mfilename)); 
% addpath(genpath(folder))
%% Choose test cases
TEST_NUM = 3;

switch TEST_NUM
case 1
    % line and triangle
    x_i = [-1, 0, 1;
            0, 0, 0];
    pgon = nsidedpoly(3);
    p_i = pgon.Vertices';
    perm_i = randperm(size(p_i, 2));
    p_i = p_i(:,perm_i);
    
    % Default fully connected
    Adj = ones(3)-eye(3);
case 2
    % random 4 points
    x_i = [4*(rand(1,5)-0.5);
           4*(rand(1,5)-0.5)];
    pgon = nsidedpoly(5);
    p_i = pgon.Vertices';
    perm_i = randperm(size(p_i, 2));
    p_i = p_i(:,perm_i);
    
%     Adj = [0 1 1 0 1;
%            1 0 1 1 0;
%            1 1 0 1 0;
%            0 1 1 0 1;
%            1 0 0 1 0];
   % Default fully connected
    Adj = ones(5)-eye(5);
case 3
    % squares with swapped indices
    x_i = [-1 1 1 -1;
           -1 -1 1 1 ];
    p_i = [1 -1 1 -1;
           -1 -1 1 1 ];
    perm_i = randperm(size(p_i, 2));
    p_i = p_i(:,perm_i);
    
    Adj = [0 1 0 1;
           1 0 1 0;
           0 1 0 1;
           1 0 1 0];
case 4
    x_i = [-2.76942602  1.20449776;
        -4.70545649  3.16478552;
        -4.75356765 -0.76411523;
        -4.75139742  1.21944433;
        -6.76305292  1.21700958]';
    p_i = [-2 -4 2 4 0;
            0 0 0 0 0];
    p_i_bar = p_i;
    
    perm = [0, 4, 3, 2, 1]+1;
    p_i = p_i(:,perm);

    
    % Default fully connected
    Adj = ones(5)-eye(5);
%     Adj = [0 1 0 1 0;
%            1 0 1 0 1;
%            0 1 0 1 0;
%            1 0 1 0 1;
%            0 1 0 1 0];
%  
end

%% Run test
[~,n] = size(x_i);

% best alignment & rotation angle in 2D
tau0 = mean(x_i,2) - mean(p_i,2);
W1 = 0;
W2 = 0;
for i = 1:n
    W1 = W1 + (x_i(:,i)-mean(x_i,2))'*(p_i(:,i)-mean(p_i,2));
    W2 = W2 + (x_i(:,i)-mean(x_i,2))'*Rot(pi/2)*(p_i(:,i)-mean(p_i,2));
end
theta0 = atan2(W2, W1);

% run CBAA
[assign_CBAA] = CBAA_aclswarm(Adj, p_i, x_i);
% Evaluate assignment
[dis_old_CBAA, dis_new_CBAA] = eval_assign(assign_CBAA,tau0,theta0,1:n,tau0,theta0, p_i,x_i, true);
assign_CBAA

