%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analyze simulation trial data and generate a row of results for aclswarm
%
%
% Parker Lusk
% 7 Jan 2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc;
set(0,'DefaultLineLineWidth',1); % 0.5
exfig = 0;

% -------------------------------------------------------------------------
% Setup

n = 15;                         % number of vehicles simulated
m = 10;                         % number of sim trials to process (1..m)
bagpath = './trials/';          % directory of bags
bagprefix = 'mitacl15_afc_';    % prefix of bags to process
vehfun = @(n) ['SQ' n 's'];     % how to build a vehname topic

% signal smoothing
tau = 2;        % time constant [s] for LPF

% initialize variables
dist = zeros(m,n);


% -------------------------------------------------------------------------
% Load and process data

for trial = 1:m
    % build bag name with zero-padded number
    zptrial = sprintf(['%0' num2str(length(num2str(65))) 'd'], trial);
    bagname = [bagpath bagprefix zptrial '.bag'];
    
    % Load flight data
    fprintf('Loading trial %d\n', trial);
    vehs = readACLBag(vehfun, bagname);
    
    vname = fieldnames(vehs);
    for i = 1:numel(vname)       
        % calculate how many samples to smooth over
        k = ceil(tau/mean(abs(diff(vehs.(vname{i}).state.t))));

        % smooth over translation
        d = sum(abs(diff(movmean(vehs.(vname{i}).state.pos',k,'Endpoints',0))));
        dist(trial, i) = norm([d(1) d(2)]);
    end    
end

% -------------------------------------------------------------------------
% Calculate statistics