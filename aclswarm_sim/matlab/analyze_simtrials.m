%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analyze simulation trial data and generate results for aclswarm
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

n = 15;                              % number of agents
m = 10;                              % total number of sim trials
file = 'trials/aclswarm_trials_cafc.csv'; % data file to process

% -------------------------------------------------------------------------
% Load and process data

data = csvread(file);

successful_trials = size(data,1);
completion = 100 * successful_trials / m;
dist = data(:,1:n);
time = data(:,n+1:end);

% average swarm distance traveled
avgdist = mean(dist,2);

% total time btwn formations
total = sum(time(:,1:end),2);

fprintf('Completion: %0.02f %%\n', completion);
fprintf('Average Time: %0.02f seconds\n', mean(total));
fprintf('Average Distance\n');
fprintf('\tmin: %0.02f meters\n', min(avgdist));
fprintf('\tavg: %0.02f meters\n', mean(avgdist));
fprintf('\tstd: %0.02f meters\n', std(avgdist));
fprintf('\tmax: %0.02f meters\n', max(avgdist));