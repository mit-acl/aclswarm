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

m = 10;                              % total number of sim trials
file = 'trials/aclswarm_trials.csv'; % data file to process

% -------------------------------------------------------------------------
% Load and process data

data = csvread(file);

successful_trials = size(data,1);
completion = 100 * successful_trials / m;
dist = data(:,1:end-1);
time = data(:,end);

% average swarm distance traveled
avgdist = mean(dist,2);

fprintf('Completion: %0.02f %%\n', completion);
fprintf('Average Time: %0.02f seconds\n', mean(time));
fprintf('Average Distance\n');
fprintf('\tmin: %0.02f meters\n', min(avgdist));
fprintf('\tavg: %0.02f meters\n', mean(avgdist));
fprintf('\tstd: %0.02f meters\n', std(avgdist));
fprintf('\tmax: %0.02f meters\n', max(avgdist));