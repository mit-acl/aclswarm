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

n = 30;                              % number of agents
m = 100;                              % total number of sim trials
file = 'trials/nafc.csv'; % data file to process

% -------------------------------------------------------------------------
% Load and process data

data = csvread(file);

% the first column is the trial number and can be thrown away
r = 1;

% how many formations are there?
f = size(data(:,r+n+1:end),2) / 3;

successful_trials = size(data,1);
completion = 100 * successful_trials / m;
dist = data(:,r+(1:n)); r = r + n;
time = data(:,r+(1:f)); r = r + f;
coltime = data(:,r+(1:f)); r = r + f;
nrAssigns = data(:,r+(1:f)); r = r + f;

% average swarm distance traveled
avgdist = mean(dist,2);

% total time btwn formations
total = sum(time(:,1:end),2);

% total time in collision avoidance
Tcoltime = sum(coltime(:,1:end),2);

% total number of assignments
TnrAssign = sum(nrAssigns(:,1:end),2);

fprintf('Completion: %0.02f %%\n', completion);
fprintf('Average Time: %0.02f seconds\n', mean(total));
fprintf('\tstd: %0.02f seconds\n', std(total));
fprintf('Average Time in ColAvoid: %0.02f seconds\n', mean(Tcoltime));
fprintf('\tstd: %0.02f seconds\n', std(Tcoltime));
fprintf('Average Num Assignments: %0.02f\n', mean(TnrAssign));
fprintf('\tstd: %0.02f\n', std(TnrAssign));
fprintf('Average Distance\n');
fprintf('\tmin: %0.02f meters\n', min(avgdist));
fprintf('\tavg: %0.02f meters\n', mean(avgdist));
fprintf('\tstd: %0.02f meters\n', std(avgdist));
fprintf('\tmax: %0.02f meters\n', max(avgdist));