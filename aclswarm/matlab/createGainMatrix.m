%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create Gain Matrix
%
% Can be called externally (e.g., see operator.py) to generate gains.
%
% Parker Lusk
% 10 Feb 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc;
addpath('Helpers');

infile = '/tmp/formation.yaml';
indata = YAML.read(infile);

adj = indata.adj;
qDes = indata.points';

% reun the correct gain design routine
if strcmp(indata.method, 'original')
    addpath('cvx');
    cvx_startup;
elseif strcmp(indata.method, 'sdp')
    addpath('cvx');
    cvx_startup;
elseif strcmp(indata.method, 'admm')
    A = ADMMGainDesign3D(qDes, adj);
end

% round to precision
A = round(A,8);

% build struct for YAML dump
X = struct('gains', A);

outfile = '/tmp/gains.yaml';
YAML.write(outfile, X);