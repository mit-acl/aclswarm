% p = [[ 0  0  0];
%      [ 1  1  0];
%      [-1  1  0];
%      [ 0 -1  0];
%      [ 0 -2  0]];
% 
% rng(1);
% R = eul2rotm([pi/2 0 pi/4], 'ZYX')
% a = -2; b = 2;
% t = a + (b-a).*rand(3,1)
%  
% q = (R*p' + t)';
% 
% figure(1), clf;
% scatter3(p(:,1),p(:,2),p(:,3));
% hold on;
% scatter3(q(:,1),q(:,2),q(:,3));
% axis([-3 3 -3 3 -3 3]);
% xlabel('X'); ylabel('Y'); zlabel('Z');
% view(0,90);
% 
% paligned = arun(q, p);
% scatter3(paligned(:,1),paligned(:,2),paligned(:,3));

%% Using logged data
vehid = 0;
[n,q,adjmat,P,p,aligned,newP] = read_alignment(['~/.ros/alignment_' num2str(vehid) '.bin']);
vehid = vehid + 1; % matlab

[qnbrs, pnbrs] = nbrsof(vehid, adjmat, q, p, P);

% figure
figure(vehid), clf;
subplot(3,2,[1 2]); grid on; hold on;
title(['Vehicle ' num2str(vehid)]);
plotPts(q, 'name','State');
plotPts(aligned, 'name','C++ Aligned', 'permvec',newP);
axis([-5 5 -2 2 -3 3]);
% axis square;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(0,90);

%%%%%%%% MATLAB Arun
pa = arun(q, p, 1);
subplot(3,2,[1 2 3 4]); grid on; hold on;
plotPts(q, 'name','State');
plotPts(aligned, 'name','C++ Aligned', 'permvec',newP);
% plotPts(pa, 'name','MATLAB aligned', 'permvec',newP);
axis([-6 6 -6 6 -3 3]);
axis square;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(0,90);

%%%%%%%% Original desired formation (no assignment)
subplot(3,2,5); grid on; hold on; axis square
plotPts(p, 'name','Desired Formation');
axis([-6 6 -6 6 -3 3]);

%%%%%%%% Desired formation (after assignment)
subplot(3,2,6); grid on; hold on; axis square
plotPts(aligned, 'name','Assigned Formation', 'permvec', newP);
axis([-6 6 -6 6 -3 3]);
%% Select neighbors
% use only local information for alignment

function [qnbrs, pnbrs] = nbrsof(vehid, adjmat, q, p, P)
% add myself
adjmat = adjmat + eye(size(adjmat));

% work in "formation space" --- which formation pt am i?
i = P(vehid);

% figure out who my neighbors are
mynbrs = adjmat(i,:);

% index of each formation pt that is my nbr
jvec = mynbrs.*(1:size(adjmat,1));

% create inverse perm vec: formpt to vehid
jinv(P) = 1:length(P);

% map from formpt to vehidx
nbrvec = jinv(jvec);

pnbrs = p(jvec,:);
qnbrs = q(nbrvec,:);
end

%% Arun's Method (3d)
% Minimizes ||q - (Rp + t)||^2
function aligned = arun(q, p, only2d)

% ASSUME: q, p are Nx3 but need to be 3xN
qq = q';
pp = p';

% shift point clouds by centroids
mu_q = mean(qq,2); % (rowwise)
mu_p = mean(pp,2); % (rowwise)
Q = qq - mu_q;
P = pp - mu_p;

% construct H matrix (3x3)
H = Q * P';

% perform SVD of H
[U,~,V] = svd(H);
d = [1, 1, det(U*V')];

% solve rotation-only problem
Rhat = U*diag(d)*V'

if only2d
    % extract only yaw
    eul = rotm2eul(Rhat, 'ZYX');
    Rhat = eul2rotm([eul(1) 0 0], 'ZYX');
end

% solve translation
that = mu_q - Rhat*mu_p

aligned = (Rhat*p' + that)';
end

%% Plotting helpers

function plotPts(p, varargin)
% assume: p is Nx3

ip = inputParser;
ip.addParameter('name','');
ip.addParameter('permvec',[]);
ip.addParameter('labels','show');
ip.parse(varargin{:});

% If a permutation matrix was provided, then relabel pts
if ~isempty(ip.Results.permvec)
    P = ip.Results.permvec;
    p = p(P,:);
end

h = scatter3(p(:,1),p(:,2),p(:,3),400,'filled');

if ~isempty(ip.Results.name)
   set(h,'DisplayName',ip.Results.name);
   legend;
end

if strcmp(ip.Results.labels, 'show')
    % magic number
    magic = 0.1; % make text appear in center of scatter point

    % label each point
    k=1:size(p,1);
    text(p(:,1)-magic,p(:,2),p(:,3),num2str(k'),...
        'Color','black','FontWeight','bold');
end
end