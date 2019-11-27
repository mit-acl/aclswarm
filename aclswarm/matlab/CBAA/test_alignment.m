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
vehid = 1;
vehid = vehid + 1; % matlab 1-based indexing
assignment = 6;

% Load data from C++ implementation
fname = ['~/.ros/veh' num2str(vehid-1) '_assignment' num2str(assignment) '.bin'];
[n,q,adjmat,P,p,aligned,newP] = read_alignment(fname);
c = struct();
c.P = P;
c.aligned = aligned;
c.newP = newP

% Load data from MATLAB implementation
[newP, aligned_ps] = CBAA_aclswarm(adjmat, p(:,1:2)', q(:,1:2)');
pa = [aligned_ps{vehid}; zeros(1,n)]';
m = struct();
m.P = P;
m.aligned = pa;
m.newP = newP;

% MATLAB Arun implementation of C++
pa = align(vehid, adjmat, q, p, P);

figure(1), clf;
subplot(3,3,[1 2 3 4 5 6]); grid on; hold on;
plotPts(q, 'name','State','labels','show');
plotPts(c.aligned, 'name','C++ Aligned', 'labels','show', 'permvec',c.newP);
plotPts(m.aligned, 'name','MATLAB aligned', 'labels','show', 'permvec',m.newP);
% plotPts(pa, 'name','Arun aligned', 'labels','show', 'permvec',m.newP);
axissq([q;c.aligned;m.aligned], 2);
axis square;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(0,90);
% view(-20,30);

%%%%%%%% Original desired formation (no assignment)
subplot(3,3,7); grid on; hold on; axis square; title('From Operator');
plotPts(p);
axissq(p, 2);

%%%%%%%% Desired formation (after assignment)
subplot(3,3,8); grid on; hold on; title('C++ Assigned');
plotPts(c.aligned, 'permvec', c.newP);
axissq(c.aligned, 2);

%%%%%%%% Desired formation (after assignment)
subplot(3,3,9); grid on; hold on; axis square; title('MATLAB Assigned');
plotPts(m.aligned, 'permvec', m.newP);
axissq(m.aligned, 2);
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
jvec(jvec==0) = [];

% create inverse perm vec: formpt to vehid
jinv(P) = 1:length(P);

% map from formpt to vehidx
nbrvec = jinv(jvec);

pnbrs = p(jvec,:);
qnbrs = q(nbrvec,:);
end

%% Formation Alignment

function aligned = align(vehid, adjmat, q, p, P)

[qnbrs, pnbrs] = nbrsof(vehid, adjmat, q, p, P);

% ASSUME: q, p are nxd but need to be dxn
qq = qnbrs';
pp = pnbrs';

use2D = false;

[~,sQ,~] = svd(qq' - mean(qq'));
[~,sP,~] = svd(pp' - mean(pp'));
sQ = diag(sQ);
sP = diag(sP);
rQ = sum(sQ>0.05*sQ(1));
rP = sum(sP>0.05*sP(1));

if rQ==1
    disp('Line Swarm');
    use2D = true;
elseif rQ==2
    disp('Flat Swarm');
    use2D = true;
elseif rQ==3
    disp('3D Swarm');
end

if rP==1
    disp('Line Formation');
    use2D = true;
elseif rP==2
    disp('Flat Formation');
    use2D = true;
elseif rP==3
    disp('3D Formation');
end

if use2D
    disp('Using 2D Arun');
    qq = qq(1:2,:);
    pp = pp(1:2,:);
end

[R, t] = arun(qq, pp);

if size(R,1) == 2
    R(3,3) = 1;
    t(3) = 0;
end

aligned = (R*p' + t)';
end

%% Arun's Method
% Minimizes ||q - (Rp + t)||^2
function [R, t] = arun(q, p)

% ASSUME: q, p are dxn (d: 2D or 3D)
d = size(q,1);

% shift point clouds by centroids
mu_q = mean(q,2); % (rowwise)
mu_p = mean(p,2); % (rowwise)
Q = q - mu_q;
P = p - mu_p;

% construct H matrix (dxd)
H = Q * P';

% perform SVD of H
[U,~,V] = svd(H);
D = eye(size(H));
D(d,d) = det(U*V');

% solve rotation-only problem
R = U*D*V';

% solve translation
t = mu_q - R*mu_p;
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

function axissq(x, m)
axis square;
axis([min(x(:,1))-m max(x(:,1))+m min(x(:,2))-m max(x(:,2))+m -3 3]);

ax = axis;
cx = (ax(1)+ax(2))/2;
cy = (ax(3)+ax(4))/2;
dx = ax(2)-ax(1);
dy = ax(4)-ax(3);

if dx > dy
    axis([ax(1) ax(2) cy-dx/2 cy+dx/2 -3 3]);
else
    axis([cx-dy/2 cx+dy/2 ax(3) ax(4) -3 3]);
end

end