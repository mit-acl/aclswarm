
% to reset
% X = []; Y = []

% MIT
X = [39 39 39 39 39 39 41 41 41 41 41 41 43 43 43 43 45 45 45 45 47 47 47 47 47 47 49 49 49 49 49 49 52 52 52 52 52 52 54 54 54 54 54 54 57 57 57 57 57 57 59 59 59 59 59 59 61 61 63 63];
Y = [19 21 23 25 27 29 29 27 25 23 21 19 29 27 25 23 23 25 27 29 29 27 25 23 21 19 19 21 23 25 27 29 29 27 25 23 21 19 19 21 23 25 27 29 29 27 25 23 21 19 19 21 23 25 27 29 29 27 29 27];
Z = [0 2 4 6 8 10 10 8 6 4 2 0 10 8 6 4 4 6 8 10 10 8 6 4 2 0 0 2 4 6 8 10 10 8 6 4 2 0 0 2 4 6 8 10 10 8 6 4 2 0 0 2 4 6 8 10 10 8 10 8];
% A
X = [X   68.0000   78.0000   78.0000   73.0000   70.5000   75.5000   78.0000   78.0000   78.0000   73.0000   70.5000   75.5000   74 76];
Y = [Y   19.0000   19.0000   29.0000   19.0000   19.0000   19.0000   24.0000   26.5000   21.5000   24.0000   21.5000   26.5000   21 23];
Z = [Z   0         0   10.0000         0         0         0    5.0000    7.5000    2.5000    5.0000    2.5000    7.5000    2.0000   4];
% C
Cx = [81   88   88   84   82   84   82     88.0000   84.5000   83.0000   84.5000   88.0000] + 0.5;
X = [X  Cx ];
Y = [Y   24   29   19   19   21   29   27     27.0000   27.0000   24.0000   21.0000   21.0000];
Z = [Z    5    10     0     0     2    10     8     8     8     5     2     2];
% L
Lx = [91.0000   91.0000  101.0000   91.0000   91.0000   91.0000  93.5000   96.0000   98.5000    93.0000   99.0000   95.0000   93.5000   93] + 1;
X = [X   Lx];
Y = [Y   29.0000   19.0000   19.0000   26.5000   24.0000   21.5000  19.0000   19.0000   19.0000    28.0000   21.0000   21.5000   24.5000   21];
Z = [Z   10.0000         0         0    7.5000    5.0000    2.5000         0         0         0    9.0000    2.0000    2.5000    5.5000   2];

n = length(X)
pts = [X;Y;Z];
s = sort(pdist(pts'));
s(1:10)

figure(1), clf;
grid on; hold on;
axis equal
axis([20 120 0 50]); a = axis;
set(gca, 'xtick', a(1):a(2));
set(gca, 'ytick', a(3):a(4));

% plot points from last time
scatter3(X,Y,Z, 'fill');

if 0
    % magic number
    magic = 0.1; % make text appear in center of scatter point

    k=1:n;
    lbl = num2str(k');

    % label each point
    text(X-magic,Y,Z,lbl,...
        'Color','black','FontWeight','bold');
end


% % see help for hints (e.g., backspace removes last click)
% [X,Y] = getpts(gca);
% 
% % plot points
% X = round(X/0.5)*0.5;
% Y = round(Y/0.5)*0.5;
% scatter(X,Y,zeros(1,length(X)), 'fill');

%% Gain design
addpath('Helpers');

% adjList = cell(n,1);
% 
% % connect to kth subsequent nbrs
% k = 20;
% for i = 1:n   
%     adjList{i} = mod(i+(1:k)-1, n+1-1)+1;
% end
% 
% % convert to adj mat
% adj = zeros(n,n);
% for i = 1:n
%     adj(i, adjList{i}) = 1;
% end
% adj = adj + adj';

adj = ones(n,n) - eye(n,n);
% Choose (at most) k edges to remove:
k = 7000;
rowIdx = randi(n, 1,k);
colIdx = randi(n, 1,k);    
for j = 1 : k
    adj(rowIdx(j),colIdx(j)) = 0;
    adj(colIdx(j),rowIdx(j)) = 0;
end

% figure(2), clf;
% G = graph(adj);
% plot(G, 'XData', X, 'YData', Y);
% grid on; hold on;
% axis equal
% axis([20 120 0 50]); a = axis;
% set(gca, 'xtick', a(1):a(2));
% set(gca, 'ytick', a(3):a(4));

tic
Aadmm = ADMMGainDesign3D(pts, adj);
toc

% % Eigenvalues of  the gain matrix
eigAdmm = sort( abs(eig(Aadmm)) );
eigAdmm(1:10)
trace(Aadmm)

%% Write gains
% Need http://vision.is.tohoku.ac.jp/~kyamagu/software/yaml/ in path

% round to precision
A = round(Aadmm,8);

% build struct for YAML dump
% note: doesn't build yaml correctly: need to edit to make 'formations:'
% a list and properly indent.
S2 = struct('name', 'mitacl', 'scale', 1, 'points', pts', 'adjmat', adj,...
            'gains', A);
S1 = struct('agents', n, 'formations', S2);
root = struct('mitacl100', S1);

outfile = ['/tmp/formations.yaml.' num2str(k)];
YAML.write(outfile, root);
