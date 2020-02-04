%% Hungarian assignment algorithm
%
% q is 2xN current positions
% p is 2xN given formation points
%
function assign = Hungarian_hexswarm(p, q) 
n = size(p,2); % Number of agents

% Find the alignment of the formation to the swarm
[R, t] = arun(q, p);

% Aligned points
pa = R * p + t;

% Score
S = pdist2(pa', q');

% Hungarian algorithm
assign = Hungarian(S); 
