%% Hungarian assignment algorithm
%
% q is 2xN current positions
% p is 2xN given formation points
%
function assign = Hungarian_hexswarm(p, q) 

n = size(p,2); % Number of agents


% R gives the rotation applied to p to align with q
p_pr = p - mean(p, 2);
q_pr = q - mean(q, 2);
H = p_pr*q_pr';
[U,~,V] = svd(H);
R = V*diag([1, det(V*U')])*U'; % Rotation for alignment

t = mean(q, 2) - R * mean(p, 2); % Translation for alignment

% Aligned points
pa = R * p + t;

% Score
S = zeros(n);
for i = 1 : n
    for j = 1 : n
        S(i,j) = norm(pa(:,i) - q(:,j)); % Check order of i,j   !!!!!!!!!!!
    end
end

% Hungarian algorithm
assign = Hungarian(S); 
