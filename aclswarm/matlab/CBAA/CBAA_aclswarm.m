function [assign,aligned_ps] = CBAA_aclswarm(adj, pm, qm)
%CBAA_HEXSWARM CBAA implementation for swarm assignment
%   adj     nxn adjacency matrix
%   pm      2xn matrix of desired formation points
%   qm      2xn matrix of current vehicle positions
%
%   assign  1xn permutation vector. maps vehidx to formpt
%
%   Note: adj and pm should reflect the current assignment.

n = size(pm, 2);    % number of agents
G = adj + eye(n);   % Assume that each agent is its own neighbor

%% Alignment

% compute the R and t for agents
Rs = cell(1,n);
ts = cell(1,n);

% compute aligned formations for agents
aligned_ps = cell(1,n);

% Each agent uses its local information (based on graph connectivity) to
% align the desired formation points onto the current vehicle positions.
% This happens on each vehicle independently of each other.
for i = 1:n
    % determine who UAV i's neighbors are (including itself)
    nbr = get_nbr(G, i);

    % Current UAV positions and the formation points of UAV i's neighbors.
    ps = pm(:,nbr);
    qs = qm(:,nbr);

    % compute R and t
    [R, t] = align(qs, ps);

    % store R and t
    Rs{i} = R;
    ts{i} = t;

    % store the aligned formation for UAV i
    aligned_ps{i} = R * pm + t;
end

%% Assignment
% Task list, price list, done list. 
N_task = n;
task_ij  = zeros(n,N_task);  % agent i's understanding on whether task j is assigned to i
price_ij = zeros(n,N_task);  % agent i's understanding on the highest price of task j
who_ij   = zeros(n,N_task);   % log who was associated with the market price

% Helper that computes c_ij
eval_c_ij = @(q_i, p_j) 1/(norm(q_i-p_j));
c_ij = zeros(n);

% CBAA
max_iters = n*(n-1);  % n*D, where D is the diameter of G
for iters = 1:max_iters
    % SELECT TASK, if agent does not have any assignment
    for i = 1:n
        % For agent i
        if sum(task_ij(i,:))==0
            % Compute capabilities
            for j = 1:N_task
                c_ij(i,j) = eval_c_ij(qm(:,i), aligned_ps{i}(:,j));
            end
            % Get mask for biddable tasks
            h_ij = c_ij(i,:) >= price_ij(i,:);
            if sum(h_ij)>0
                % get the new assignment
                [new_max_prices, J_i] = max(h_ij.*c_ij(i,:));
                task_ij(i,J_i) = 1;
                price_ij(i,J_i) = new_max_prices;
                who_ij(i,J_i) = i;  % associate agent i with the price
            end
        end
    end
    
    % UPDATE TASK, all agents have to constantly do this
    sent_price_ij = price_ij;
    sent_who_ij = who_ij;
    for i = 1:n
        nbr = get_nbr(G, i);
        received_price_ij = sent_price_ij(nbr,:);
        received_who_ij = sent_who_ij(nbr,:);
        [max_prices, sender_indices] = max(received_price_ij);
        who_max_prices = zeros(size(max_prices));
        for k = 1:N_task
            who_max_prices(k) = received_who_ij(sender_indices(k), k);
        end
        
        % update agent i's local prices, and labels for whose prices, through max-consensus among nbrs
        price_ij(i,:) = max_prices;
        who_ij(i,:) = who_max_prices;
        
        % look at i's assignment and see if others can outbid i
        J_i = find(task_ij(i,:));
        
        % check whether agent i still wins, otherwise update i's assignment
        price_J_i = zeros(n,1);
        price_J_i(nbr) = received_price_ij(:,J_i);
        price_J_i(i) = price_ij(i,J_i);  % add in the updated price of agent i
        who_J_i = zeros(n, 1);
        who_J_i(nbr) = received_who_ij(:,J_i);
        who_J_i(i) = who_ij(i, J_i);  % add in updated price setters based on agent i's local info
        
        % get who has the highest price. 
        % Other agents with the same highest prices should have the same 
        % label for who set the highest price
        [~, who_won] = max(price_J_i);  
        % get who set the highest price originally
        whose_price = who_J_i(who_won);
        
        % if who set the highest price is not agent i originally, then
        % reset agent i's task assignment, because other agents (more
        % specifically agent indexed by "whose_price" should get this
        % assignment.
        if whose_price ~= i
            task_ij(i,J_i) = 0;
        end
    end
end

% extract assignment
assign = zeros(1, n);
for i = 1:n
    assign(i) = find(task_ij(i,:));
end

end

%% align
% Use Arun's method to align 2d points with known correspondence
function [R, t] = align(q, p)
% q is 2xN current positions
% p is 2xN given formation points
% R gives the rotation applied to p to align with q
p_pr = p - mean(p, 2);
q_pr = q - mean(q, 2);

H = p_pr*q_pr';
[U,~,V] = svd(H);
R = V*diag([1, det(V*U')])*U';

% recover optimal translation
t = mean(q, 2) - R * mean(p, 2);
end

%% get_nbr
% Determine the neighbors of vehicle i
function [nbr] = get_nbr(adj, i)
nbr = find(adj(i,:));
end