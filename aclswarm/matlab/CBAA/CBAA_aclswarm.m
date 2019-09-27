function [assign] = CBAA_aclswarm(Adj, p_i, x_i)
[~, N] = size(x_i);
G = Adj + eye(N);  % Assume that each agent is its own neighbor

% compute the R and t for agents
Rs = cell(1,N);
ts = cell(1,N);
% compute aligned formations for agents
aligned_ps = cell(1,N);
for i = 1:N
    % get relevant agent and formation coordinates
    nbr = get_nbr(G, i);
    ps = p_i(:,nbr);
    xs = x_i(:,nbr);
    % compute R and t
    [R] = get_R(xs, ps);
    t = mean(xs, 2) - R * mean(ps, 2);
    % store R and t
    Rs{i} = R;
    ts{i} = t;
    aligned_ps{i} = R * p_i + t;
end

% Task list, price list, done list. 
N_task = N;
task_ij = zeros(N,N_task);  % agent i's understanding on whether task j is assigned to i
price_ij = zeros(N,N_task);  % agent i's understanding on the highest price of task j
who_ij = zeros(N,N_task);   % log who was associated the market price

% Helper that computes c_ij
eval_c_ij = @(x_i, p_j) 1/(norm(x_i-p_j));
c_ij = zeros(N);

% CBAA
max_iters = N*(N-1);  % N*D, where D is the diameter of G
for iters = 1:max_iters
    % SELECT TASK, if agent does not have any assignment
    for i = 1:N
        % For agent i
        if sum(task_ij(i,:))==0
            % Compute capabilities
            for j = 1:N_task
                c_ij(i,j) = eval_c_ij(x_i(:,i), aligned_ps{i}(:,j));
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
    for i = 1:N
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
        price_J_i = zeros(N,1);
        price_J_i(nbr) = received_price_ij(:,J_i);
        price_J_i(i) = price_ij(i,J_i);  % add in the updated price of agent i
        who_J_i = zeros(N, 1);
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
assign = zeros(1, N);
for i = 1:N
    assign(i) = find(task_ij(i,:));
end

end

function [R] = get_R(q, p)
% q is 2xN current positions
% p is 2xN given formation points
% R gives the rotation applied to p to align with q
p_pr = p - mean(p, 2);
q_pr = q - mean(q, 2);

H = p_pr*q_pr';
[U,~,V] = svd(H);
R = V*diag([1, det(V*U')])*U';
end

function [nbr] = get_nbr(Adj, i)
nbr = find(Adj(i,:));
end