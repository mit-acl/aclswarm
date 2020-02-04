function [assign,aligned_ps] = CBAA_aclswarm(adj, pm, qm)
%CBAA_ACLSWARM CBAA implementation for swarm assignment
%   adj     nxn adjacency matrix
%   pm      3xn matrix of desired formation points
%   qm      3xn matrix of current vehicle positions
%
%   assign  1xn permutation vector. Maps idx of qm to idx of pm (formpt)
%
%   Note: adj and qm should reflect the current assignment. In particular,
%   this is imperative for the alignment stage (Arun's method requires
%   point correspondences). This implementation (cf. C++ impl) also uses
%   this permuted state (qm, adj) in the assignment stage. Thus, the
%   resulting assignment maps from the indices of qm to formation points
%   (the indices of pm). But note that "the indices of qm" does not refer
%   to vehicles, as qm is already a permuted version of the vehicles.
%   Therefore, to recover the permutation from vehicles to formpts, we must
%   accumulate the permutation by composing this new permutation with the
%   permutation used to permute (adj, qm). This is illustrated below.
%
%   Using row representation, let sigma2 represent the new permutation. Let
%   sigma1 be the permutation used to permute the input data, adj and qm:
%
%       sigma2 = CBAA(p, q1)
%
%   where q1 = Qsigma1*q0 and q0 is in the original vehicle order. Since we
%   assumed row representation, permutation matrices compose naturally and
%   we can recover the desired permutation (mapping vehidx to formpt) with
%   Q = Qsigma2*Qsigma1.

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
    qs = qm(:,nbr); % n.b., this assumes qm is already ordered by formpt

    % compute R and t
    [R, t] = arun(qs, ps);

    % store R and t
    Rs{i} = R;
    ts{i} = t;

    % store the aligned formation for UAV i
    aligned_ps{i} = R * pm + t;
end

%% Assignment
% Task list, price list, done list. 
N_task = n;
tasks  = zeros(n,N_task); % agent i's understanding on whether task j is assigned to i
prices = zeros(n,N_task); % agent i's understanding on the highest price of task j
whose  = zeros(n,N_task); % log who was associated with the market price

% Helper that computes c, the bid table
eval_c_ij = @(q_i, p_j) 1/(norm(q_i-p_j));
c = zeros(n);

% CBAA
max_iters = n*(n-1);  % n*D, where D is the diameter of G
for iter = 1:max_iters

    %
    % SELECT TASK, if agent does not have any assignment
    %

    % for each formation point, i (and therefore the corresponding vehicle)
    for i = 1:n

        % if the vehicle at formation point i already has a task, just skip
        if sum(tasks(i,:)) ~= 0, continue; end

        % Compute the veh at formpt i's bid price for each task/formpt
        for j = 1:N_task
            c(i,j) = eval_c_ij(qm(:,i), aligned_ps{i}(:,j));
        end

        % Mask the tasks for which we can bid higher than anyone else
        h_ij = c(i,:) >= prices(i,:);
        if any(h_ij)
            % get the new assignment
            [new_max_price, j] = max(h_ij.*c(i,:));

            % indicate that the veh at formpt i claims formpt j
            tasks(i,j) = 1;

            % log the price that veh at formpt i is willing to pay for j
            prices(i,j) = new_max_price;

            % associate the veh at formpt i with the price for formpt j
            whose(i,j) = i; % NOTE: this is what defines the meaning of
                            % the output assignment. This means that
                            % 'whose' is a permutation that maps formpt j
                            % to the vehicle at formpt i (i.e., you must
                            % undo the last permutation, or, accumulate).
        end
    end

    %
    % UPDATE TASK, all agents have to constantly do this
    %

    % for each formation point, i (and therefore the corresponding vehicle)
    for i = 1:n
        % who are the neighbors of formation point i?
        nbr = get_nbr(G, i);

        % === simulate communication ======================================
        received_price_ij = prices(nbr,:);
        received_who_ij = whose(nbr,:);
        % =================================================================

        % which of my nbrs (or me, formpt i) bid the most for each task?
        [max_prices, sender_indices] = max(received_price_ij);
        who_max_prices = zeros(size(max_prices));
        for j = 1:N_task
            who_max_prices(j) = received_who_ij(sender_indices(j), j);
        end

        % update agent i's local prices, and labels for whose prices,
        % through max-consensus among nbrs
        prices(i,:) = max_prices;
        whose(i,:) = who_max_prices;

        %
        % Check if the vehicle at formpt i was outbid
        %

        % which formation point did this veh/formpt previously claim?
        j = find(tasks(i,:));

        % check if formpt i was outbid. Otherwise update i's assignment
        price_j = zeros(n,1);
        price_j(nbr) = received_price_ij(:,j);
        price_j(i) = prices(i,j);  % add in the updated price of agent i
        who_j = zeros(n, 1);
        who_j(nbr) = received_who_ij(:,j);
        who_j(i) = whose(i, j);  % add in updated price setters based on agent i's local info

        % get who has the highest price. 
        % Other agents with the same highest prices should have the same 
        % label for who set the highest price
        [~, who_won] = max(price_j);
        % get who set the highest price originally
        whose_price = who_j(who_won);

        % if who set the highest price is not agent i originally, then
        % reset agent i's task assignment, because other agents (more
        % specifically agent indexed by "whose_price") should get this
        % assignment.
        if whose_price ~= i
            tasks(i,j) = 0;
        end
    end
end

% Note: `tasks` is a permutation matrix that maps the index of pm into the
% index of qm. We would like a permutation matrix that maps the index of qm
% into the the index of pm (i.e., the desired formation). Thus, transpose.
tasks = tasks';

% Extract assignment permutation. Maps index of qm to index of pm. Use this
% to keep pm fixed and reorder qm so that qm(k) <--> pm(k) correspond.
assign = zeros(1, n);
for j = 1:n
    % This iterates across columns, searching for which row has a 1;
    % i.e., this is creating a row representation permutation matrix.
    % See https://en.wikipedia.org/wiki/Permutation_matrix.
    assign(j) = find(tasks(:,j));
end

end

%% get_nbr
% Determine the neighbors of vehicle i
function nbr = get_nbr(adj, i)
nbr = find(adj(i,:));
end