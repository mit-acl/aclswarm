function vehstruct = readACLBag(vehfun, bagpath)
%READBAG Extracts messages into matrices without using custom msg defn

bag = rosbag(bagpath);

% Identify which vehicles are in the bag
vehregex = vehfun('(?<n>\d+)'); % to select vehicles via bag topics
matches = regexp(bag.AvailableTopics.Row, vehregex, 'tokens');
matches = vertcat(matches{:});
vehs = unique([matches{:}]);

vehstruct = struct();

%
% loop through each vehicle
%
for i = 1:length(vehs)
    % build the vehicle name
    v = vehfun(cell2mat(vehs(i)));
    
    % topic builder lambda
    t = @(topic) [ '/' v '/' topic];
    
    bagsel = select(bag, 'Topic', t('state'));
    msgs = readMessages(bagsel,'DataFormat','struct'); stateMsg=[msgs{:}];
    tsec = header([stateMsg.Header]); tstate = statestamp(stateMsg);
    pos = Vector3Message([stateMsg.Pos]);
    vel = Vector3Message([stateMsg.Vel]);
    quat = QuaternionMessage([stateMsg.Quat]);
    omega = Vector3Message([stateMsg.W]);
    state = struct('t',tsec,'tstate',tstate,'pos',pos,'vel',vel,...
                   'quat',quat,'omega',omega);    

    vehstruct.(v).state = state;
    
    % time sync
    mint = min([state.t]);
    vehstruct.(v).mint = mint;
end

% time sync across all vehicles
mint = min(structfun(@(s)s.mint, vehstruct));
for i = 1:length(vehs)
    % build the vehicle name
    v = vehfun(cell2mat(vehs(i)));

    fn = fieldnames(vehstruct.(v));
    for k = 1:numel(fn)
        if isstruct(vehstruct.(v).(fn{k}))
            vehstruct.(v).(fn{k}).t = vehstruct.(v).(fn{k}).t - mint;
        end
    end
end

end

function t = header(headerMsg)
stamp = [headerMsg.Stamp];
sec = [stamp.Sec];
nsec = [stamp.Nsec];
t = double(sec) + double(nsec)*1e-9;
end

function t = statestamp(stateMsg)
stamp = [stateMsg.StateStamp];
sec = [stamp.Sec];
nsec = [stamp.Nsec];
t = double(sec) + double(nsec)*1e-9;
end

function [pos, quat] = PoseMessage(poseMsg)
pos = Vector3Message([poseMsg.Position]);
quat = QuaternionMessage([poseMsg.Orientation]);
end

function [linvel, angvel] = TwistMessage(twistMsg)
linvel = Vector3Message([twistMsg.Linear]);
angvel = Vector3Message([twistMsg.Angular]);
end

function vec = Vector3Message(vectorMsg)
X = [vectorMsg.X]; Y = [vectorMsg.Y]; Z = [vectorMsg.Z];
vec = [X' Y' Z']';
end

function q = QuaternionMessage(quatMsg)
W = [quatMsg.W]; X = [quatMsg.X]; Y = [quatMsg.Y]; Z = [quatMsg.Z];
q = [W' X' Y' Z']';
end