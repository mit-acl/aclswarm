%% Distributed collision avoidacen by rotating the control vectors
%
%
%
function u = ColAvoid(ctrl, qm, par)

n           = par.n;                  % Number of agents
dcoll       = par.dcoll;              % Collision avaoidance distance 
rcoll       = par.rcoll;              % Collision avaoidance circle radius


%% Inter-agent distance matrix

Dc = zeros(n,n);
for i = 1 : n
    for j = i+1 : n
        Dc(i,j) = norm(qm(:,i)-qm(:,j),2);
    end
end
Dc = Dc + Dc';


%% Collision avoidance

% Index of neighbors inside collision radius
colIdx = Dc < dcoll;
colIdx = colIdx - diag(ones(1,n));

% Stop flag to avoid collision
stopFlag = false(n,1);

for i = 1 : n % Agent
        
    coneAng = []; % Angle of cone sides are stored in columns of 'coneAng'

    % Find cone angles
    for k = 1 : n % Neighbors
        if colIdx(i,k)  % If collision avoidance is needed
            
            dnb = Dc(i,k);                        % Distance to neighbor
            vec = qm(:,k) - qm(:,i);              % Vector from agent to its neighbor
            tht = atan2d(vec(2), vec(1));         % Angle of connecting vector             
            
            % 'alp' is the vertex half-angle of the collision cone
            if dnb <= dcoll
                alp = 90;
            else
                alp = abs( asind(rcoll/dnb) );    
            end                        
            
            % Angle of cone sides
            thtm = tht - alp;
            thtp = tht + alp;
            
            % Bring all angles to the range [-180, 180] degrees
            if thtm < -180
                coneAng = [coneAng; [wrapTo180(thtm), 180] ];
                coneAng = [coneAng; [-180, thtp] ];
            elseif thtp > 180
                coneAng = [coneAng; [-180, wrapTo180(thtp)] ];
                coneAng = [coneAng; [thtm, 180] ];
            else
                coneAng = [coneAng; [thtm, thtp] ];
            end

        end
    end
    
    if any(colIdx(i,:))  % If collision avoidance is needed

        % Control vector angle in world coordinate frame
        thtC = atan2d(ctrl(2,i), ctrl(1,i));  

        % If control vector is inside a cone change its direction 
        if any( and((thtC >= coneAng(:,1)), (thtC <= coneAng(:,2))) ) 

            angs = [-180 : 5 : 180];    % Possible motion directions to test  
            angsIdx = true(size(angs)); % Index of angles outside of the collision cones

            % Determine which angles are inside the collision cones
            for k = 1 : length(angs)
                r = angs(k);
                if any( and((r >= coneAng(:,1)), (r <= coneAng(:,2))) )
                    angsIdx(k) = false;
                end
            end

            angsFeas = angs(angsIdx);  % Feasible directions to take

            % If there is no feasible angle stop
            if isempty(angsFeas) 
                stopFlag(i) = true;
            end

            % Find closest non-colliding control direction
            thtDiff = abs( wrapTo180(thtC - angsFeas) );
            [~,minIdx] = min(thtDiff);
            thtCnew = angsFeas(minIdx);

            % Check if the feasible control direction is within +-90 degrees,
            % otherwise stop
            if abs( wrapTo180(thtCnew - thtC) ) >= 90 
                stopFlag(i) = true;
            end

            % Modified control vector
            if stopFlag(i)
                ctrl(:,i) = zeros(2,1);
            else
                ctrl(:,i) = norm(ctrl(:,i)) * [cosd(thtCnew); sind(thtCnew)]; 
            end

        end
    end
    
end


% Control direction after collision avoidance modifications
u = ctrl(:);
