classdef Boid < handle
    properties (GetAccess = public, SetAccess = public)
        arena
        id
        speed
        knowsGoal
        foundGoal
        turning
        atGoal
        targetDirection
        neighbors
        flock
        lines
    end
    
    methods
        
        function this = Boid(f, id)
            this.flock = f;
            this.arena = f.arena;        % Robotarium
            this.id = id;                % Numbered from 1 to N
            this.speed = 1;
            this.arena.set_velocities(this.id, [this.speed;0]);
            this.knowsGoal = false;
            this.foundGoal = false;
            this.turning = false;
            this.atGoal = false;
            this.targetDirection = [];
            this.neighbors = [];
            this.lines = [line(0,0) line(0,0) line(0,0) line(0,0) line(0,0) line(0,0) line(0,0) line(0,0) line(0,0)];
        end
        
        function repVel = calculate_repVel(this, locations, neighborsRep)
            repVelocity = zeros(2, 1);
            for i = neighborsRep
                delta = -locations(1:2, i) + locations(1:2, this.id);
                dist = norm(delta);
                delta = delta ./ dist^2;
                % delta = delta ./ norm(delta);
                repVelocity = repVelocity + delta;
            end
            repVel = repVelocity;
        end
        
        function oriVel = calculate_oriVel(this, locations, neighborsOri)
            orientationVelocity = zeros(2, 1);
%             this.id, neighborsOri;
            for i = neighborsOri
                theta = locations(3:3, i);
                delta = -locations(1:2, i) + locations(1:2, this.id);
                dist = norm(delta);
                heading = [cos(theta); sin(theta)];
                orientationVelocity = orientationVelocity + heading; % ./ dist;
            end
            oriVel = orientationVelocity;
        end
        
        function attVel = calculate_attVel(this, locations, neighborsAtr, neighborsRep)
            attractionVelocity = zeros(2, 1);
            sum = zeros(2, 1);
            count = 0;
            for i = neighborsAtr
                if ~ ismember(i, neighborsRep)
                    count = count + 1;
                    sum = sum + locations(1:2, i);
                end
            end
            if count>0
                sum = sum ./ count;
                attractionVelocity = sum - locations(1:2, this.id);
            end
            attVel = attractionVelocity;
        end
        
        function wallRep = calculate_wallRep(this, locations)
            xpos = locations(1, this.id);
            ypos = locations(2, this.id);
            wallRepVel = zeros(2, 1);
            if (xpos + 0.6 < this.flock.wallRepRadius)
                % Wall on left
                dist = xpos + 0.6;
                if dist <= 0
                    dist = 0.0001;
                end
                wallRepVel = wallRepVel + [(1 / dist), 0]';
            elseif (0.6 - xpos < this.flock.wallRepRadius)
                % Wall on right
                dist = 0.6 - xpos;
                if dist <= 0
                    dist = 0.0001;
                end
                wallRepVel = wallRepVel + [(-1 / dist), 0]';
            end
            if (ypos + 0.35 < this.flock.wallRepRadius)
                % Wall on down
                dist = ypos + 0.35;
                if dist <= 0
                    dist = 0.0001;
                end
                wallRepVel = wallRepVel + [0, (1 / dist)]';
            elseif (0.35 - ypos < this.flock.wallRepRadius)
                % Wall on up
                dist = 0.35 - ypos;
                if dist <= 0
                    dist = 0.0001;
                end
                wallRepVel = wallRepVel + [0, (-1 / dist)]';
            end
            wallRep = wallRepVel;
        end
        
        function desiredVelocity = swarm(this, f)
            W_att = 0.5;
            W_ori = 0.5;
            locations = f.locations;
            neighborsRep = delta_disk_neighbors(locations, this.id, f.radiusRep);
            neighborsOri = delta_disk_neighbors(locations, this.id, f.radiusOri);
            neighborsAtr = delta_disk_neighbors(locations, this.id, f.radiusAtr);
            
            % OVUNC: This part is left untouched for now. ----------------
            % Trim neighborsRep, neighborsOri, neighborsAtr
            if f.model == 'M'
                % Ignore Repulsion/Ori if foundGoal
                for j = 1:1:size(neighborsRep, 2)
                    if norm(locations(1:2, neighborsRep(j)) - f.goal) < f.foundGoalRadius
                        neighborsRep(j) = 0;
                    end
                end
                for i = length(neighborsRep):-1:1
                    if neighborsRep(i) == 0
                        neighborsRep(i) = [];
                    end
                end
                
                for j = 1:1:size(neighborsOri, 2)
                    if norm(locations(1:2, neighborsOri(j)) - f.goal) < f.foundGoalRadius
                        neighborsOri(j) = 0;
                    end
                end
                for i = length(neighborsOri):-1:1
                    if neighborsOri(i) == 0
                        neighborsOri(i) = [];
                    end
                end
            else
                % Below is for Topological and Visual
                % if an element is in neighborsRep, but not in neighbors,
                % remove it from neighborsRep
                for j = 1:1:size(neighborsRep, 2)
                    %Is neighborsRep(1, j) in the list of neighbors?
                    if (~ismember(neighborsRep(j), this.neighbors))
                        %No, remove it from neighborsRep
                        neighborsRep(j) = 0;
                    elseif norm(locations(1:2, neighborsRep(j)) - f.goal) < f.foundGoalRadius
                        neighborsRep(j) = 0;
                    end
                end
                for i = length(neighborsRep):-1:1
                    if neighborsRep(i) == 0
                        neighborsRep(i) = [];
                    end
                end
                %if an element is in neighborsOri, but not in neighbors,
                %remove it from neighborsOri
                for j = 1:1:size(neighborsOri, 2)
                    %Is neighborsRep(1, j) in the list of neighbors?
                    if (~ismember(neighborsOri(j), this.neighbors))
                        %No, remove it from neighborsRep
                        neighborsOri(j) = 0;
                    elseif norm(locations(1:2, neighborsOri(j)) - f.goal) < f.foundGoalRadius
                        neighborsOri(j) = 0;
                    end
                end
                for i = length(neighborsOri):-1:1
                    if neighborsOri(i) == 0
                        neighborsOri(i) = [];
                    end
                end
                %if an element is in neighborsAtr, but not in neighbors,
                %remove it from neighborsAtr
                for j = 1:1:size(neighborsAtr, 2)
                    %Is neighborsRep(1, j) in the list of neighbors?
                    if (~ismember(neighborsAtr(j), this.neighbors))
                        %No, remove it from neighborsRep
                        neighborsAtr(j) = 0;
                    end
                end
                if numel(neighborsAtr) > 0
                    for i = length(neighborsAtr):-1:1
                        if neighborsAtr(i) == 0
                            neighborsAtr(i) = [];
                        end
                    end
                end
            end
            
            % -------------------------------------------------------------
            
            locations = f.locations;

            % Repulsion
            repVelocity = this.calculate_repVel(locations, neighborsRep);

            % Orientation
            oriVelocity = this.calculate_oriVel(locations, neighborsOri);

            % Attraction
            attVelocity = this.calculate_attVel(locations, neighborsAtr, neighborsRep);
            
            % Wall Repulsion
            wallRepVel = this.calculate_wallRep(locations);

            % Final Calculation
%             desiredVelocity = orientationVelocity + attractionVelocity...
%                 + wallRepVel + repVelocity; % + noise + repVelocity
% 
%             if ~norm(orientationVelocity) == 0
%                 orientationVelocity = orientationVelocity ./ norm(orientationVelocity) ./ 3;
%             end
%             if ~norm(attractionVelocity) == 0
%                 attractionVelocity = attractionVelocity ./ norm(attractionVelocity) ./ 3;
%             end
%             if ~norm(wallRepVel) == 0
%                 wallRepVel = wallRepVel ./ norm(wallRepVel) ./ 3;
%             end

%           Wall repulsion is given priority over attraction & orientation  
            if norm(wallRepVel) ~= 0
                desiredVelocity = wallRepVel;
            else
                desiredVelocity = attVelocity * W_att + oriVelocity * W_ori;
            end
            
            % Visualization -----------------------------------------------
%             delete(this.vecline1)
%             if norm(desiredVelocity) ~= 0
%                 normd = desiredVelocity;
%                 xpos = locations(1, this.id);
%                 ypos = locations(2, this.id);
%                 x = [xpos, xpos + normd(1)*0.1];
%                 y = [ypos, ypos + normd(2)*0.1];
%                 this.vecline1 = line(x, y);
%             end
%             delete(this.lines(1))
%             if norm(oriVelocity) ~= 0
%                 normd = oriVelocity;
%                 xpos = locations(1, this.id);
%                 ypos = locations(2, this.id);
%                 x = [xpos, xpos + normd(1)*0.5];
%                 y = [ypos, ypos + normd(2)*0.5];
%                 this.lines(1) = line(x, y, 'Color', 'y');
%             end
%             delete(this.lines(2))
%             if norm(attVelocity) ~= 0
%                 normd = attVelocity;
%                 xpos = locations(1, this.id);
%                 ypos = locations(2, this.id);
%                 x = [xpos, xpos + normd(1)*0.5];
%                 y = [ypos, ypos + normd(2)*0.5];
%                 this.lines(2) = line(x, y,'Color', 'r');
%             end


            % -------------------------------------------------------------
            
            % -------------------------------------------------------------
            
            % To unicycle
            if norm(desiredVelocity) == 0
                desiredVelocity = [this.speed; 0];
                % this.arena.set_velocities(this.id, [this.speed; 0]);
            else
                % desiredVelocity = desiredVelocity ./ norm(desiredVelocity);
                desiredAngle = atan2(desiredVelocity(2, 1),desiredVelocity(1, 1));
                desiredAngle = desiredAngle - locations(3:3, this.id);
                desiredAngle = mod(desiredAngle, 2*pi);
                if desiredAngle > pi
                    desiredAngle = desiredAngle - (2 * pi);
                elseif desiredAngle < -pi
                    desiredAngle = desiredAngle + (2 * pi);
                end
                angularVelocity = desiredAngle * this.arena.time2iters(1);
                desiredVelocity = [this.speed; angularVelocity];
                % desiredVelocity = desiredVelocity ./ norm(desiredVelocity);
            end
        end
        
        function result = sensesGoal(this, goal, senseGoalRadius)
            if this.knowsGoal
                result = true;
            else
                locations = this.flock.locations;
                result = (norm(locations(1:2, this.id) - goal)) < senseGoalRadius;
            end
        end
        
        function getCommNeighbors_M(this, f)
            locations = this.flock.locations;
            this.neighbors = delta_disk_neighbors( locations,this.id, f.radiusAtr);
        end
        
        function getCommNeighbors_T(this, f, locations)
            agentAgentDistance = zeros(1, size(locations, 2));
            for j = 1:1:size(locations, 2)
                %Calulate my distance to everyone else
                agentAgentDistance(1, j) = norm(locations(1:2, this.id) - locations(1:2, j));
            end
            %Sort my distances to everyone else in ascending order and
            %note the indices
            [~, agentAgentDistanceIndices] = sort(agentAgentDistance, 'ascend');
            %Ignore self, and keep the next nTop agents
            this.neighbors = agentAgentDistanceIndices(1, 2:1:f.nTop + 1);
        end
        
        function getCommNeighbors_V(this, f, locations)
            potentialNeighbors = delta_disk_neighbors(locations, this.id, f.communicationRadius);
            if isempty(potentialNeighbors)
                this.neighbors = potentialNeighbors;
            else

                agentAgentDistance = zeros(1, length(potentialNeighbors));

                for j = 1:length(potentialNeighbors)

                    agentAgentDistance(1, j) = norm(locations(1:2, this.id) -...
                        locations(1:2, potentialNeighbors(1, j)));
                end

                [~, agentAgentDistanceIndices] = sort(agentAgentDistance);
                potentialNeighbors = agentAgentDistanceIndices;
                agentAgentAngle = zeros(1, length(agentAgentDistance));
                for j = length(agentAgentAngle)
                    y = locations(2, j) - locations(2, this.id);
                    x = locations(1, j) - locations(1, this.id);
                    theta = atan2(y, x);
                    if theta < 0
                        theta = theta + (2 * pi);
                    end
                    agentAgentAngle(1, j) = theta;
                end
                if locations(3, this.id) < pi

                    direction = locations(3, this.id) + pi;
                else
                    direction = locations(3, this.id) - pi;
                end
                for j = 1:length(agentAgentAngle)
                    if abs(direction - agentAgentAngle(1, j)) < f.blindspot
                        potentialNeighbors(1, j) = 0;
                    end
                end
                for j = 2:length(agentAgentAngle)
                    for i = 1:(j - 1)
                        if ((2 * pi - agentAgentAngle(1, j) +...
                                agentAgentAngle(1, i)) < 0.1396) ||...
                                ((2 * pi - agentAgentAngle(1, i) +...
                                agentAgentAngle(1, j)) < 0.1396)
                            potentialNeighbors(1, j) = 0;
                        elseif abs(agentAgentAngle(1, j) - agentAgentAngle(1, i)) < 0.1396
                            potentialNeighbors(1, j) = 0;
                        end
                    end                                          
                end
                if ~isempty(f.obstacles)
                    agentObstacleDistance = zeros(1, length(f.obstacles));
                    agentObstacleAngle = zeros(1, length(f.obstacles));
                    for i = 1:size(f.obstacles, 2)                           
                        agentObstacleDistance(1, i) = norm(f.obstacles(:, i) -...
                            locations(1:2, this.id));
                        y = f.obstacles(2, i) - locations(2, this.id);
                        x = f.obstacles(1, i) - locations(1, this.id);
                        theta = atan2(y, x);
                        if theta < 0
                            theta = theta + (2 * pi);
                        end
                        agentObstacleAngle(1, i) = theta;
                    end
                    for j = 1:length(potentialNeighbors)
                        n = 1;
                        while potentialNeighbors(1, j) ~= 0 && n <= length(f.obstacles)
                            if (abs(agentAgentAngle(1, j) -...
                                    agentObstacleAngle(1, n)) < 0.1396) &&...
                                    agentObstacleDistance(1, n) < agentAgentDistance(1, j)
                                potentialNeighbors(1, j) = 0;
                            end
                            n = n + 1;    
                        end
                    end                        
                end

                for i = length(potentialNeighbors):-1:1
                    if potentialNeighbors(1, i) == 0
                        potentialNeighbors(i) = [];
                    end
                end
                this.neighbors = potentialNeighbors;
            end
        end
        
        function getCommNeighbors(this, f)
            locations = this.flock.locations;
            if f.model == 'M'
                this.getCommNeighbors_M(f)
            elseif f.model == 'T'
                this.getCommNeighbors_T(f, locations)
            else
                this.getCommNeighbors_V(f, locations)
            end
        end
        
        function dxi = run(this, f)
            locations = this.flock.locations;
            if (norm(locations(1:2, this.id) - f.goal) < f.senseGoalRadius)
                this.knowsGoal = true;
            end
            desiredVelocity = this.swarm(f);
            dxi = desiredVelocity;
        end        
    end
end   


