classdef Boid < handle
    properties (GetAccess = public, SetAccess = public)
        arena
        id
        speed
        turning
        targetDirection
        neighbors
        flock
        lines
        circles
        text
    end
    
    methods
        
        function this = Boid(f, id)
            this.flock = f;
            this.arena = f.arena;                % Robotarium
            this.id = id;                        % Numbered from 1 to N
            this.speed = 1;
            this.arena.set_velocities(this.id, [this.speed;0]);
            this.turning = false;
            this.targetDirection = [];
            this.neighbors = [];
            this.lines = gobjects(1,length(f.boids));
            this.lines(:) = line(0,0);
            this.circles = gobjects(1,3); % Rally Ldr, Rori, Ratt     
            this.circles(:) = viscircles([0,0],0);
            this.text = text(0,0,'');
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
                orientationVelocity = orientationVelocity + heading;
            end
            oriVel = orientationVelocity;  % / norm(orientationVelocity);
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
            attVel = attractionVelocity;  % / norm(attractionVelocity);
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
            global ATT_WEIGHT ORI_WEIGHT GOAL_ATT_WEIGHT
            W_att = ATT_WEIGHT;
            W_ori = ORI_WEIGHT;
            W_gatt = GOAL_ATT_WEIGHT;
            locations = f.locations;
            neighborsRep = delta_disk_neighbors(locations, this.id, f.radiusRep);
            neighborsOri = delta_disk_neighbors(locations, this.id, f.radiusOri);
            neighborsAtr = delta_disk_neighbors(locations, this.id, f.radiusAtr);
            
            
            % Trim neighborsRep, neighborsOri, neighborsAtr
            if f.model == 'M'
                for i = length(neighborsRep):-1:1
                    if neighborsRep(i) == 0
                        neighborsRep(i) = [];
                    end
                end
                for i = length(neighborsOri):-1:1
                    if neighborsOri(i) == 0
                        neighborsOri(i) = [];
                    end
                end
            else
                % if an element is in neighborsRep, but not in neighbors,
                % remove it from neighborsRep
                for j = 1:1:size(neighborsRep, 2)
                    if (~ismember(neighborsRep(j), this.neighbors))
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
                    if (~ismember(neighborsOri(j), this.neighbors))
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
                    if (~ismember(neighborsAtr(j), this.neighbors))
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

%           Wall repulsion is given priority over attraction & orientation  
            if norm(wallRepVel) ~= 0
                desiredVelocity = wallRepVel;
            else
                desiredVelocity = (attVelocity * W_att + oriVelocity * W_ori) / (W_ori + W_att);
            end
                      
            % To unicycle dynamics
            if norm(desiredVelocity) == 0
                desiredVelocity = [this.speed; 0];
            else
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
            end
        end
              
        function getCommNeighbors_M(this, f)
            locations = this.flock.locations;
            this.neighbors = delta_disk_neighbors(locations, this.id, f.radiusAtr);
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
            potentialNeighbors = delta_disk_neighbors(locations, this.id, f.vdist);
            if isempty(potentialNeighbors)
                this.neighbors = potentialNeighbors;
                return
            else
                agentAgentDistance = zeros(1, length(potentialNeighbors));

                % Calculate agentAgentDistance for this boid
                for j = 1:length(potentialNeighbors)
                    agentAgentDistance(1, j) = norm(locations(1:2, this.id) -...
                        locations(1:2, potentialNeighbors(j)));
                end
                
                % Calculate agentAgentAngle for this boid
                agentAgentAngle = zeros(1, length(agentAgentDistance));
                for j = 1:length(potentialNeighbors)
                    y = locations(2, potentialNeighbors(j)) - locations(2, this.id);
                    x = locations(1, potentialNeighbors(j)) - locations(1, this.id);
                    theta = atan2(y, x);
                    agentAgentAngle(1, j) = locations(3, this.id)-theta;
                    if agentAgentAngle(1, j) > pi
                        agentAgentAngle(1, j) = agentAgentAngle(1, j) - 2*pi;
                    elseif agentAgentAngle(1, j) < -pi
                        agentAgentAngle(1, j) = agentAgentAngle(1, j) + 2*pi;
                    end
                end
                
                % Remove agents in blindspot from neighbors
                for j = 1:length(agentAgentAngle)
                    if abs(agentAgentAngle(1, j)) > pi - f.blindspot/2
                        potentialNeighbors(1, j) = 0;
                    end
                end
                
                % Occulsion Calculation ----------------------------------
                
                for i = 1:length(potentialNeighbors)
                    for j = 1:length(potentialNeighbors)
                        if agentAgentDistance(j)>agentAgentDistance(i) ...
                            && potentialNeighbors(i)~=0 && potentialNeighbors(j)~=0
                            % i blocks j
                            occulsionAngle = atan2(0.03, agentAgentDistance(i));
                            if abs(agentAgentAngle(i)-agentAgentAngle(j))<abs(occulsionAngle)
                                potentialNeighbors(j) = 0;
                            end
                        end
                    end
                end
                
                %---------------------------------------------------------
                % No obstacle Occlusion
               
                % Clean up removed neighbors
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
            desiredVelocity = this.swarm(f);
            dxi = desiredVelocity;
        end        
    end
end   


