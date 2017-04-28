classdef Flock < handle
    properties (GetAccess = public, SetAccess = public)
        boids
        arena
        radiusRep
        radiusOri
        radiusAtr
        wallRepRadius
        communicationRadius
        initPos
        obstaclePos
        obstacleHandle
        numObstacles
        nTop
        model
        fileID
        blindspot
        locations
        barrierCert
        unibarrierCert
        si2uni
        positionCont
    end
    
    methods
        % Constructor
        function this = Flock(r, numBoids)
            this.arena = r;
            for i = 1:numBoids
                this.boids = [this.boids; Boid(this,  i)];
            end
        end
        
        function updateLocations(this)
            this.locations = this.arena.get_poses();
        end
        
        % The goal is having 90% of agents passing the obstacle
        function goalReached = check_goal(this)
            N = length(this.boids);
            count = 0;
            x = this.locations;
            x = x(1:2, :);
            for i = 1:N
                x(1,i)
                this.obstaclePos
                if x(1, i) > this.obstaclePos(:, 1) + 0.2
                    count = count + 1;
                end
            end
            if count / N > 0.9
                goalReached = true;
            else
                goalReached = false;
            end
        end
        
        function this = goToStart(this)
            flag = 0;
            threshold = 0.15;
            t = 0;
            N = length(this.boids);
            while ~flag
                % Relax the threshold over time
                t = t+1;
                if mod(t,100) == 0
                    threshold = threshold + 0.02;
                end
                
                x = this.arena.get_poses();
                x_temp = x(1:2,:);
                x_goal = this.initPos;

                %% Algorithm
                % Let's make sure we're close enough the the goals
                if norm(x_goal-x_temp,1)< threshold
                     flag = 1;
                end
                dx = this.positionCont(x(1:2, :), x_goal);
                % Normalization of controls.
                dxmax = 0.1;
                for i = 1:N
                    if norm(dx(:,i)) > dxmax
                        dx(:,i) = dx(:,i)/norm(dx(:,i))*dxmax;
                    end
                end

                %% Apply barrier certs. and map to unicycle dynamics
                dx = this.barrierCert(dx, x);
                dx = this.si2uni(dx, x);    
                this.arena.set_velocities(1:N, dx);
                this.arena.step();
            end
        end
        
        function this = initialRotations(this, desired)
            N = length(this.boids);
            exitFlag = false;
            while ~exitFlag
                x = this.arena.get_poses();
                rotations = x(3,:);
                dxu = zeros(2,N);
                exitFlag = true;
                for i = 1:N
                    if abs(rotations(i) - desired(i)) > 0.01 
                        dxu(1:2, i) = [0; 0.2];
                        exitFlag = false;
                    else
                        dxu(1:2, i) = [0; 0];
                    end
                end
                this.arena.set_velocities(1:N, dxu);
                this.arena.step();
            end
        end
        
        function this = resetBoids(this)
             for i = 1:size(this.boids)
                currentBoid = this.boids(i, 1);
                delete(currentBoid.lines);
             end
        end
        
        function this = visualizeNeighbors(this, currentBoid)
            delete(currentBoid.lines);
            for j = 1:length(currentBoid.neighbors)
                nbr_id = currentBoid.neighbors(j);
                nbr = this.boids(nbr_id, 1);
                x1 = this.locations(1, currentBoid.id);
                x2 = this.locations(1, nbr.id);
                y1 = this.locations(2, currentBoid.id);
                y2 = this.locations(2, nbr.id);
                currentBoid.lines(j) = line([x1 x2], [y1 y2],'Color', 'r');
            end
        end
        
        function this = visualizeObstacle(this)
            delete(this.obstacleHandle)
            this.obstacleHandle = viscircles(this.obstaclePos',0.025);
        end
        
        function this = run(this)
            N = length(this.boids);
            dxu = zeros(2, N);
            xi = this.locations;
            
%           this.visualizeObstacle();
            this.obstaclePos = this.obstaclePos + [-0.001; 0];
            
            % For each agent
            for i = 1:size(this.boids)
                currentBoid = this.boids(i, 1);
                currentBoid.getCommNeighbors(this);             
%                 this.visualizeNeighbors(currentBoid);
                dxu(:,currentBoid.id) = this.boids(i, 1).run(this);
            end

            % Go slowly if swarming
            for i = 1:length(this.boids)
                dxu(1:2, i) = dxu(1:2, i)*0.05;
            end
            
            % Utilize barrier certs, and send velocities
            dxu = [dxu [0.001; 3.14]];          % Add obstacale speed to dxu;
            xi = [xi [this.obstaclePos; 0]];    % Add obstacle pos to xi;
            
            
            dxu = this.unibarrierCert(dxu, xi); % Use barrier certs with extended state vector
            dxu = dxu(:, 1:N);                  % Rescale the velocities for the robots
            
            this.arena.set_velocities(1:N, dxu);
            this.arena.step();
        end
    end
end


