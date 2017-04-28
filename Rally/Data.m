classdef Data < handle
    properties (GetAccess = public, SetAccess = public)
        flock
        informedNeighborhood_ar
        avgDistGoal_ar
        percReached_ar
    end
    
    methods
        function this = Data(f, timeout)
            this.flock = f;
            this.informedNeighborhood_ar = NaN(timeout,1);
            this.avgDistGoal_ar = NaN(timeout,1);
            this.percReached_ar = NaN(timeout,1);
        end

        function calc_all(this, t)
            this.calc_informedNetworkPerc(t);
            this.calc_avgDistGoal(t);
        end

%         function calc_informedNeighborhood(this, t)
%             IN = 0;
%             N = length(this.flock.boids);
%             informed = false;
%             for i = 1:N
%             currentBoid = this.flock.boids(i, 1);
%                 if currentBoid.isLeader
%                     informed = true;
%                 end
%                 for j = currentBoid.neighbors
%                     if this.flock.boids(j, 1).isLeader
%                         informed = true;
%                         break
%                     end
%                 end
%                 if informed
%                     IN = IN + 1;
%                 end
%             end
%             this.informedNeighborhood_ar(t,1) = IN / N;
%         end

        function calc_informedNetworkPerc(this, t)
            IN = 0;
            N = length(this.flock.boids);
            for i = 1:N
            currentBoid = this.flock.boids(i, 1);
                if currentBoid.infNetworkMember
                    IN = IN + 1;
                end
            end
            this.informedNeighborhood_ar(t,1) = IN / N;
        end
        
        function calc_avgDistGoal(this,t)
            N = length(this.flock.boids);
            dist_ar = zeros(N,1);
            count = N;
            for i = 1:N
                goaldist = norm(this.flock.locations(1:2, i) - this.flock.goal);
                if ~this.flock.boids(i,1).atGoal
                    dist_ar(i) = goaldist;
                    count = count-1;
                end
            end
            this.avgDistGoal_ar(t,1) = mean(dist_ar);
            this.percReached_ar(t,1) = count/N;
        end
        
        
        function resetData(this, timeout)
            this.informedNeighborhood_ar = zeros(timeout,1);
            this.avgDistGoal_ar = zeros(timeout,1);
            this.percReached_ar = zeros(timeout,1);
        end
        
        function generateDataFile(this, name)
            infNH = this.informedNeighborhood_ar;
            avgGD = this.avgDistGoal_ar;
            perRe = this.percReached_ar;
            N = length(this.flock.boids);
            T = table(infNH, avgGD, perRe);
            writetable(T,[name '_' num2str(N) '_' this.flock.model ...
                '_' num2str(this.flock.radiusRep) ...
                '_' num2str(this.flock.radiusOri) ...
                '_' num2str(this.flock.radiusAtr) ...
                '.csv'],'Delimiter',',');
        end
        
        function results = returnExpResults(filename)
            fid=fopen(filename,'rt');
            input = textscan(fid, '%f %f %f','HeaderLines',1,'Delimiter',',');
            % NEEDS ATTENTION!
            mod = filename(10);
            nAg = filename(8);
            nOb = 0;
            rRep = 0.01;
            rOr = 0.2;
            rAt = 0.2;
            groups = 2;
            informed = 0.2;
            % ----------------
            foundGoalLast = input(end,1);
            avgGoalDistLast = input(end,2);
            informedNeighborhoods = mean(input(:,3));
            
            
            results = [mod nAg nOb rRep rOr rAt groups informed ...
                foundGoalLast avgGoalDistLast informedNeighborhoods];
        end
    end
end