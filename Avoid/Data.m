classdef Data < handle
    properties (GetAccess = public, SetAccess = public)
        flock
        avgA2A_ar
        isolatedPerc_ar
        CCO_ar
    end
    
    methods
        function this = Data(f, timeout)
            this.flock = f;
            this.isolatedPerc_ar = NaN(timeout,1);
            this.avgA2A_ar = NaN(timeout,1);
            this.CCO_ar = NaN(timeout,1);
        end

        function calc_all(this, t)
            this.calc_CCO(t);
            this.calc_isolated(t);
            this.calc_avgAgent2Agent(t);
        end

        function calc_CCO(this, t)
            N = length(this.flock.boids);
            CCO_list = zeros(1,N);
            if t<900
                this.CCO_ar(t,1) = 0;
                return
            else
                for boidId = 1:N
                    i = this.flock.boids(boidId, 1);
                    % Calc CCO for Boid i
                    closed = [];
                    open = [i.id];
                    while(~isempty(open))
                        cur = open(end);
                        curBoid = this.flock.boids(cur, 1);
                        open = open(1:length(open)-1);
                        closed = [closed, cur];
                        for n=curBoid.neighbors
                            if (~any(n==open) && ~any(n==closed))
                                open = [open, n];
                            end
                        end
                    end
                    CCO_list(1,boidId) = length(closed);
                end
            this.CCO_ar(t,1) = max(CCO_list);
            end
        end
        
        function calc_isolated(this, t)
            I = 0;
            N = length(this.flock.boids);
            for i = 1:N
            currentBoid = this.flock.boids(i, 1);
                if isempty(currentBoid.neighbors)
                    I = I + 1;
                end
            end
            this.isolatedPerc_ar(t,1) = I / N;
        end
        
        function calc_avgAgent2Agent(this, t)
            N = length(this.flock.boids);
            a2a = zeros(1,N*(N-1)/2);
            k = 1;
            for i = 1:N
                for j = 1:N
                    if i>j
                        k = k+1;
                        boid1pos = this.flock.locations(1:2,i);
                        boid2pos = this.flock.locations(1:2,j);
                        dist = norm(boid1pos - boid2pos);
                        a2a(1,k) = dist;
                    end
                end
            end
            this.avgA2A_ar(t,1) = mean(a2a);
        end
              
        
        function resetData(this, timeout)
            this.CCO_ar = zeros(timeout,1);
            this.avgA2A_ar = zeros(timeout,1);
            this.isolatedPerc_ar = zeros(timeout,1);
        end
        
        function generateDataFile(this, name)
            CCO = this.CCO_ar;
            avgA2A = this.avgA2A_ar;
            isolated = this.isolatedPerc_ar;
            N = length(this.flock.boids);
            T = table(CCO, avgA2A, isolated);
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