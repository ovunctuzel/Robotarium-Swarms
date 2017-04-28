classdef Data < handle
    properties (GetAccess = public, SetAccess = public)
        flock
        percFound_ar
        avgVisitPerTarget_ar
        CCO_ar
    end
    
    methods
        function this = Data(f, timeout)
            this.flock = f;
            this.CCO_ar = NaN(timeout,1);
            this.avgVisitPerTarget_ar = NaN(timeout,1);
            this.percFound_ar = NaN(timeout,1);
        end

        function calc_all(this, t)
            this.calc_avgVisitPerTarget(t);
            this.calc_percFound(t);
            this.calc_CCO(t);
        end

        function calc_CCO(this, t)
            N = length(this.flock.boids);
            CCO_list = zeros(1,N);
            if t<0
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

        function calc_avgVisitPerTarget(this, t)
           this.avgVisitPerTarget_ar(t) = mean(sum(this.flock.goalVisits));
        end
        
        function calc_percFound(this, t)
           this.percFound_ar(t) = mean(this.flock.goalMarks); 
        end

        function resetData(this, timeout)
            this.CCO_ar = NaN(timeout,1);
            this.avgVisitPerTarget_ar = NaN(timeout,1);
            this.percFound_ar = NaN(timeout,1);
        end
        
        function generateDataFile(this, name)
            CCO = this.CCO_ar;
            avpt = this.avgVisitPerTarget_ar;
            perFo = this.percFound_ar;
            N = length(this.flock.boids);
            T = table(avpt, CCO, perFo);
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
            connected = input(end,1);
            averageVisits = input(end,2);
            percFound = input(end,3);
            
            
            results = [mod nAg nOb rRep rOr rAt groups informed ...
                connected averageVisits percFound];
        end
    end
end