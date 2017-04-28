% Robotarium Dimensions -0.6, 0.6; -0.3, 0.3
% Search Task Input Generator

clusters = {[4 4], ...
            [3 3 3 3], ...
            [2 2], ...
            [4 4 2] ...
            };
        
goals = 4;

        
% for i=1:length(clusters)
%     mkdir(['R2_T2_C' num2str(i)])
%     mkdir(['R2_T3_C' num2str(i)])
%     mkdir(['R2_M_C' num2str(i)])
%     mkdir(['R2_V_C' num2str(i)])
% end

%R1 = [0.01, 0.12, 0.15]

for i=1:length(clusters)
    cd(['R2_M_C', num2str(i)])
    generateInputFile(10, clusters{i}, goals, 0.01, 0.15, 0.2, 'M');
    cd ..
end

for i=1:length(clusters)
    cd(['R2_T2_C', num2str(i)])
    generateInputFile(10, clusters{i}, goals, 0.01, 0.15, 0.2, 'T2');
    cd ..
end

for i=1:length(clusters)
    cd(['R2_T3_C', num2str(i)])
    generateInputFile(10, clusters{i}, goals, 0.01, 0.15, 0.2, 'T3');
    cd ..
end

for i=1:length(clusters)
    cd(['R2_V_C', num2str(i)])
    generateInputFile(10, clusters{i}, goals, 0.01, 0.15, 0.2, 'V');
    cd ..
end


function generateInputFile(numberExp, clusters, goals, Rrep, Rori, Ratt, Mod)
    for ex = 1:numberExp
        N = sum(clusters);
        RobotX  = zeros(N,1);
        RobotY  = zeros(N,1);
        GoalX   = zeros(goals,1);
        GoalY   = zeros(goals,1);
        randOk = true;
        while randOk
            randomPts = [];
            goalPts = [];
            startPts = [];
            for c=1:goals
                GoalX = randomrange(-0.5, 0.5);
                GoalY = randomrange(-0.2, 0.2);
                randomPts = [randomPts; [GoalX GoalY]]; %#ok<*AGROW>
                goalPts = [goalPts; [GoalX GoalY]];
            end
            for c=1:length(clusters)
               rx = randomrange(-0.5, 0.5);
               ry = randomrange(-0.2, 0.2);
               randomPts = [randomPts; [rx ry]];
               startPts = [startPts; [rx ry]];
            end
            randOk = ~checkValid(randomPts);
        end
        
        for c=1:length(clusters)
            for i=1:clusters(c)
                s = sum(clusters(1:c));
                s = s - clusters(c);
                RobotX (i+s) = startPts(c,1);
                RobotY (i+s) = startPts(c,2);
            end
        end
        GoalX = goalPts(:,1);
        GoalY = goalPts(:,2);
        t_GoalX = [num2cell(GoalX); repmat({''}, N-goals,1)];
        t_GoalY = [num2cell(GoalY); repmat({''}, N-goals,1)];
        t_Rrep = [{Rrep}; repmat({''}, N-1,1)];
        t_Rori = [{Rori}; repmat({''}, N-1,1)];
        t_Ratt = [{Ratt}; repmat({''}, N-1,1)];
        nAg = [{N}; repmat({''}, N-1,1)];
        groups = [{length(clusters)}; repmat({''}, N-1,1)];

        T = table(RobotX, RobotY, t_GoalX, t_GoalY, t_Rrep, t_Rori, t_Ratt, string([{Mod}; repmat({''}, N-1,1)]), nAg, groups);
        writetable(T,['SearchInput', num2str(ex), '.csv'],'Delimiter',',');
    end
end

function r = checkValid(randomPts)
    for i=1:length(randomPts(:,1))
        for j=1:length(randomPts(:,1))
            if i>j
                rpt1 = randomPts(i,:);
                rpt2 = randomPts(j,:);
                if norm (rpt1 - rpt2) < 0.1
                    r = false;
                    return
                end
            end
        end
    end
    r = true;
end

function r = randomrange(a, b)
    r = (b-a).*rand + a;
end

