% Robotarium Dimensions -0.6, 0.6; -0.3, 0.3
% GoTo Task Input Generator

clusters = {[1 1 1 1 1 1 1 1], ...
            [1 1 1 1 1 1 1 1 1 1 1 1], ...
            [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1], ...
            };
        
% for i=1:length(clusters)
%     mkdir(['R1_T2_C' num2str(i)])
%     mkdir(['R1_T3_C' num2str(i)])
%     mkdir(['R1_M_C' num2str(i)])
%     mkdir(['R1_V_C' num2str(i)])
% end

%R1 = [0.01, 0.12, 0.15]

for i=1:length(clusters)
    cd(['R1_M_C', num2str(i)])
    generateInputFile(10, clusters{i}, 0.01, 0.12, 0.15, 'M');
    cd ..
end

for i=1:length(clusters)
    cd(['R1_T2_C', num2str(i)])
    generateInputFile(10, clusters{i}, 0.01, 0.12, 0.15, 'T2');
    cd ..
end

for i=1:length(clusters)
    cd(['R1_T3_C', num2str(i)])
    generateInputFile(10, clusters{i}, 0.01, 0.12, 0.15, 'T3');
    cd ..
end

for i=1:length(clusters)
    cd(['R1_V_C', num2str(i)])
    generateInputFile(10, clusters{i}, 0.01, 0.12, 0.15, 'V');
    cd ..
end


function generateInputFile(numberExp, clusters, Rrep, Rori, Ratt, Mod)
    for ex = 1:numberExp
        N = sum(clusters);
        RobotX  = zeros(N,1);
        RobotY  = zeros(N,1);
        randOk = true;
        while randOk
            randomPts = [];
            GoalX = randomrange(-0.4, 0.4);
            GoalY = randomrange(-0.15, 0.15);
            randomPts = [randomPts; [GoalX GoalY]]; %#ok<*AGROW>
            for c=1:length(clusters)
               rx = randomrange(-0.5, 0.5);
               ry = randomrange(-0.2, 0.2);
               randomPts = [randomPts; [rx ry]];
            end
            randOk = ~checkValid(randomPts);
        end
        
        for c=1:length(clusters)
            for i=1:clusters(c)
                s = sum(clusters(1:c));
                s = s - clusters(c);
                RobotX (i+s) = randomPts(c+1,1);
                RobotY (i+s) = randomPts(c+1,2);
            end
        end
            
        t_GoalX = [{GoalX}; repmat({''}, N-1,1)];
        t_GoalY = [{GoalY}; repmat({''}, N-1,1)];
        t_Rrep = [{Rrep}; repmat({''}, N-1,1)];
        t_Rori = [{Rori}; repmat({''}, N-1,1)];
        t_Ratt = [{Ratt}; repmat({''}, N-1,1)];
        nAg = [{N}; repmat({''}, N-1,1)];
        groups = [{length(clusters)}; repmat({''}, N-1,1)];

        T = table(RobotX, RobotY, t_GoalX, t_GoalY, t_Rrep, t_Rori, t_Ratt, string([{Mod}; repmat({''}, N-1,1)]), nAg, groups);
        writetable(T,['GoToInput', num2str(ex), '.csv'],'Delimiter',',');
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

