% Robotarium Dimensions -0.6, 0.6; -0.3, 0.3
% Rally Task Input Generator

clusters = {[8], [4 4], [3 3 2], [2 2 2 2], ...
            [12], [6 6], [4 4 4], [3 3 3 3], ...
            [15], [7 8], [5 5 5], [4 4 4 3], [4 4 2]};
        
% for i=1:length(clusters)
%     mkdir(['R1_I1_T2_C' num2str(i)])
%     mkdir(['R1_I1_T3_C' num2str(i)])
%     mkdir(['R1_I1_M_C' num2str(i)])
%     mkdir(['R1_I1_V_C' num2str(i)])
% end
%R1 = [0.01, 0.12, 0.15]
%I1 = 0.5

for i=1:length(clusters)
    cd(['R1_I1_M_C', num2str(i)])
    generateInputFile(100, clusters{i}, 0.5, 0.01, 0.12, 0.15, 'M');
    cd ..
end

for i=1:length(clusters)
    cd(['R1_I1_T2_C', num2str(i)])
    generateInputFile(100, clusters{i}, 0.5, 0.01, 0.12, 0.15, 'T2');
    cd ..
end

for i=1:length(clusters)
    cd(['R1_I1_T3_C', num2str(i)])
    generateInputFile(100, clusters{i}, 0.5, 0.01, 0.12, 0.15, 'T3');
    cd ..
end

for i=1:length(clusters)
    cd(['R1_I1_V_C', num2str(i)])
    generateInputFile(100, clusters{i}, 0.5, 0.01, 0.12, 0.15, 'V');
    cd ..
end


function generateInputFile(numberExp, clusters, Informed, Rrep, Rori, Ratt, Mod)
    for ex = 1:numberExp
        N = sum(clusters);
        RobotX  = zeros(N,1);
        RobotY  = zeros(N,1);
%         randomPts = [];

%         GoalX = randomrange(-0.4, 0.4);
%         GoalY = randomrange(-0.15, 0.15);
%         randomPts = [randomPts; [GoalX GoalY]]; %#ok<*AGROW>
% 
%         for c=1:length(clusters)
%             randOk = true;
%             while randOk
%                rx = randomrange(-0.45, 0.45);
%                ry = randomrange(-0.15, 0.15);
%                randOk = ~checkValid(randomPts, [rx ry]);
%             end
%             randomPts = [randomPts; [rx ry]];
% 
%             for i=1:clusters(c)
%                 s = sum(clusters(1:c));
%                 s = s - clusters(c);
%                 RobotX (i+s) = rx;
%                 RobotY (i+s) = ry;
%             end
%         end

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
        t_Informed = [{Informed}; repmat({''}, N-1,1)];
        t_Rrep = [{Rrep}; repmat({''}, N-1,1)];
        t_Rori = [{Rori}; repmat({''}, N-1,1)];
        t_Ratt = [{Ratt}; repmat({''}, N-1,1)];
        nAg = [{N}; repmat({''}, N-1,1)];
        groups = [{length(clusters)}; repmat({''}, N-1,1)];

        T = table(RobotX, RobotY, t_GoalX, t_GoalY, t_Informed, t_Rrep, t_Rori, t_Ratt, string([{Mod}; repmat({''}, N-1,1)]), nAg, groups);
        writetable(T,['RallyInput', num2str(ex), '.csv'],'Delimiter',',');
    end
end

function r = checkValid_old(randomPts, pt) %#ok<*DEFNU>
    for i=1:length(randomPts(:,1))
       rpt = randomPts(i,:);
       if norm (rpt - pt) < 0.3
           r = false;
           return
       end
    end
    r = true;
    disp(r);
end

function r = checkValid(randomPts)
    for i=1:length(randomPts(:,1))
        for j=1:length(randomPts(:,1))
            if i>j
                rpt1 = randomPts(i,:);
                rpt2 = randomPts(j,:);
                if norm (rpt1 - rpt2) < 0.3
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

