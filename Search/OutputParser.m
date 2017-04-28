%Run inside Data folder!

allFolders = dir();
allFolders = allFolders(3:length(allFolders));
folderNames = cell(length(allFolders),1);
for i=1:length(allFolders)
    s = allFolders(i);
    folderNames{i} = s.name;
end

results_ar = [];
for i = 1:48%length(folderNames)
    folderName = folderNames{i}
    cd(folderName);
    allFiles = dir();
    allFiles = allFiles(3:length(allFiles));
    fileNames = cell(length(allFiles),1);
    for j=1:length(allFiles)
        s = allFiles(j);
        fileNames{j} = s.name;
    end
    
    for j=1:length(fileNames)
        results = returnExpResults(fileNames{j}, folderName);
        results_ar = [results_ar; results];
    end
    cd ..
end
    
T = table(results_ar(:,1),results_ar(:,2),results_ar(:,3),results_ar(:,4),results_ar(:,5),results_ar(:,6),results_ar(:,7),results_ar(:,8),results_ar(:,9),results_ar(:,10));
T.Properties.VariableNames = {'mod' 'nAg' 'rRep' 'rOr' 'rAt' 'groups' 'informed' 'foundGoalLast' 'avgGoalDistLast' 'informedNeighborhoods'};
writetable(T,'ExperimentalResults.csv','Delimiter',',');




function results = returnExpResults(filename, foldername)
    fid=fopen(filename,'rt');
    input = textscan(fid, '%f %f %f','HeaderLines',1,'Delimiter',',');
    I_ar = [0.5; 0.25];
    R_ar = [0.01 0.12 0.15; 0.01 0.2 0.2];
    C_ar = {[8], [4 4], [3 3 2], [2 2 2 2], ...
            [12], [6 6], [4 4 4], [3 3 3 3], ...
            [15], [7 8], [5 5 5], [4 4 4 3]};

    str = strsplit(foldername,'_');
    mod = str{4};
    cluster = C_ar{str2num(str{5}(2:length(str{5})))};
    groups = length(cluster);
    nAg = sum(cluster);
    rRep = R_ar(str2num(str{2}(2)),1);
    rOr = R_ar(str2num(str{2}(2)),2);
    rAt = R_ar(str2num(str{2}(2)),3);
    rAt = rAt(1:length(rAt)-4);
    informed = I_ar(str2num(str{3}(2)));
    % ----------------
    cutoff = 400;
    input = cell2mat(input);
    for i = 1:length(input(:,1))
        if input(i,:) == [0 0 0]
            cutoff = i;
            break
        end
    end
    input = input(1:cutoff-1,:);
    foundGoalLast = input(end,1);
    avgGoalDistLast = input(end,2);
    informedNetworks = mean(input(:,3));

    results = {mod nAg rRep rOr rAt groups informed ...
        foundGoalLast avgGoalDistLast informedNetworks};
    fclose(fid);
end