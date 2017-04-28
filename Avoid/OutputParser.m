%Run inside Data folder!

allFolders = dir();
allFolders = allFolders(3:length(allFolders));
folderNames = cell(length(allFolders),1);
for i=1:length(allFolders)
    s = allFolders(i);
    folderNames{i} = s.name;
end

results_ar = [];
for i = 1:12%length(folderNames)
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
    
T = table(results_ar(:,1),results_ar(:,2),results_ar(:,3),results_ar(:,4),results_ar(:,5),results_ar(:,6),results_ar(:,7),results_ar(:,8));
T.Properties.VariableNames = {'mod' 'nAg' 'rRep' 'rOr' 'rAt' 'CCO' 'isolated' 'avgA2A'};
writetable(T,'ExperimentalResults.csv','Delimiter',',');




function results = returnExpResults(filename, foldername)
    fid=fopen(filename,'rt');
    input = textscan(fid, '%f %f %f','HeaderLines',1,'Delimiter',',');
    I_ar = [0.5; 0.25];
    R_ar = [0.01 0.12 0.15; 0.01 0.2 0.2];

    str = strsplit(foldername,'_');
    mod = str{3};
    nAg = str2num(str{4}(2:length(str{4})));
    rRep = R_ar(str2num(str{2}(2)),1);
    rOr = R_ar(str2num(str{2}(2)),2);
    rAt = R_ar(str2num(str{2}(2)),3);

    % ----------------
    cutoff = 902;
    input = cell2mat(input);
    for i = 1:length(input(:,1))
        if input(i,:) == [0 0 0]
            cutoff = i;
            break
        end
    end
    input = input(1:cutoff-1,:);
    
    CCO = input(end,1)
    dispersion = input(end,2) - input(1,2);
    isolated = input(end,3);
   
    results = {mod nAg rRep rOr rAt CCO isolated dispersion};
    fclose(fid);
end