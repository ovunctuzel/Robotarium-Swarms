% Robotarium Dimensions -0.6, 0.6; -0.3, 0.3
% Avoid Task Input Generator

nAg_ar = [8, 10, 12, 15];

% for i=1:length(nAg_ar)
%     mkdir(['R1_T2_N' num2str(nAg_ar(i))])
%     mkdir(['R1_T3_N' num2str(nAg_ar(i))])
%     mkdir(['R1_M_N' num2str(nAg_ar(i))])
%     mkdir(['R1_V_N' num2str(nAg_ar(i))])
% end

%R1 = [0.01, 0.12, 0.15]
%I1 = 0.5

for i=1:length(nAg_ar)
    cd(['R1_M_N', num2str(nAg_ar(i))])
    generateInputFile(100, 0.01, 0.15, 0.2, 'M', nAg_ar(i));
    cd ..
end

for i=1:length(nAg_ar)
    cd(['R1_T2_N', num2str(nAg_ar(i))])
    generateInputFile(100, 0.01, 0.15, 0.2, 'T2', nAg_ar(i));
    cd ..
end

for i=1:length(nAg_ar)
    cd(['R1_T3_N', num2str(nAg_ar(i))])
    generateInputFile(100, 0.01, 0.15, 0.2, 'T3', nAg_ar(i));
    cd ..
end

for i=1:length(nAg_ar)
    cd(['R1_V_N', num2str(nAg_ar(i))])
    generateInputFile(100, 0.01, 0.15, 0.2, 'V', nAg_ar(i));
    cd ..
end

function generateInputFile(numberExp, Rrep, Rori, Ratt, Mod, N)
    for ex = 1:numberExp
        t_Rrep = [{Rrep}; repmat({''}, N-1,1)];
        t_Rori = [{Rori}; repmat({''}, N-1,1)];
        t_Ratt = [{Ratt}; repmat({''}, N-1,1)];
        nAg = [{N}; repmat({''}, N-1,1)];

        T = table(t_Rrep, t_Rori, t_Ratt, string([{Mod}; repmat({''}, N-1,1)]), nAg);
        writetable(T,['AvoidInput', num2str(ex), '.csv'],'Delimiter',',');
    end
end

