clear all

experiments = 100;

N = 8;
for exp=1:4
    experimentSet = ['R1_I1_M_C', num2str(exp)];
    Ovunc_main_Rally

    experimentSet = ['R1_I1_T2_C', num2str(exp)];
    Ovunc_main_Rally

    experimentSet = ['R1_I1_T3_C', num2str(exp)];
    Ovunc_main_Rally
    
    experimentSet = ['R1_I1_V_C', num2str(exp)];
    Ovunc_main_Rally
end

N = 12;
for exp=5:8
    experimentSet = ['R1_I1_M_C', num2str(exp)];
    Ovunc_main_Rally

    experimentSet = ['R1_I1_T2_C', num2str(exp)];
    Ovunc_main_Rally

    experimentSet = ['R1_I1_T3_C', num2str(exp)];
    Ovunc_main_Rally
    
    experimentSet = ['R1_I1_V_C', num2str(exp)];
    Ovunc_main_Rally
end

N = 15;
for exp=9:12
    experimentSet = ['R1_I1_M_C', num2str(exp)];
    Ovunc_main_Rally

    experimentSet = ['R1_I1_T2_C', num2str(exp)];
    Ovunc_main_Rally

    experimentSet = ['R1_I1_T3_C', num2str(exp)];
    Ovunc_main_Rally
    
    experimentSet = ['R1_I1_V_C', num2str(exp)];
    Ovunc_main_Rally
end
