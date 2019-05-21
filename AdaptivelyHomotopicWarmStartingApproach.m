
%% 自适应同伦热启动过程中的各个参数赋值
step = 0.2;                 
alpha = 0.5;               
Nexpand = 10;             
epsilon_exit = 1e-5;     
epsilon_0 = 0.05;         
%%
%%计算初始障碍物的几何中心
GenerateGeometricCenters;
%%
%%计算伽马为0.05时障碍物的各个参数的值
GenerateVertexes(epsilon_0);
%% 
%%最简单的子问题的解决，调用ampl工具
!ampl rf.run
!ampl r0.run
!ampl r1.run
%% 
%%解决子问题0，如果子问题0解决失败则退出
!ampl r2.run
load flag.txt;

if (flag == 0)
    error 'Solution Failure at Subproblem 0.';
end

%% 
%%相关的参数
gamma_achieved = epsilon_0;
counter = 0;
store_gamma = [];
store_step = [];

%% 
%%自适应同伦热启动的过程
while (gamma_achieved ~= 1)
    store_step = [store_step, step]; 
    if ((gamma_achieved + step) <= 1) 
        gamma_trial = gamma_achieved + step;
    else 
        gamma_trial = 1;
    end
    GenerateVertexes(gamma_trial);
    store_gamma = [store_gamma, gamma_achieved];
    
    !ampl r2.run
    load flag.txt; 

    if (flag == 1) 
        gamma_achieved = gamma_trial;
        counter = counter + 1; 
    else 
        step = step .* alpha; 
        counter = 0;
    end

    if (counter >= Nexpand)
        step = step ./ alpha;
        counter = 0;
    end
    fclose('all');

    if (step <= epsilon_exit)
        error Intermediate_Subproblem_Solution_Failure_due_to_epsilon_exit_Criterion
    end
end

is_success = 1;