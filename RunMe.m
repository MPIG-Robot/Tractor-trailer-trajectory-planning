clear
clc
close
%%
%%声明障碍物顶点的位置
global polygon_obstacle_vertex
%%
%%给出零时刻拖挂卡车的参数（拖车的位置、车辆各个部分的方位角、速度加速度）
x0_1 = -10;         
y0_1 = -17;         
theta0_1 = 0;      
theta0_2 = 0;      
theta0_3 = 0;      
theta0_4 = 0;      
v_0 = 0;             
a_0 = 0; 
%%
%%给出最后时刻车辆停止的位置、速度、加速度以及角加速度
x_center_tf = 12;       
y_center_tf = 0;         
v_tf = 0;                   
a_tf = 0;                    
w_tf = 0;                   
%%
%%论文图5-1，5-2
%%给障碍物的顶点赋值，这里假设每个障碍物有四个顶点。（如下有三个障碍物的话，数组应有24个元素）
%polygon_obstacle_vertex = [0,2,20,2,20,20,0,20,-5,-1.8,20,-1.8,20,-30,-5,-30]; 
%%车辆始末状态的赋值，boundary_constraints = [x0_1, y0_1, theta0_1, theta0_2, theta0_3, theta0_4, phy_0, v_0, a_0, w_0, x_center_tf, y_center_tf, v_tf, a_tf, w_tf];
%boundary_constraints = [-10,-10,pi/2,pi/2,pi/2,pi/2,0,0,0,0,8,0,0,0,0]; 

%%论文图5-3
%polygon_obstacle_vertex = [0,2,20,2,20,20,0,20,-5,-1.8,20,-1.8,20,-30,-5,-30];
%boundary_constraints = [-7,-10,pi/2,pi/2,pi/2,pi/2,0,0,0,0,8,0,0,0,0];

%%论文图5-4
%polygon_obstacle_vertex = [-20 -15 -10 -15 -5 0 -20 5 0 -20 20 -20 20 -2 -2 -2];
%boundary_constraints = [-10,-17,0,0,0,0,0,0,0,0,12,0,0,0,0];

%%论文图5-5
polygon_obstacle_vertex = [-12.81,4,-11.56,1.01,-15.19,-2.2,-25,4,     -9,5.12,-6,15,5,10,-3.52,1.21,    0.47,-2.18,0.38,-8,-4.47,-8,-4.38,-1.94,    -13,-3,-8,-0.5,-5,-5-8,-13,-4-8];
boundary_constraints = [-10,10,0,0,0,0,0,0,0,0,0,0,0,0,0];

%%论文图5-6
%polygon_obstacle_vertex = [-12.81,4,-11.56,1.01,-15.19,-2.2,-25,4,     -9,5.12,-6,15,5,10,-3.52,1.21,    0.47,-2.18,0.38,-8,-4.47,-8,-4.38,-1.94,    -13,-3,-8,-0.5,-5,-5-8,-13,-4-8];
%boundary_constraints = [5,0,pi,pi,pi,pi,0,0,0,0,-25,15,0,0,0];
%%
%%判断以上输入的障碍物顶点位置和车辆始末状态值元素个数是否正确
if (mod(length(polygon_obstacle_vertex), 8) ~= 0)
    error obstacle vertex size wrong
end
if (length(boundary_constraints) ~= 15)
    error two-point boundary conditions wrong
end
%%
%%调用函数，分别储存开始和结束的车辆配置参数到两个文件
SetTwoPointBoundaryConditionsToFiles (boundary_constraints);
%%
%%自适应同伦热启动
AdaptivelyHomotopicWarmStartingApproach;
%%
if (is_success)
    figure (101)
    plot(store_gamma);
    xlabel('Number of Cycle');
    ylabel('\gamma_a_c_h_i_e_v_e_d')
    title('Evolution of \gamma_a_c_h_i_e_v_e_d');
    
    figure (102)
    plot(store_step);
    xlabel('Number of Cycle');
    ylabel('step')
    title('Evolution of step');
    
    figure (103)
    DrawTrajectories; 
   
end
