#代码简介：
拖挂卡车的路径规划

#方法：
自适应同伦热启动方法寻找合适的离散程度，用first-order explicit Runge-Kutta method解决离散问题，用Interior Point Method (IPM)解决非线性规划问题

#工具：
matlab ， ampl（ipopt）

#参考文献：
Trajectory Planning for a Tractor with Multiple Trailers in Extremely Narrow Environments: A Unified Approach *
IEEE 2019 International Conference on Robotics and Automation (ICRA)

#参考代码：
https://github.com/libai1943/Tractor-trailer-trajectory-planning-case-studies-ICRA-19

#在原代码中改进：
增加障碍物的情况：
1.其中case1和case2对比了相同障碍物的情况下，车辆初始位置的变化对路径的影响。
2.Case1和case3对比了障碍物轻微的左移对路径的影响
3.Case4中改变初始setp对最后结果中的自适应step的变化值。在AdaptivelyHomotopicWarmStartingApproach.m中注释。分别问step=0.1,0.05,0.01
3.Case5和case6对比初始位置互换对路径的影响
4.case7中反应了初始位置离障碍物过远，而产生非理想路径
