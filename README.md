# 拖挂卡车的路径规划

## 方法：
自适应同伦热启动方法寻找合适的离散程度，用first-order explicit Runge-Kutta method解决离散问题，用Interior Point Method (IPM)解决非线性规划问题

## 工具：
matlab ， ampl（ipopt）

## 主要参考：
https://github.com/libai1943/Tractor-trailer-trajectory-planning-case-studies-ICRA-19 

## 改进
增加了更多障碍物的情况，case5,6更加复杂；  
对比了相同情况下，车辆的起始位置不同对路径的影响；  
相同情况下，起始位置互换对路径的影响
