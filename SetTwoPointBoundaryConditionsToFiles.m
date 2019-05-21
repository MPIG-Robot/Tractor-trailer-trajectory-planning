
%%函数SetTwoPointBoundaryConditionsToFiles（）的定义
function SetTwoPointBoundaryConditionsToFiles(vector)

warning off

 X1 = vector(1 : 10);
 X2 = vector(11 : 15);
 %%
 %%储存车辆的开始配置到'Initial_config'
delete('Initial_config');
fid = fopen('Initial_config', 'w');
for ii = 1 : length(X1)
    fprintf(fid,'%g  %f \r\n', ii, X1(ii));
end
fclose(fid);
%%储存车辆的结束配置到'Terminal_config'
delete('Terminal_config');
fid = fopen('Terminal_config', 'w');
for ii = 1 : length(X2)
    fprintf(fid,'%g  %f \r\n', ii, X2(ii));
end
fclose(fid);