

function GenerateVertexes(gamma)
%%
%%如果gamma不在0和1之间则判定输入有误
if ((gamma > 1) || (gamma < 0))
    error 'Invalid \gamma';
end

warning off

%%
%%通过polygon_obstacle_vertex元素的个数获得障碍物的个数
global polygon_obstacle_vertex
number_of_obstacles = length(polygon_obstacle_vertex) / 8;
%%
%%获得初始障碍物的几何中心
center = [];
for ii = 1 : number_of_obstacles
    temp = polygon_obstacle_vertex(((ii-1)*8+1) : ((ii-1)*8+8));
    center_x = mean([temp(1),temp(3),temp(5),temp(7)]);
    center_y = mean([temp(2),temp(4),temp(6),temp(8)]);
    center = [center, repmat([center_x,center_y],1,4)];
end
%%
%%计算获得新生成的障碍物顶点的位置
Error_vector = polygon_obstacle_vertex - center;
New_vertex = Error_vector .* gamma + center;
%%
%%把当前新障碍物顶点的位置存入文件'Current_vertex'中
delete('Current_vertex');
fid = fopen('Current_vertex', 'w');
ind = 0;
for ii = 1 : number_of_obstacles % From the first obstacle to the last
    for jj = 1 : 4 % From the first vertex to the fourth (the obstacles are all assumed to be quadrilateral)
        for kk = 1 : 2  % From the x coordinate to the y corordinate.
            ind = ind + 1;
            fprintf(fid,'%g %g %g %f \r\n', ii,jj,kk,New_vertex(ind));
        end
    end
end
fclose(fid);
%%
%%保存障碍物的个数至文件'Number_obstacle'
delete('Number_obstacle');
fid = fopen('Number_obstacle', 'w');
fprintf(fid,'%g', number_of_obstacles);
fclose(fid);
%%
%%计算每个障碍物的面积，防碰撞限制中会用到
Area = [];
for ii = 1 : number_of_obstacles
    temp = New_vertex(((ii-1)*8+1) : ((ii-1)*8+8));
    x_axis = [temp(1),temp(3),temp(5),temp(7)];
    y_axis = [temp(2),temp(4),temp(6),temp(8)];
    Area = [Area, CalculateArea(x_axis,y_axis)];
end
%%
%%分别保存每个障碍物的面积至文件'Area'
delete('Area');
fid = fopen('Area', 'w');
for ii = 1 : number_of_obstacles
    fprintf(fid,'%g    %g\r\n', ii, Area(ii));
end
fclose(fid);