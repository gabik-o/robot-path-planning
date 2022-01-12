load officemap.mat;

%%%User Input
%How far do you want to stay away from wall and obstacles [m]

safety=0.18 %[m]

%%% Calculation

kernel_size=safety*map.Resolution*2;
mat = occupancyMatrix(map);
mat_enlarged = zeros (718+kernel_size, 1330+kernel_size);
mat_enlarged(kernel_size:781+kernel_size-1,kernel_size:1330+kernel_size-
1)=mat;
mat_new_enlarged=zeros(size(mat_enlarged));
kernel=ones(kernel_size);
mat_size=size(mat_enlarged);
for i=1:mat_size(1)
disp(i)
for j=1:mat_size(2)
pixel=mat_enlarged(i,j);
if pixel==1
mat_new_enlarged(i-kernel_size/2:i+kernel_size/2-1,jkernel_
size/2:j+kernel_size/2-1)=kernel;
end
end
end
mat_new=mat_new_enlarged(kernel_size:781+kernel_size-
1,kernel_size:1330+kernel_size-1);
map_bigger_obstacles = binaryOccupancyMap(mat_new,100);
map_bigger_obstacles.LocalOriginInWorld=map.LocalOriginInWorld;

%Plot
%Map with enlarged obstacles

figure(1)
show(map_bigger_obstacles)
  
%Map with original obstacles
figure(2)
show(map)
