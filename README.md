### 总结使用Matlab进行数据处理的一些小技巧

#### 一  绘制2D、3D曲线图demo

- 绘制2d图像

  

- 绘制3d图像

  

#### 二 UWB数据与VSLAM数据进行对齐，且计算距离误差demo

clc;
close all;
clear all;

%% read uwb data
path_uwb = data/1/data_2020_1104_UWB_ceres.txt';


fuwb=load(path_uwb);
uwb_time = (fuwb(:,1)-fuwb(1,1))./1000.;
ax_=fuwb(:,2);
ay_=fuwb(:,3);
error_ceres_dist = fuwb(:,5);

%figure(1)
%plot(ax_,ay_,'*y');

figure(2)
plot(error_ceres_dist,'*b');

title('最小二乘final_cost分布');
xlabel('time');
ylabel('final_cost/m');
magnify


%ax_ = fuwb(:,2)/100.0;
%ay_ = fuwb(:,3)/100.0;

%test1, 未对齐， 对齐到vslam坐标系
 ux_filt=ay_-6.24-1.1973-1; %% 
 uy_filt=3.53-ax_-0.9163-1.3;


% test2, 已经对齐
%如果已经对齐，就令ux_filt=ax_; uy_filt=ay_;
%ux_filt=ax_; uy_filt=ay_;



%% read truth from vslam

path = 'data/1/wheel_encoder.txt';
wheelF=load(path);
slam_time_test = wheelF(:,1)./1000.;

slam_time = wheelF(:,1)./1000.+127.7140;%时间戳已经基本对齐


s_x = wheelF(:,12);%vslam output x
s_y = wheelF(:,13);%vslam output y
sl_z = wheelF(:,14);%vslam output theta


sl_x=s_x+0.12*cos(sl_z) + 0.04 * sin(sl_z);
sl_y=s_y+0.12*sin(sl_z) - 0.04 * cos(sl_z);

%sl_x=s_x+0.1*cos(sl_z) + 0.04 * sin(sl_z);
%sl_y=s_y+0.1*sin(sl_z) - 0.04 * sin(sl_z);
%sl_x = s_x;
%sl_y = s_y;

figure(3)
plot(ux_filt,uy_filt,'*b')

hold on

plot(sl_x,sl_y,'*r')

xlabel('X/m')
ylabel('Y/m')
title('vslam和UWB_filtered轨迹粗对齐');
legend('show')


figure(4)
plot(slam_time,sl_x,'*r');
hold on
plot(uwb_time,ux_filt,'*b')
xlabel('time');
ylabel('X');
title('vslam与UWB_Filtered在x上的轨迹')
legend('show')

%% calculate the error

error= zeros(3,size(sl_x,1)); % x,y,z
error_sum=zeros(3,1);
last_j=1;
for i=50:1:size(sl_x,1)
    for j=last_j:1:size(ux_filt,1)  
        if uwb_time(j) > slam_time(i) 
            error(1,i)=ux_filt(j)-sl_x(i);   
            error(2,i)=uy_filt(j)-sl_y(i);             
            last_j=j;        
            break;
        end
    end
end
error_dx=mean(error(1,:));%对齐误差 x方向
error_dy=mean(error(2,:));%对齐误差 y方向


for i=50:1:size(error,2)         
    error_sum(1)=error_sum(1)+abs(error(1,i)-error_dx);
    error_sum(2)=error_sum(2)+abs(error(2,i)-error_dy);
end

error_mean_x=error_sum(1)/(size(sl_x,1)-50) %平均 x方向 偏差
error_mean_y=error_sum(2)/(size(sl_y,1)-50) %平均 y方向 偏差




%%对VSLAM数据进行插补
sl_x_interpolation = zeros(1,size(sl_x,1));
sl_y_interpolation = zeros(1,size(sl_y,1));
slam_time_interpolation = zeros(1,size(slam_time,1));

for i = 1:size(sl_x,1)-1
    sl_x_interpolation(i) = (sl_x(i) + sl_x(i+1))/2.0;
    sl_y_interpolation(i) = (sl_y(i) + sl_y(i+1))/2.0;
    slam_time_interpolation(i) = (slam_time(i+1) + slam_time(i))/2.0 ;
end

sl_x_new = zeros(1,2*size(s_x,1) - 1);
sl_y_new = zeros(1,2*size(s_y,1) - 1);
slam_time_new = zeros(1,2*size(slam_time,1) - 1);

for k = 1:size(sl_x,1)-1 
   sl_x_new(2*k -1) = sl_x(k);
   sl_x_new(2*k) = sl_x_interpolation(k);
   sl_y_new(2*k -1) = sl_y(k);
   sl_y_new(2*k) = sl_y_interpolation(k);

   slam_time_new(2*k -1) = slam_time(k);
   slam_time_new(2*k) = slam_time_interpolation(k);
end

figure(5)
plot(sl_x,'k*');
hold on
plot(sl_x_interpolation,'m*');
hold on
plot(sl_x_new,'g*');
title('对vslam数据X方向进行插补');
xlabel('time');
ylabel('X/m');
legend('x','X-插补','X-插补后结果');
magnify


figure(6)
plot(sl_y,'k*');
hold on
plot(sl_y_interpolation,'m*');
hold on
plot(sl_y_new,'g*');
magnify

title('对vslam数据Y方向进行插补');
xlabel('time');
ylabel('Y/m');
legend('Y','Y-插补','Y-插补后结果');

figure(7)
plot(slam_time,'k*');
hold on 
plot(slam_time_interpolation,'m*');
hold on
plot(slam_time_new,'g*');
hold off;

data_ceres_error_slam = zeros(2,size(sl_x_new,1));

data_align_slam_xy = zeros(2,size(sl_x_new,1));
data_align_uwb_xy = zeros(2,size(sl_x_new,1));


last_j=200;
dist_=0;
%figure(3)
%plot(ux_filt-error_dx,uy_filt-error_dy,'*m');

% 时间戳未做插补时求距离
% for i=1:1:5000
%     for j=last_j:1:size(sl_x,1)-2
%         if  j <= length(slam_time) && slam_time(j) > (uwb_time(i) - 0)
%             
%             dist_(j)=sqrt((sl_x(j)-ux_filt(i)+error_dx)^2+(sl_y(j)-uy_filt(i)+error_dy)^2);
%                 
%             data_align_slam_xy(1,i) = sl_x(j);
%             data_align_slam_xy(2,i) = sl_y(j);
%             
%             data_align_uwb_xy(1,i) = ux_filt(i) - error_dx;
%             data_align_uwb_xy(2,i) = uy_filt(i) - error_dy;
%             
%             data_ceres_error_slam(1,i) = dist_(j);
%             
%             data_ceres_error_slam(2,i) = error_ceres_dist(i);
%             
%             break;
%         end
%     end
% end

%%对vslam数据的时间戳进行了中间取平均插补
for i=1:1:size(uwb_time,1)
    for j=last_j:1:size(sl_x_new,2)-2
        if  j <= length(slam_time_new) && slam_time_new(j) > (uwb_time(i) - 0)
            
            dist_(j)=sqrt((sl_x_new(j)-ux_filt(i)+error_dx)^2+(sl_y_new(j)-uy_filt(i)+error_dy)^2);
                
            data_align_slam_xy(1,i) = sl_x_new(j);
            data_align_slam_xy(2,i) = sl_y_new(j);
            
            data_align_uwb_xy(1,i) = ux_filt(i) - error_dx;
            data_align_uwb_xy(2,i) = uy_filt(i) - error_dy;
            
            data_ceres_error_slam(1,i) = dist_(j);
            
            data_ceres_error_slam(2,i) = error_ceres_dist(i);
            
            break;
        end
    end
end



figure(8)
plot(data_ceres_error_slam(1,size(data_ceres_error_slam,2)),'*k');

hold on
plot(abs(data_ceres_error_slam(2,1500:size(data_ceres_error_slam,2))),'*m');

title('距离差');
xlabel('time');
ylabel('error_dis/m');
legend('error_dis','error_final_cost')

figure(9)
plot(data_align_slam_xy(1,:),'b-');
hold on 
plot(data_align_uwb_xy(1,:),'k:');
magnify

title('在X方向上对vslam与uwb数据对齐');
xlabel('time');
ylabel('X/m');
legend('插补后的VSLAM','UWB数据');


figure(10)
plot(data_align_slam_xy(2,:),'b-');
hold on 
plot(data_align_uwb_xy(2,:),'k:');
magnify

title('在Y方向上对vslam与uwb数据对齐');
xlabel('time');
ylabel('Y/m');
legend('插补后的VSLAM','UWB数据');

nonzeroCnt = 0;
for i = 1:1:length(dist_)
    if(abs(dist_(i)) > 0.0001)
        nonzeroCnt =nonzeroCnt + 1;
    end
end
error_distance = sum(dist_)/nonzeroCnt % 平均距离偏差

持续完善中....