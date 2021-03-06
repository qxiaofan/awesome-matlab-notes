### matlab在项目中的常见用法总结

####　一 UWB数据与VSLAM数据进行对齐，且计算距离误差demo

clc;
close all;
clear all;

R = [0.9994516721827295 0.03311125142827273         0;
-0.03311125142827273 0.9994516721827295        -0;
        0        -0         1]
    
T = [-8.175637488594226;
-5.121279414939621;
-0]

%% read uwb data
path_uwb = '1/data_UWB_least_square.txt';


fuwb=load(path_uwb);
%uwb_time = (fuwb(:,1)-fuwb(1,1))./1000.;

uwb_time = fuwb(:,1)./1000.; 

ax_=fuwb(:,2);
ay_=fuwb(:,3);
error_ceres_dist = fuwb(:,5);

figure(2)
plot(error_ceres_dist,'*b');

title('最小二乘final_cost分布');
xlabel('time');
ylabel('final_cost/m');
saveas(gcf,'final_cost','png');
magnify


%ax_ = fuwb(:,2)/100.0;
%ay_ = fuwb(:,3)/100.0;

%test1, 未对齐， 对齐到vslam坐标系
%ux_filt=ay_-6.24-1.1973-1; %% 
%uy_filt=3.53-ax_-0.9163-1.3;


ux_filt = zeros(size(ax_,1),1);
uy_filt = zeros(size(ay_,1),1);


 for i = 1:size(ax_,1)
     uwb_least_square = [ax_(i);ay_(i);0];
     uwb_in_slam = R*uwb_least_square  + T;
     ux_filt_temp = uwb_in_slam(1);
     uy_filt_temp = uwb_in_slam(2);
     ux_filt(i,1) = ux_filt_temp;
     uy_filt(i,1) = uy_filt_temp;
 end;


% test2, 已经对齐
%如果已经对齐，就令ux_filt=ax_; uy_filt=ay_;
%ux_filt=ax_; uy_filt=ay_;


%% read truth from vslam

path = '1/wheel_encoder.txt';
wheelF=load(path);
slam_time_test = wheelF(:,1)./1000.;

%slam_time = wheelF(:,1)./1000.
slam_time = wheelF(:,1)./1000.+1610956445.239;%时间戳已经基本对齐

% figure(222)
% plot(slam_time,'b-');
% hold on
% plot(uwb_time,'r*');


s_x = wheelF(:,12);%vslam output x
s_y = wheelF(:,13);%vslam output y
sl_z = wheelF(:,14);%vslam output theta


%sl_x=s_x+0.12*cos(sl_z) + 0.04 * sin(sl_z);
%sl_y=s_y+0.12*sin(sl_z) - 0.04 * cos(sl_z);

sl_x=s_x+0.12*cos(sl_z) + 0.04 * sin(sl_z);
sl_y=s_y+0.12*sin(sl_z) - 0.04 * sin(sl_z);
%sl_x = s_x;
%sl_y = s_y;

figure(3)
plot(ux_filt,uy_filt,'*b')

hold on

plot(sl_x,sl_y,'*r')

xlabel('X/m')
ylabel('Y/m')
title('vslam和UWB_filtered轨迹粗对齐');
saveas(gcf,'uwb_vslam_rude_aligned','png');
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

data_ceres_error_slam = zeros(2,size(sl_x_new,1));

data_align_slam_xy = zeros(2,size(sl_x_new,1));
data_align_uwb_xy = zeros(2,size(sl_x_new,1));


last_j=100;
dist_MAE = 0;
dist_RMSE = 0;
%figure(3)
%plot(ux_filt-error_dx,uy_filt-error_dy,'*m');

% 时间戳未做插补时求距离

%%对vslam数据的时间戳进行了中间取平均插补
num = 0;
for i=1:1:size(uwb_time,1)
    for j=last_j:1:size(sl_x_new,2)-2
        if  j <= length(slam_time_new) && slam_time_new(j) > (uwb_time(i) - 0)

           num = num + 1;
            dist_MAE(j)=sqrt((sl_x_new(j)-ux_filt(i)+error_dx)^2 + abs(sl_y_new(j)-uy_filt(i)+error_dy)^2);
            dist_RMSE(j) = (sl_x_new(j) - ux_filt(i)+error_dx)^2 + (sl_y_new(j)-uy_filt(i)+error_dy)^2;
           
            %dist_MAE(j)=sqrt((sl_x_new(j)-ux_filt(i))^2 + abs(sl_y_new(j)-uy_filt(i)^2));
            %dist_RMSE(j) = (sl_x_new(j) - ux_filt(i))^2 + (sl_y_new(j)-uy_filt(i))^2;


​           
​            %dist_(j) = (sl_x_new(j) - ux_filt(i))^2 + (sl_y_new(j) - uy_filt(i))^2;
​                
​            data_align_slam_xy(1,i) = sl_x_new(j);
​            data_align_slam_xy(2,i) = sl_y_new(j);
​            
            data_align_uwb_xy(1,i) = ux_filt(i) - error_dx;
            data_align_uwb_xy(2,i) = uy_filt(i) - error_dy;
            
            data_ceres_error_slam(1,i) = dist_MAE(j);
            
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
saveas(gcf,'x_aligned','png');


figure(10)
plot(data_align_slam_xy(2,:),'b-');
hold on 
plot(data_align_uwb_xy(2,:),'k:');
magnify

title('在Y方向上对vslam与uwb数据对齐');
xlabel('time');
ylabel('Y/m');
legend('插补后的VSLAM','UWB数据');
saveas(gcf,'y_aligned','png');

nonzeroCnt = 0;
for i = 1:1:length(dist_MAE)
    if(abs(dist_MAE(i)) > 0.0001)
        nonzeroCnt =nonzeroCnt + 1;
    end
end
nonzeroCnt
size(dist_MAE,2)
size(dist_RMSE,2)
num

error_distance_MAE = sum(dist_MAE)/num % 平均距离偏差

error_distance_RMSE = sqrt(sum(dist_RMSE)/num) %RMSE

持续完善中....