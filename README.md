### 总结使用Matlab进行数据处理的一些小技巧

#### 一  绘制2D、3D曲线图demo

- 绘制2d图像

  

- 绘制3d图像

  

#### 二 UWB数据与VSLAM数据进行对齐，且计算距离误差demo

%% read uwb data


path_uwb = 'data/1/ukf_data.txt'  % test2, 已经对齐


fuwb=load(path_uwb);
uwb_time = (fuwb(:,1)-fuwb(1,1));
ax_=fuwb(:,4)/100.0;
ay_=fuwb(:,5)/100.0;

%ax_ = fuwb(:,2)/100.0;
%ay_ = fuwb(:,3)/100.0;

%test1, 未对齐， 对齐到vslam坐标系
 ux_filt=ay_-6.24; %% 
 uy_filt=3.53-ax_;

% test2, 已经对齐
%如果已经对齐，就令ux_filt=ax_; uy_filt=ay_;
%ux_filt=ax_; uy_filt=ay_;



%% read truth from vslam
path = 'data/1/wheel_encoder.txt'
wheelF=load(path);
slam_time = wheelF(:,1)./1000-185.61;%时间戳已经基本对齐

s_x = wheelF(:,12);%vslam output x
s_y = wheelF(:,13);%vslam output y
sl_z = wheelF(:,14);%vslam output theta

sl_x=s_x+0.12*cos(sl_z) + 0.04 * sin(sl_z);
sl_y=s_y+0.12*sin(sl_z) - 0.04 * cos(sl_z);

figure(1)
plot(ux_filt,uy_filt,'*b')

hold on

plot(sl_x,sl_y,'*r')

xlabel('X/m')
ylabel('Y/m')
title('vslam和UWB_filtered轨迹对比');
legend('show')


figure(2)
plot(slam_time,sl_x,'*r');hold on
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

error_dx
error_dy


for i=50:1:size(error,2)         
    error_sum(1)=error_sum(1)+abs(error(1,i)-error_dx);
    error_sum(2)=error_sum(2)+abs(error(2,i)-error_dy);
end

error_mean_x=error_sum(1)/(size(sl_x,1)-50) %平均 x方向 偏差
error_mean_y=error_sum(2)/(size(sl_y,1)-50) %平均 y方向 偏差



last_j=100;
dist_=0;
% plot(ux_filt-error_dx,uy_filt-error_dy)
for i=1:1:5000
    for j=last_j:1:1900
        if  j <= length(slam_time) && slam_time(j) > (uwb_time(i)-0)
            dist_(j)=sqrt((sl_x(j)-ux_filt(i)+error_dx)^2+(sl_y(j)-uy_filt(i)+error_dy)^2);
            break;
        end
    end
end
error_distance = mean(dist_) % 平均距离偏差



持续完善中....