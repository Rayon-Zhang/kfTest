%%
% from：https://blog.csdn.net/weixin_40599145/article/details/113000157
%%
close all;
clear;
clc;
%% 参数初始化
%参数
UAV.v_x = 15;                     %无人机的初始横向速度
UAV.v_y = 20;                     %无人机的初始纵向速度
UAV.x = -20;                      %无人机的初始横向坐标
UAV.y = 20;                       %无人机的初始纵向坐标

UAV.State = [UAV.x; UAV.y; UAV.v_x; UAV.v_y];      %无人机的状态变量

%实际上无人机运行过程中会有一些偏差，所以这里模拟了一些过程噪音
UAV.x_noise = 0.05;                %无人机运动的横向坐标偏差的方差
UAV.y_noise = 0.05;                %无人机运动的纵向坐标偏差的方差
UAV.v_x_noise = 0.1;              %无人机运动的横向速度偏差的方差
UAV.v_y_noise = 0.1;              %无人机运动的纵向速度偏差的方差

%GPS传感器在测量的过程中也有噪音，所以这里也模拟了一些测量噪音
GPS_sensor.X_noise = 2;           %GPS的横向噪声方差
GPS_sensor.X = UAV.State(1,1);    %用于记录GPS的横向坐标测量值
GPS_sensor.Y_noise = 2;           %GPS的纵向噪声方差
GPS_sensor.Y = UAV.State(2,1);    %用于记录GPS的纵向坐标测量值   

%假设每次测量的时间是0.01s
delta_t = 0.01;                      %单位  秒

%% 线性动态方程模型参数和观测方程
A = [1,0,delta_t,0; 
     0,1,0,delta_t; 
     0,0,1,      0;
     0,0,0,      1]; % 系统状态矩阵
C = [1, 0, 0, 0; 
     0, 1, 0, 0];   % 系统位置的观测矩阵
Q = diag([UAV.x_noise, UAV.y_noise, UAV.v_x_noise, UAV.v_y_noise]); % 过程噪音的协方差矩阵
R = diag([GPS_sensor.X_noise, GPS_sensor.Y_noise]); % 测量噪音的协方差矩阵
state_dim = size(Q, 1);
observe_dim = size(R, 1);

%% 存储数据初始化，包括坐标和测量值
storage_Num   = 1000;     %采样次数
storage_State = [];       %存储真实运动的状态变量值，后期用于对比
storage_X     = [];       %存储GPS横向坐标测量值
storage_Y     = [];       %存储GPS纵向坐标测量值

%% 实际航迹计算并显示
for k = 1:storage_Num    %k表示第k次测量
    storage_State = [storage_State, UAV.State]; % 记录实际航迹
    state_noise = [randn*sqrt(UAV.x_noise);
                   randn*sqrt(UAV.y_noise);
                   randn*sqrt(UAV.v_x_noise);
                   randn*sqrt(UAV.v_y_noise)]; % 模拟实际航迹中的过程噪音
    UAV.State = A * UAV.State + state_noise; %在理想航迹上叠加过程噪音，制作出真实航迹
end

figure;subplot(121);
plot(storage_State(1,:),storage_State(2,:), '-');
xlabel('横坐标(cm)');ylabel('纵坐标(cm)');
title('无人机运动航迹');
axis([-40, 40, 0, 100]);

%% 数值重新初始化，用于记录测量数据
storage_X = [];
storage_Y = [];
UAV.State = [UAV.x; 
             UAV.y; 
             UAV.v_x; 
             UAV.v_y]; 
GPS_sensor.X = UAV.State(1,1);
GPS_sensor.Y = UAV.State(2,1);     

%% 传感器观测存在白噪声偏差
for k = 1:storage_Num    %k表示第k次测量
    storage_X = [storage_X,GPS_sensor.X]; %记录GPS的测量数据
    storage_Y = [storage_Y,GPS_sensor.Y];
    
    UAV.State = storage_State(:, k); %把真实航迹取出来
    GPS_sensor.X = UAV.State(1,1) + randn * sqrt(GPS_sensor.X_noise); %在真实航迹上面叠加测量噪音来制作测量数据
    GPS_sensor.Y = UAV.State(2,1) + randn * sqrt(GPS_sensor.Y_noise);
end

subplot(122);
hold on;plot(storage_X, storage_Y, '-');
xlabel('横坐标(cm)');ylabel('纵坐标(cm)');
title('未经滤波算法计算轨迹的结果');
axis([-40, 40, 0, 100]);

%% 卡尔曼滤波估计最优状态
KF_State = [UAV.x; UAV.y; UAV.v_x; UAV.v_y]; 
KF_err = zeros(1, storage_Num);
DIR_err = zeros(1, storage_Num);
P = 0.01 * eye(4); %eye(4)代表4阶单位矩阵，这里是对估计均方误差矩阵Pk赋初值
for k = 1:storage_Num
    Sensor_Meas = [storage_X(1,k);storage_Y(1,k)]; % 测量向量Zk
    
    [KF_State(:,k+1), P] = Track_KF(P, A, C, Q, R, KF_State(:,k), Sensor_Meas); 
    %调用卡尔曼滤波函数更新估计状态和估计均方误差矩阵
    
    KF_err(k) = norm(storage_State(1:2, k) - KF_State(1:2, k)); 
    DIR_err(k) = sqrt((storage_State(1, k) - storage_X(1,k))^2 + (storage_State(2, k) - storage_Y(1,k))^2);
end

%% 画出轨迹曲线
k = 1:storage_Num;

figure;hold on;
subplot(121);
plot(KF_State(1,:), KF_State(2,:), 'r-');
legend('KF滤波的轨迹');
axis([-50, 50, 0, 100]);
xlabel('横坐标(cm)');ylabel('纵坐标(cm)');
title('滤波轨迹');

subplot(122);hold on;
plot(storage_X , storage_Y,'g-');
plot(KF_State(1,:), KF_State(2,:), 'r-');
plot(storage_State(1,:),storage_State(2,:),'b-');
legend('未滤波的轨迹', 'KF滤波的轨迹', '实际轨迹');
axis([-50, 50, 0, 100]);
xlabel('横坐标(cm)');ylabel('横坐标(cm)');
title('轨迹对比');

