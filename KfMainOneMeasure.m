%%
% from：https://blog.csdn.net/weixin_40599145/article/details/113000157
%%
close all;
clear;
clc;
%% 参数初始化
%参数
car.v0_x = 15;                     %无人机的初始横向速度
car.v0_y = 20;                     %无人机的初始纵向速度
car.x0 = -20;                      %无人机的初始横向坐标
car.y0 = 20;                       %无人机的初始纵向坐标

car.State = [car.x0; 
             car.y0; 
             car.v0_x; 
             car.v0_y];      %无人机的状态变量

%实际上无人机运行过程中会有一些偏差，所以这里模拟了一些过程噪音
car.x_noise = 0.05;                %无人机运动的横向坐标偏差的方差
car.y_noise = 0.05;                %无人机运动的纵向坐标偏差的方差
car.v_x_noise = 0.1;               %无人机运动的横向速度偏差的方差
car.v_y_noise = 0.1;               %无人机运动的纵向速度偏差的方差
%假设每次测量的时间是0.01s
delta_t = 0.01;                      %单位  秒

%% 线性动态方程模型参数和观测方程
A = [1,   0, delta_t,  0; 
     0,   1,    0,  delta_t; 
     0,   0,    1,     0;
     0,   0,    0,     1]; % 系统状态矩阵
Q = diag([car.x_noise, car.y_noise, car.v_x_noise, car.v_y_noise]); % 过程噪音的协方差矩阵

%% 存储数据初始化，包括坐标和测量值
ms   = 1000;       %采样次数
%% 实际航迹计算并显示
actual_state = []; %存储真实运动的状态变量值，后期用于对比
for k = 1:ms    %k表示第k次测量
    actual_state = [actual_state, car.State]; % 记录实际航迹
    state_noise = [randn * sqrt(car.x_noise);
                   randn * sqrt(car.y_noise);
                   randn * sqrt(car.v_x_noise);
                   randn * sqrt(car.v_y_noise)]; % 模拟实际航迹中的过程噪音
    car.State = A * car.State + state_noise; %在理想航迹上叠加过程噪音，制作出真实航迹
end

figure;subplot(121);
plot(actual_state(1,:),actual_state(2,:), '-');
xlabel('横坐标(cm)');ylabel('纵坐标(cm)');
title('无人机运动航迹');
axis([-40, 40, 0, 100]);

%% 数值重新初始化，用于记录测量数据
C = [1, 0, 0, 0; 
     0, 1, 0, 0];   % 系统位置的观测矩阵
%% GPS传感器1
sensor1.X_noise = 2;           %GPS的横向噪声方差
sensor1.Y_noise = 2;           %GPS的纵向噪声方差

R1 = diag([sensor1.X_noise, sensor1.Y_noise]); % 测量噪音的协方差矩阵

sensor1_X     = [];       %存储GPS横向坐标测量值
sensor1_Y     = [];       %存储GPS纵向坐标测量值

sensor1.State = [car.x0; 
             car.y0; 
             car.v0_x; 
             car.v0_y]; 

sensor1.X = sensor1.State(1,1);    %用于记录GPS的横向坐标测量值
sensor1.Y = sensor1.State(2,1);    %用于记录GPS的纵向坐标测量值   

for k = 1:ms    % 第k次测量
    sensor1_X = [sensor1_X,sensor1.X]; %记录GPS的测量数据
    sensor1_Y = [sensor1_Y,sensor1.Y];
    
    car.State = actual_state(:, k); %把真实航迹取出来
    sensor1.X = car.State(1,1) + randn * sqrt(sensor1.X_noise); %在真实航迹上面叠加测量噪音来制作测量数据
    sensor1.Y = car.State(2,1) + randn * sqrt(sensor1.Y_noise);
end

subplot(122);
plot(sensor1_X, sensor1_Y, '-');
legend('sensor1');
xlabel('横坐标(cm)');ylabel('纵坐标(cm)');
title('未经滤波算法计算轨迹的结果');
axis([-40, 40, 0, 100]);

%% 卡尔曼滤波估计最优状态
KF_State = [car.x0; 
            car.y0; 
            car.v0_x; 
            car.v0_y]; 
KF_err  = zeros(1, ms);
DIR_err = zeros(1, ms);
P1 = 0.01 * eye(4); %eye(4)代表4阶单位矩阵，这里是对估计均方误差矩阵Pk赋初值
for k = 1:ms
    Sensor_Meas1 = [sensor1_X(1,k);sensor1_Y(1,k)]; % 测量向量Zk
    
    [KF_State(:,k+1), P1] = Track_KF(P1, A, C, Q, R1, KF_State(:,k), Sensor_Meas1); 
    %调用卡尔曼滤波函数更新估计状态和估计均方误差矩阵
    KF_err(k) = norm(actual_state(1:2, k) - KF_State(1:2, k)); 
    DIR_err(k) = sqrt((actual_state(1, k) - sensor1_X(1,k))^2 + (actual_state(2, k) - sensor1_Y(1,k))^2);
end

%% 画出轨迹曲线
k = 1:ms;

figure;hold on;
subplot(121);
plot(KF_State(1,:), KF_State(2,:), 'r-');
legend('KF滤波的轨迹');
axis([-50, 50, 0, 100]);
xlabel('横坐标(cm)');ylabel('纵坐标(cm)');
title('滤波轨迹');

subplot(122);hold on;
plot(sensor1_X , sensor1_Y,'g-');
plot(KF_State(1,:), KF_State(2,:), 'r-');
plot(actual_state(1,:),actual_state(2,:),'b-');
legend('未滤波的轨迹', 'KF滤波的轨迹', '实际轨迹');
axis([-50, 50, 0, 100]);
xlabel('横坐标(cm)');ylabel('横坐标(cm)');
title('轨迹对比');

