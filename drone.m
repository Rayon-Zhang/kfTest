%%
% from：https://blog.csdn.net/weixin_40599145/article/details/113000157
%%
close all;
clear;
clc;
rng(2);
%% 参数初始化
%参数
car.x0   = -20;                    %car初始横向坐标
car.v0_x = 15;                     %car初始横向速度
car.y0   = 20;                     %car初始纵向坐标
car.v0_y = 20;                     %car初始纵向速度
car.z0   = 100;  
car.v0_z = 0;

car.State = [car.x0; 
             car.y0;
             car.z0;
             car.v0_x; 
             car.v0_y;
             car.v0_z
             ];            %状态变量

% 实际上car运行过程中会有一些偏差，所以这里模拟了一些过程噪音
car.x_noise   = 0.1;                % car运动的横向坐标偏差的方差
car.y_noise   = 0.1;                % car运动的纵向坐标偏差的方差
car.z_noise   = 0.1;               % car运动的纵向坐标偏差的方差
car.v_x_noise = 0.1;                % car运动的横向速度偏差的方差
car.v_y_noise = 0.1;                % car运动的纵向速度偏差的方差
car.v_z_noise = 0.1;               % car运动的纵向速度偏差的方差

%假设每次测量的时间是0.01s
delta_t = 0.01;                      %单位  秒

%% 线性动态方程模型参数和观测方程
A = [1,    0,    0,  delta_t,  0,    0; 
     0,    1,    0,     0,  delta_t, 0; 
     0,    0,    1,     0,     0, delta_t;
     0,    0,    0,     1,     0,    0;
     0,    0,    0,     0,     1,    0;
     0,    0,    0,     0,     0,    0]; % 系统状态矩阵
Q = diag([car.x_noise, car.y_noise, car.z_noise, car.v_x_noise, car.v_y_noise,car.v_z_noise]); % 过程噪音的协方差矩阵

%% 存储数据初始化，包括坐标和测量值
ms   = 1000;       %采样次数
%% 实际航迹计算并显示
actual_state = []; % 存储真实运动的状态变量值，后期用于对比
for k = 1:ms       % k表示第k次测量
    actual_state = [actual_state, car.State];    % 记录实际航迹
    state_noise = [randn * sqrt(car.x_noise);
                   randn * sqrt(car.y_noise);
                   randn * sqrt(car.z_noise);
                   randn * sqrt(car.v_x_noise);
                   randn * sqrt(car.v_y_noise);
                   randn * sqrt(car.v_z_noise)]; % 模拟实际航迹中的过程噪音
    car.State = A * car.State + state_noise;     % 在理想航迹上叠加过程噪音，制作出真实航迹
end

%% 数值重新初始化，用于记录测量数据
C = [1, 0, 0, 0, 0, 0; 
     0, 1, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1];   % 系统位置的观测矩阵
%% GPS传感器1
sensor1.cn0 = 20;

sensor1.X_noise = 2;           % GPS的横向噪声方差
sensor1.Y_noise = 2;           % GPS的纵向噪声方差
sensor1.Z_noise = 2;           % GPS的纵向噪声方差
sensor1.Vx_noise = 1;          % GPS的横向噪声方差
sensor1.Vy_noise = 1;          % GPS的纵向噪声方差
sensor1.Vz_noise = 1;          % GPS的纵向噪声方差

sensor1_X = [];       % 存储GPS横向坐标测量值
sensor1_Y = [];       % 存储GPS纵向坐标测量值
sensor1_Z = [];       % 存储GPS纵向坐标测量值
sensor1_Vx = [];       % 存储GPS横向坐标测量值
sensor1_Vy = [];       % 存储GPS纵向坐标测量值
sensor1_Vz = [];       % 存储GPS纵向坐标测量值

sensor1.State = [car.x0; 
                 car.y0; 
                 car.z0;
                 car.v0_x; 
                 car.v0_y;
                 car.v0_z]; 

sensor1.X = sensor1.State(1,1);         % 用于记录GPS的横向坐标测量值
sensor1.Y = sensor1.State(2,1);         % 用于记录GPS的纵向坐标测量值
sensor1.Z = sensor1.State(3,1);         % 用于记录GPS的纵向坐标测量值  
sensor1.Vx = sensor1.State(4,1);
sensor1.Vy = sensor1.State(5,1);
sensor1.Vz = sensor1.State(6,1);

for k = 1:ms                          
    sensor1_X = [sensor1_X,sensor1.X];  % 记录GPS的测量数据
    sensor1_Y = [sensor1_Y,sensor1.Y];
    sensor1_Z = [sensor1_Z,sensor1.Z];
    sensor1_Vx = [sensor1_Vx, sensor1.Vx];
    sensor1_Vy = [sensor1_Vy, sensor1.Vy];
    sensor1_Vz = [sensor1_Vz, sensor1.Vz];
    
    car.State = actual_state(:, k);                            
    sensor1.X = car.State(1,1) + randn * sqrt(sensor1.X_noise); % 在真实航迹上面叠加测量噪音来制作测量数据
    sensor1.Y = car.State(2,1) + randn * sqrt(sensor1.Y_noise);
    sensor1.Z = car.State(3,1) + randn * sqrt(sensor1.Z_noise);
    sensor1.Vx = car.State(4,1) + randn * sqrt(sensor1.Vx_noise);
    sensor1.Vy = car.State(5,1) + randn * sqrt(sensor1.Vy_noise);
    sensor1.Vz = car.State(6,1) + randn * sqrt(sensor1.Vz_noise);
end

%% GPS传感器2
sensor2.cn0 = 30;

sensor2.X_noise = 2;           % GPS的横向噪声方差
sensor2.Y_noise = 2;           % GPS的纵向噪声方差
sensor2.Z_noise = 2;           % GPS的纵向噪声方差
sensor2.Vx_noise = 1;          % GPS的横向噪声方差
sensor2.Vy_noise = 1;          % GPS的纵向噪声方差
sensor2.Vz_noise = 1;          % GPS的纵向噪声方差

sensor2_X = [];       % 存储GPS横向坐标测量值
sensor2_Y = [];       % 存储GPS纵向坐标测量值
sensor2_Z = [];       % 存储GPS纵向坐标测量值
sensor2_Vx = [];       % 存储GPS横向坐标测量值
sensor2_Vy = [];       % 存储GPS纵向坐标测量值
sensor2_Vz = [];       % 存储GPS纵向坐标测量值

sensor2.State = [car.x0; 
                 car.y0; 
                 car.z0;
                 car.v0_x; 
                 car.v0_y;
                 car.v0_z]; 

sensor2.X = sensor2.State(1,1);         % 用于记录GPS的横向坐标测量值
sensor2.Y = sensor2.State(2,1);         
sensor2.Z = sensor2.State(3,1);         
sensor2.Vx = sensor2.State(4,1);
sensor2.Vy = sensor2.State(5,1);
sensor2.Vz = sensor2.State(6,1);

for k = 1:ms                           
    sensor2_X = [sensor2_X,sensor2.X];  % 记录GPS的测量数据
    sensor2_Y = [sensor2_Y,sensor2.Y];
    sensor2_Z = [sensor2_Z,sensor2.Z];
    sensor2_Vx = [sensor2_Vx, sensor2.Vx];
    sensor2_Vy = [sensor2_Vy, sensor2.Vy];
    sensor2_Vz = [sensor2_Vz, sensor2.Vz];
    
    car.State = actual_state(:, k);                             % 把真实航迹取出来
    sensor2.X = car.State(1,1) + randn * sqrt(sensor2.X_noise); % 在真实航迹上面叠加测量噪音来制作测量数据
    sensor2.Y = car.State(2,1) + randn * sqrt(sensor2.Y_noise);
    sensor2.Z = car.State(3,1) + randn * sqrt(sensor2.Z_noise);
    sensor2.Vx = car.State(4,1) + randn * sqrt(sensor2.Vx_noise);
    sensor2.Vy = car.State(5,1) + randn * sqrt(sensor2.Vy_noise);
    sensor2.Vz = car.State(6,1) + randn * sqrt(sensor2.Vz_noise);
end

%% 高度传感器
sensor3.cn0 = 40;
sensor3.Z_noise = 0.0001;           % GPS的纵向噪声方差
sensor3_Z = [];                  % 存储GPS纵向坐标测量值
sensor3.State = [car.x0; 
                 car.y0; 
                 car.z0;
                 car.v0_x; 
                 car.v0_y;
                 car.v0_z];
sensor3.Z = sensor3.State(3,1); 

for k = 1:ms                           
    car.State = actual_state(:, k);                             % 把真实航迹取出来
    sensor3.Z = car.State(3,1);% + randn * sqrt(sensor3.Z_noise);
    sensor3_Z = [sensor3_Z,sensor3.Z];  % 记录GPS的测量
end

%% 参数设置
R1 = diag([sensor1.X_noise, sensor1.Y_noise, sensor1.Z_noise, sensor1.Vx_noise, sensor1.Vy_noise, sensor1.Vz_noise]); % 测量噪音的协方差矩阵
R2 = diag([sensor2.X_noise, sensor2.Y_noise, sensor2.Z_noise, sensor2.Vx_noise, sensor2.Vy_noise, sensor2.Vz_noise]); % 测量噪音的协方差矩阵
R3 = diag([sensor3.Z_noise]);

P = 0.01 * eye(6); % 对估计均方误差矩阵Pk赋初值 
P3 = 1 * eye(1);

C3 = [0, 0, 1, 0, 0, 0];

%% 卡尔曼滤波估计最优状态
KF_State = [car.x0; 
            car.y0; 
            car.z0;
            car.v0_x; 
            car.v0_y;
            car.v0_z]; 
KF_err  = zeros(1, ms);
DIR_err = zeros(1, ms);
for k = 1:ms
    [pred_state, P] = predict(P, A, Q, KF_State(:,k));
    
     % 测量向量Zk  
    Sensor_Meas1 = [sensor1_X(1,k);sensor1_Y(1,k); sensor1_Z(1,k); sensor1_Vx(1,k); sensor1_Vy(1,k); sensor1_Vz(1,k)];
    Sensor_Meas2 = [sensor2_X(1,k);sensor2_Y(1,k); sensor2_Z(1,k); sensor2_Vx(1,k); sensor2_Vy(1,k); sensor2_Vz(1,k)]; 
    
    [KF_State(:,k), P]   = update(P, C, Q, R1, pred_state, Sensor_Meas1);
    [KF_State(:,k + 1), P] = update(P, C, Q, R2, KF_State(:,k), Sensor_Meas2);

    Sensor_Meas3 = [sensor3_Z(1,k)];
    [KF_State(:,k+1), P3] = Track_KF(P3, A, C3, Q, R3, KF_State(:,k), Sensor_Meas3);

    KF_err(k) = norm(actual_state(1:3, k) - KF_State(1:3, k)); 
    DIR_err(k) = sqrt((actual_state(1, k) - sensor1_X(1,k))^2 + (actual_state(2, k) - sensor1_Y(1,k))^2 + (actual_state(3, k) - sensor1_Z(1,k))^2);
end

%% 画出轨迹曲线
k = 1:ms;

figure;
plot3(sensor1_X, sensor1_Y, sensor1_Z,'b-');hold on
plot3(sensor2_X, sensor2_Y, sensor2_Z, 'g-');hold on
plot3(actual_state(1,:),actual_state(2,:),actual_state(3,:), 'r','LineWidth',1.5); 
legend('sensor1','sensor2','actual');
xlabel('横坐标(cm)');ylabel('纵坐标(cm)');title('运动轨迹');
grid on

figure,
plot3(KF_State(1,:), KF_State(2,:), KF_State(3,:),'r-');hold on
plot3(actual_state(1,:),actual_state(2,:),actual_state(3,:),'b-');
legend('滤波轨迹', '实际轨迹');
xlabel('横坐标(cm)');ylabel('横坐标(cm)');title('轨迹对比');
grid on
% figure,plot(KF_err);
% xlabel('time');ylabel('err');title('kf误差')

figure,subplot(3,1,1)
plot((KF_State(1,1:end-1) - actual_state(1,:)));title('x方向误差');
subplot(3,1,2)
plot((KF_State(2,1:end-1) - actual_state(2,:)));title('y方向误差');
subplot(3,1,3)
plot((KF_State(3,1:end-1) - actual_state(3,:)));title('z方向误差');

%% 误差统计
KF_err_sorted = sort(KF_err);
disp('误差统计');
disp(['mean: ',num2str(mean(KF_err_sorted)), ' m']);
disp(['cep68: ', num2str(KF_err_sorted(ceil(length(KF_err_sorted) * 0.68))), ' m']);
disp(['cep95: ', num2str(KF_err_sorted(ceil(length(KF_err_sorted) * 0.95))), ' m']);
