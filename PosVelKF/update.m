%% 卡尔曼滤波子函数
function [State, P] = update(P, H, Q, R, pred_state, Sensor_Meas)
%Track_KF 此处显示有关此函数的摘要
% State 最优估计状态
% P 估计均方误差矩阵
% A 系统矩阵
% H 观测矩阵
% Q 过程噪音的协方差矩阵
% R 测量噪音的协方差矩阵
% prev_state 前一时刻的状态
% Sensor_Meas 当前时刻的测量向量
N = size(Q, 1); % 计算状态变量的个数
innov = Sensor_Meas - H * pred_state;
%% Update
S = inv(H * P * H' + R);                               % 矩阵求逆
K = P * H' * S;                                        % 计算卡尔曼增益
State = pred_state + K * innov;    % 状态更新方程
P = (eye(N) - K * H) * P;                              % 估计均方误差矩阵的更新，用于下一次迭代
end
