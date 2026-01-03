%% 卡尔曼滤波子函数
function [State, P] = Track_KF(P, A, H, Q, R, prev_state, Sensor_Meas)
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
%% Predict
pred_state = A * prev_state;                                % 预测状态
pred_P = A * P * A' + Q;                                    % 预测误差的协方差矩阵
innov = Sensor_Meas - H * pred_state;

% if innov(1) > 3 || innov(2) > 3 || innov(3) > 2 || innov(4) > 2
%     State = prev_state;
%     disp('meansure err');
%     return;
% end

%% Update
S = inv(H * pred_P * H' + R);                               % 矩阵求逆
K = pred_P * H' * S;                                        % 计算卡尔曼增益
State = pred_state + K * innov;    % 状态更新方程
P = (eye(N) - K * H) * pred_P;                              % 估计均方误差矩阵的更新，用于下一次迭代
end
