%% 卡尔曼滤波子函数
function [pred_state, pred_P] = predict(P, A, Q, prev_state)
%Track_KF 此处显示有关此函数的摘要
% State 最优估计状态
% P 估计均方误差矩阵
% A 系统矩阵
% H 观测矩阵
% Q 过程噪音的协方差矩阵
% R 测量噪音的协方差矩阵
% prev_state 前一时刻的状态
% Sensor_Meas 当前时刻的测量向量
%% Predict
pred_state = A * prev_state;                                    % 预测状态
pred_P = A * P * A' + Q;                                        % 预测误差的协方差矩阵
end
