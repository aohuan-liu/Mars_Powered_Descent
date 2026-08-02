 %% 训练神经网络近似优化求解器
 % 思路：用MPC求解器生成大量(状态 -> 最优控制)数据，
 % 训练神经网络学习这个映射，实现毫秒级推理
 %
 % 输入: 无（使用solve_fuel_optimal生成数据）
 % 输出: net - 训练好的神经网络
 
 function net = train_nn_approximator(params, num_samples)
     if nargin < 2
         num_samples = 200;  % 初始采样数
     end
     
     fprintf('生成训练数据 (共 %d 个样本)...\n', num_samples);
     
     % 存储数据
     X_data = [];
     Y_data = [];
     
     % 在不同初始条件下运行求解器
     for i = 1:num_samples
         % 随机化初始条件
         params_i = params;
         params_i.r0(1) = params.r0(1) + (rand - 0.5) * 200;  % ±100m
         params_i.v0(1) = params.v0(1) + (rand - 0.5) * 10;   % ±5 m/s
         
         % 求解
         [X_opt, U_opt, ~, status] = solve_fuel_optimal(params_i);
         
         if strcmp(status, 'Solved')
             % 提取每个时间步的 (状态, 控制) 对
             for k = 1:params.N
                 x_k = X_opt(1:7, k);     % 状态
                 u_k = U_opt(:, k);       % 比推力
                 X_data = [X_data, x_k];
                 Y_data = [Y_data, u_k];
             end
         end
     end
     
     fprintf('数据收集完成: %d 组 (状态->控制) 样本\n', size(X_data, 2));
     
     %% 搭建神经网络
     % 输入: 7维状态 (rx, ry, rz, vx, vy, vz, ln(m))
     % 输出: 3维控制 (ux, uy, uz)
     
     input_size = 7;
     output_size = 3;
     hidden_size = 64;
     
     layers = [
         featureInputLayer(input_size, 'Normalization', 'zscore', 'Name', 'input')
         fullyConnectedLayer(hidden_size, 'Name', 'fc1')
         reluLayer('Name', 'relu1')
         fullyConnectedLayer(hidden_size, 'Name', 'fc2')
         reluLayer('Name', 'relu2')
         fullyConnectedLayer(output_size, 'Name', 'output')
         regressionLayer('Name', 'reg')
     ];
     
     options = trainingOptions('adam', ...
         'MaxEpochs', 50, ...
         'MiniBatchSize', 32, ...
         'Plots', 'training-progress', ...
         'Verbose', false);
     
     %% 训练
     fprintf('训练神经网络...\n');
     net = trainNetwork(X_data, Y_data', layers, options);
     
     %% 验证
     fprintf('\n--- 验证 ---\n');
     Y_pred = predict(net, X_data);
     mse = mean((Y_pred - Y_data').^2, 'all');
     fprintf('训练集 MSE: %.6f\n', mse);
     
     %% 用NN做一次闭环仿真
     fprintf('\n运行NN闭环仿真...\n');
     x_nn = [params.r0; params.v0; params.m_wet];
     hist_nn = x_nn;
     for k = 1:params.N+1
         % NN推理
         u_nn = predict(net, x_nn);
         % 重构推力
         T_nn = u_nn' * x_nn(7);
         % 约束推力幅值
         if norm(T_nn) > params.T_max
             T_nn = T_nn / norm(T_nn) * params.T_max;
         end
         if norm(T_nn) < params.T_min
             T_nn = T_nn / norm(T_nn) * params.T_min;
         end
         % 推进
         dx = [x_nn(4:6); params.g_mars*[-1;0;0] + T_nn/x_nn(7); -params.lambda*norm(T_nn)];
         x_nn = x_nn + params.dt * dx;
         hist_nn = [hist_nn, x_nn];
     end
     
     % 保存结果
     save('results/nn_trained.mat', 'net', 'X_data', 'Y_data');
     save('results/nn_simulation.mat', 'hist_nn');
     fprintf('NN仿真完成，结果已保存\n');
 end
