 %% 简易版神经网络训练（无需Deep Learning Toolbox）
 % 使用MATLAB自带的feedforwardnet
 % 如果没有Deep Learning Toolbox，用这个替代train_nn_approximator.m
 
 function net_alt = train_nn_simple(X_data, Y_data)
     % 输入: X_data (7xN), Y_data (3xN)
     % 输出: net_alt - 训练好的网络
     
     % 创建前馈网络：2个隐藏层，每层32个神经元
     net_alt = feedforwardnet([32, 32]);
     
     % 训练参数
     net_alt.trainParam.epochs = 200;
     net_alt.trainParam.goal = 1e-5;
     net_alt.divideParam.trainRatio = 0.8;
     net_alt.divideParam.valRatio = 0.2;
     net_alt.divideParam.testRatio = 0;
     
     % 训练
     [net_alt, tr] = train(net_alt, X_data, Y_data);
     
     fprintf('训练完成 (共 %d 轮)\n', tr.num_epochs);
     fprintf('训练集 MSE: %.6f\n', tr.best_perf);
     fprintf('验证集 MSE: %.6f\n', tr.best_vperf);
 end
