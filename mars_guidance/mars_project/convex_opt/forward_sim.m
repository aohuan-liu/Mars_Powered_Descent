 %% 前向仿真：给定控制序列，用欧拉法积分出完整轨迹
 % 在跑 CVX 优化之前，先用这个函数验证动力学方程是否正确
 %
 % 输入:
 %   T_seq - 推力序列 [3 x N] 矩阵, 每列是一个时刻的推力矢量
 %   params - 结构体, 包含所有参数
 %
 % 输出:
 %   t_hist - 时间向量 [(N+1) x 1]
 %   x_hist - 状态历史 [7 x (N+1)]
 
 function [t_hist, x_hist] = forward_sim(T_seq, params)
     % 提取参数
     g = params.g_mars;
     lam = params.lambda;
     dt = params.dt;
     N = params.N;
     
     % 初始状态
     x0 = [params.r0; params.v0; params.m_wet];
     
     % 存储
     t_hist = (0:N)' * dt;
     x_hist = zeros(7, N+1);
     x_hist(:, 1) = x0;
     
     % 欧拉法积分
     for k = 1:N
         xk = x_hist(:, k);
         Tk = T_seq(:, k);  % 当前时刻推力
         
         % 动力学导数
         g_vec = [-g; 0; 0];
         rk = xk(1:3);
         vk = xk(4:6);
         mk = xk(7);
         
         dr = vk;
         dv = g_vec + Tk / mk;
         dm = -lam * norm(Tk);
         
         dx = [dr; dv; dm];
         
         % 欧拉法推进
         x_hist(:, k+1) = xk + dt * dx;
         
         % 防止质量低于干重
         if x_hist(7, k+1) < params.m_dry
             x_hist(7, k+1) = params.m_dry;
         end
     end
 end
