 %% 火星动力下降段动力学方程
 % 对应论文式(1): 3-DoF 点质量模型
 % 状态 x = [rx, ry, rz, vx, vy, vz, m]^T  (7维)
 % 控制 u = [Tx, Ty, Tz]^T                 (3维)
 %
 % 输入:
 %   t  - 时间 [s]
 %   x  - 状态向量 [r; v; m] (7x1)
 %   T  - 推力矢量 [Tx; Ty; Tz] (3x1)
 %   g  - 火星重力加速度 (标量)
 %   lambda - 燃料消耗系数
 %
 % 输出:
 %   dx - 状态导数 (7x1)
 
 function dx = mars_dynamics(t, x, T, g, lambda)
     % 解包状态
     r = x(1:3);   % 位置
     v = x(4:6);   % 速度
     m = x(7);     % 质量
     
     % 重力加速度方向: x轴向上为正，所以重力沿-x方向
     g_vec = [-g; 0; 0];
     
     % 动力学方程
     dr = v;                    % 位置导数 = 速度
     dv = g_vec + T / m;        % 速度导数 = 重力 + 推力/质量
     dm = -lambda * norm(T);    % 质量导数 = -燃料消耗率
     
     dx = [dr; dv; dm];
 end
