 %% 松弛下滑角约束 / 阶跃约束
 % 对应论文式(7)(8)及图2
 %
 % 松弛下滑角约束（式7）：
 % 当水平距离 d < l1: 无约束
 % 当 l1 ≤ d < l2: 高度 ≥ (d - l1)*tan(γ_new)
 % 当 d ≥ l2: 高度 ≥ h_safe
 %
 % 输入:
 %   r_horiz - 水平距离 [m] = sqrt(ry^2 + rz^2)
 %   altitude - 当前高度 [m] = rx
 %   type - 'relaxed'（式7）或 'stepwise'（式8）
 %
 % 输出:
 %   violation - 是否违反约束 (true/false)
 %   margin - 安全裕度 [m]（正数表示满足约束）
 
 function [violation, margin] = relaxed_glide_slope(r_horiz, altitude, params, type)
     if nargin < 4
         type = 'relaxed';
     end
     
     l1 = params.l1;
     l2 = params.l2;
     h = params.h_safe;
     gamma_new = h / (l2 - l1);  % tan(γ_new)
     
     if strcmp(type, 'relaxed')
         % 式(7): 松弛下滑角约束
         if r_horiz <= l1
             limit = 0;
         elseif r_horiz < l2
             limit = (r_horiz - l1) * gamma_new;
         else
             limit = h;
         end
     elseif strcmp(type, 'stepwise')
         % 式(8): 阶跃约束
         l_step = params.l_step;
         h_step = params.h_step;
         if r_horiz <= l_step
             limit = h_step;
         else
             limit = 0;
         end
     else
         error('未知约束类型: %s', type);
     end
     
     margin = altitude - limit;
     violation = margin < 0;
 end
