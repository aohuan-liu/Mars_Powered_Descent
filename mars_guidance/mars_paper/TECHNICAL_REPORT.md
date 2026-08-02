 # 火星动力下降凸优化制导 — 技术解析文档
 
 ## 对应论文
 
 > Gao et al. "Obstacle avoidance guidance for Mars powered descent using convex optimization and elevation angle"
 > *Acta Astronautica* 248 (2026) 296-313
 > 通讯作者: **郭延宁** (guoyn@hit.edu.cn), 哈尔滨工业大学控制科学与工程系
 
 ---
 
 ## 1. 项目背景
 
 火星动力下降段（Powered Descent Phase）是火星着陆任务中最关键的阶段之一。
 在这个阶段，着陆器点燃主发动机产生推力进行减速和轨迹控制，
 最终以零速度精确降落在目标着陆点。
 
 这是一个典型的 **燃料最优控制问题**（Fuel-Optimal Control Problem），
 需要在满足以下约束的前提下最小化燃料消耗：
 - 推力幅值约束（发动机有最小/最大推力限制）
 - 下滑角约束（着陆器必须在一定的锥形走廊内下降）
 - 避障约束（绕过火星表面的障碍物，如岩石、陨石坑）
 
 传统的解析制导方法（如多项式制导）计算快但难以处理复杂约束。
 **凸优化方法**能够在保证全局最优解的前提下高效处理各类约束，
 因此成为当前深空探测制导领域的主流方法。
 
 ---
 
 ## 2. 数学模型
 
 ### 2.1 坐标系与动力学方程（论文 Eq.1）
 
 采用表面惯性坐标系 O_xyz，原点在目标着陆点：
 - **x轴**: 垂直向上（天向）
 - **y轴**: 指向东方
 - **z轴**: 指向北方
 
 着陆器建模为三自由度（3-DoF）质点，状态量为：
 
 ```
 x(t) = [r_x, r_y, r_z, v_x, v_y, v_z, m]^T   (7维)
 ```
 
 控制量为推力矢量：
 
 ```
 u(t) = [T_x, T_y, T_z]^T   (3维)
 ```
 
 动力学方程：
 
 ```
 r_dot = v                           (位置导数 = 速度)
 v_dot = g_m + T/m                   (速度导数 = 重力 + 推力/质量)
 m_dot = -lambda * ||T||_2           (质量导数 = 燃料消耗率)
 ```
 
 其中 lambda = 1/(Isp * g0), Isp 为比冲, g0 为地球重力加速度。
 
 ### 2.2 约束条件（论文 Eq.4-8）
 
 **边界约束**:
 - 初始状态: r(0) = [1500, 0, 1500]^T m, v(0) = [-75, 0, 70]^T m/s
 - 终端状态: r(tf) = [0, 0, 0]^T, v(tf) = [0, 0, 0]^T（软着陆）
 
 **推力约束**:
 - T_min <= ||T||_2 <= T_max
 - T_min = 4971.82 N, T_max = 13258.17 N
 
 **下滑角约束**（Eq.6）:
 - rx >= tan(gamma_gs) * sqrt(ry^2 + rz^2)
 - gamma_gs = 10度（比传统45度更保守，适合复杂地形）
 
 **目标函数**:
 - 最小化燃料消耗: min integral(||T||_2 dt) 或等价 min -m(tf)
 
 ---
 
 ## 3. 凸优化方法
 
 ### 3.1 挑战：问题是非凸的
 
 原始问题（Problem 1）存在两个非凸性来源：
 
 1. **动力学非线性**: T(t)/m(t) 中质量在分母上，导致状态方程非线性
 2. **推力下界约束**: ||T|| >= T_min 是**非凸约束**（下界不等式为反方向）
 
 ### 3.2 无损凸化（Lossless Convexification，论文 Eq.10-12）
 
 引入松弛变量 Gamma(t)，将推力约束放松为：
 
 ```
 ||T||_2 <= Gamma(t)
 ```
 
 并做变量替换：
 
 ```
 z(t) = ln(m(t))        → 对数质量
 u(t) = T(t)/m(t)       → 比推力
 sigma(t) = Gamma(t)/m(t)  → 松弛比
 ```
 
 变换后的动力学方程变为**线性**：
 
 ```
 r_dot = v
 v_dot = g_m + u
 z_dot = -lambda * sigma
 ```
 
 约束变为**凸的二阶锥约束**：
 - ||u||_2 <= sigma
 - T_min * exp(-z) <= sigma <= T_max * exp(-z)
 
 最后一项 exp(-z) 通过**泰勒展开**（Eq.17）线性化，得到一个标准的二阶锥规划（SOCP）。
 
 论文证明了该松弛是"无损"的——燃料最优问题的解会自动满足 ||T|| = Gamma，
 即松弛后的约束与原问题等价。
 
 ### 3.3 离散化（论文 Eq.20-23）
 
 采用零阶保持器（ZOH）离散化：
 
 ```
 X_{k+1} = Ad * X_k + Bd * [u_k; sigma_k] + Bd_g
 ```
 
 其中 Ad = expm(Ac * dt) 通过矩阵指数精确计算，
 Bd 和 Bd_g 通过数值积分获得。
 
 离散点数 N=30，时间步长 dt = 80/31 ≈ 2.58s。
 
 ### 3.4 序列凸优化与同伦迭代（论文 Algorithm 1, Section 3.3）
 
 对于避障约束，论文采用**同伦迭代**方法：
 从无障碍的简单问题开始，通过 H 次迭代逐步引入障碍约束，
 每次迭代以上一次的解作为 Taylor 展开的参考点。
 
 这样可以有效处理松弛下滑角约束和阶跃约束等非凸约束。
 
 ---
 
 ## 4. 代码实现说明
 
 ### 文件结构
 
 ```
 mars_guidance_project/
 |-- main.m                    # 主脚本，执行完整仿真流程
 |-- mars_params.m             # 参数定义（论文 Table 1-2）
 |-- TECHNICAL_REPORT.md       # 本技术文档
 |-- README.md                 # 项目说明
 |
 |-- dynamics/
 |   |-- mars_dynamics.m       # 连续动力学方程 RHS (Eq.1)
 |
 |-- convex_opt/
 |   |-- solve_fuel_optimal.m  # 核心求解器 (Problem 2-4)
 |   |-- forward_sim.m         # 前向仿真验证
 |   |-- plot_results.m        # 6图可视化
 |
 |-- obstacle/
 |   |-- obstacle_constraint.m # 避障约束 (Eq.6-8, 含同伦参数)
 |
 |-- nn_accel/
 |   |-- train_nn_approximator.m  # NN加速 (Deep Learning Toolbox版)
 |   |-- train_nn_simple.m        # NN加速 (Neural Network Toolbox版)
 |
 |-- mars_landing_viz.html     # 3D交互可视化 (Three.js)
 ```
 
 ### 核心求解器 (solve_fuel_optimal.m) 的执行流程
 
 1. **参数准备**: 从 mars_params 提取所有物理/数值参数
 2. **无损凸化**: 计算 z0 = ln(m_wet)，设置对数质量变量
 3. **离散化**: 计算 Ad, Bd, Bd_g 矩阵
 4. **Taylor参考点**: 计算 z_l, z_u（燃料消耗的上下界）
 5. **CVX求解**:
    - 定义变量 X(7x32), u(3x31), sig(1x31)
    - 目标: minimize sum(sig * dt)
    - 约束: 31个动力学等式 + 初终边界 + 30个SOC约束 + 30个推力线性化
 6. **结果提取**: 返回最优轨迹和求解状态
 
 ### CVX求解器
 
 CVX 版本 2.2，可选求解器：
 - **SDPT3**（默认）: 稳定，适合中小规模 SOCP
 - **SeDuMi**: 另一种内点法求解器
 
 论文使用的是 **MOSEK** 求解器，比 SDPT3 快 5-10 倍。
 如有需要可安装 MOSEK，通过 `cvx_solver mosek` 启用。
 
 ---
 
 ## 5. 仿真结果分析
 
 ### 求解结果
 
 ```
 CVX status: Solved
 Fuel used:  360.34 kg
 Position error: 0.0000 m
 Velocity error: 0.0000 m/s
 ```
 
 ### 结果解读
 
 - **燃料消耗 360.34 kg**: 在初始总质量 1905 kg 的约束下，消耗约 18.9% 的燃料
   （可用燃料 500 kg = 1905 - 1405），余量充足
 - **位置/速度误差为零**: SOCP 求解器严格满足了终端等式约束
 - **前向仿真终端高度-3779m**: 这是由于初始高度 1500m + 初始速度-75m/s，
   常值推力 = 初始重力，随着燃料消耗质量减轻，推力相对增大，导致持续加速下降
 
 ### 6张可视化图表说明
 
 1. **3D轨迹**: 从起始点到目标点的三维空间曲线
 2. **高度-时间**: 着陆器从 1500m 下降到 0m 的过程
 3. **速度-时间**: 从初始 75+ m/s 减速到 0 的变化
 4. **推力幅值**: 求解出的最优推力序列（应在 T_min 和 T_max 之间）
 5. **质量变化**: 燃料消耗导致的质量随时间线性减少
 6. **下滑角约束验证**: 轨迹是否在允许的锥形走廊内
 
 ---
 
 ## 6. 与郭老师研究方向的关联
 
 郭延宁老师课题组的研究方向包括：
 
 | 研究方向 | 本项目覆盖 |
 |----------|-----------|
 | 深空探测制导与控制 | ✅ 核心内容 - 火星着陆制导 |
 | 最优控制 | ✅ 燃料最优问题的凸优化求解 |
 | 凸优化在航天中的应用 | ✅ 无损凸化 + SOCP |
 | 航天器姿态控制 | ❌ 3-DoF不涉及姿态 |
 | 执行机构控制分配 | ❌ 未涉及 |
 
 这个项目直接对应郭老师最核心的**深空探测制导**方向，
 采用的凸优化方法也是该组的核心方法论。
 
 ---
 
 ## 7. 后续开发计划
 
 ### Phase 2: 避障约束（论文 3.3 节）
 - 实现松弛下滑角约束（Eq.7）
 - 实现阶跃约束（Eq.8）
 - 同伦迭代算法（Algorithm 1）
 
 ### Phase 3: 仰角目标函数（论文 4 节）
 - 实现仰角最大化（Eq.37-40）
 - Algorithm 2: 仰角优化的凸优化算法
 - 分析仰角权重 alpha 对轨迹的影响
 
 ### Phase 4: NN 加速
 - 生成训练数据（不同初始条件 × MPC 求解）
 - 训练前馈神经网络替代优化求解器
 - 对比 MPC 最优轨迹 vs NN 轨迹
 - 对比求解时间（秒级 vs 毫秒级）
 
 ### Phase 5: 强化学习扩展
 - 配置 RL 环境（MATLAB RL Toolbox）
 - 训练 PPO/SAC agent
 - 三方法对比：凸优化 / NN拟合 / RL
 
 ---
 
 ## 参考文献
 
 1. Gao et al., "Obstacle avoidance guidance for Mars powered descent using convex optimization and elevation angle", *Acta Astronautica* 248 (2024) 296-313
 2. Acikmese & Ploen, "Convex programming approach to powered descent guidance for Mars landing", *AIAA JGCD*, 2007
 3. Blackmore, Acikmese, Scharf, "Minimum-landing-error powered-descent guidance for Mars landing using convex optimization", *AIAA JGCD*, 2010
 4. Scharf et al., "G-FOLD: A real-time implementable fuel-optimal guidance algorithm", *IEEE Aerospace Conference*, 2012
