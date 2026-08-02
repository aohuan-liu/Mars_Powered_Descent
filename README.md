# 火星动力下降凸优化制导

基于论文 [1] 的 MATLAB 实现
> Gao et al. "Obstacle avoidance guidance for Mars powered descent using convex optimization and elevation angle"
> Acta Astronautica 248 (2026) 296-313
> 通讯作者: 郭延宁 (guoyn@hit.edu.cn)

## 项目结构

```
mars_powered_descent/
├── .gitignore                      # 排除: .codex/ .agents/ OCRL PDF/ simulink子仓库
├── AGENTS.md                       # Codex 工作流指令
├── README.md                       # 本文件
│
├── matlab_mcp/                     # MATLAB MCP 服务器
│   ├── work/                       #   工作脚本和模型
│   │   ├── start_simulink_codex.m
│   │   ├── build_closed_loop_speed_control.m
│   │   ├── build_satellite_orbit_model.m
│   │   ├── closed_loop_speed_control.slx
│   │   ├── satellite_orbit_sim.slx
│   │   └── sine_test.m
│   ├── slprj/                      #   Simulink 缓存
│   └── simulink-agentic-toolkit/   #   ■ 本地存在，git忽略（需单独clone）
│
├── mars_guidance/                  # ★ 火星着陆制导项目
│   ├── mars_paper/
│   │   └── TECHNICAL_REPORT.md     #   技术解析文档
│   ├── mars_tutorial/
│   │   ├── PAPER_READING_GUIDE.md  #   论文阅读指南
│   │   └── mars_landing_viz.html   #   3D 交互可视化
│   └── mars_project/
│       ├── main.m                  #   主脚本
│       ├── mars_params.m           #   所有参数
│       ├── dynamics/
│       │   └── mars_dynamics.m     # [论文] 动力学方程 (Eq.1)
│       ├── convex_opt/
│       │   ├── forward_sim.m       # [论文] 前向仿真
│       │   ├── solve_fuel_optimal.m # [论文] 凸优化求解器
│       │   └── plot_results.m      # [论文] 可视化
│       ├── obstacle/
│       │   └── obstacle_constraint.m # [论文] 避障约束 (Eq.6/7/8, 含同伦参数)
│       └── nn_accel/
│           ├── train_nn_approximator.m  # [拓展] NN 加速
│           └── train_nn_simple.m        # [拓展] NN 加速
│
└── OCRL/                           # OCR 学习资料
    ├── HW/                         #   作业 (HW0-4)
    ├── HW_Solution/                #   作业解答
    └── lecture-notebooks/          #   课程讲义 (不含PDF)
```

> simulink-agentic-toolkit 需从 https://github.com/matlab/simulink-agentic-toolkit 单独克隆。

## 学习路线

课程（CMU 16-745 OCRL）× 论文 × 项目 的贯通路线见 [ROADMAP.md](ROADMAP.md)，两个线程（课程 / 项目）的进度统一记录在下方学习笔记。

## 论文 vs 拓展

| 部分 | 来源 | 状态 |
|------|------|------|
| 动力学建模 (Eq.1) | 论文 Section 2.1 | 已实现 |
| 无损凸化 + SOCP 求解 (Section 3.1-3.2) | 论文核心 | 已实现 |
| 同伦迭代避障 (Section 3.3-3.4) | 论文 | 待实现 |
| 仰角最大化 (Section 4) | 论文创新点 | 待实现 |
| NN 加速 (nn_accel/) | **自主拓展** | 待实现 |
| RL 对比 (PPO/SAC) | **自主拓展** | 待实现 |

## 前置条件

1. MATLAB (R2020b+)
2. CVX 工具箱: https://cvxr.com/cvx/download/
   - 下载后解压，在 MATLAB 中运行 cvx_setup
3. (可选) Deep Learning Toolbox — NN 拓展需要
4. (可选) Reinforcement Learning Toolbox — RL 拓展需要

## 运行步骤

```matlab
>> cd mars_powered_descent/mars_guidance/mars_project
>> addpath(genpath('C:\My App\Matlab\cvx'))
>> main
```

## 论文对应关系

| 论文公式 | 代码文件 | 说明 |
|----------|----------|------|
| Eq.1 | mars_dynamics.m | 3-DoF动力学 |
| Eq.4-5 | mars_params.m | 边界约束 |
| Eq.6-8 | obstacle_constraint.m | 避障约束 (含同伦参数 δ) |
| Eq.10-12 | solve_fuel_optimal.m | 无损凸化 |
| Eq.20-23 | solve_fuel_optimal.m | ZOH离散化 |
| Problem 2-4 | solve_fuel_optimal.m | SOCP求解 |
| Algorithm 1 | (后续补充) | 同伦迭代 |
| Algorithm 2 | (后续补充) | 仰角优化 |

## 学习笔记

### 7.26
- 1 论文 Abstract
  - 用同伦迭代+凸优化+仰角解决火星软着陆避障问题
- 2 论文 Introduction
  - 对比三种制导方式：解析制导、强化学习制导、最优化制导
  - 解析制导：难以处理推力幅值约束、参数敏感和燃料最优问题
  - 强化学习：计算快，但可靠性不够
  - 最优化：更直接且有效，但很多数学问题是非凸的，需要引入同伦迭代
- 3 建模
  - 飞行器建模为质点，理解动力学方程。核心约束是推力大小。
    由于发动机不能关机，存在最小推力，导致数学上的麻烦：
    ||T|| >= T_min > 0 是下界约束，下界约束天然非凸。
    这是后面要解决的核心难题之一。
  - 以目标点为原点建立惯性坐标系。传统下滑角约束：
    以原点为顶点建立倒立锥，着陆器要始终在锥外面飞行。

### 8.2
- 1 OCRL
  - HW0
