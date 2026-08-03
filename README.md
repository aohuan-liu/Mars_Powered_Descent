# 火星动力下降凸优化制导

基于论文 [1] 的 MATLAB 实现
> Gao et al. "Obstacle avoidance guidance for Mars powered descent using convex optimization and elevation angle"
> Acta Astronautica 248 (2026) 296-313
> 通讯作者: 郭延宁 (guoyn@hit.edu.cn)

## 项目实现

基于论文完整复现了火星动力下降段的凸优化制导算法（MATLAB + CVX/SOCP）：

- **燃料最优 SOCP 求解（Problem 4）**：`convex_opt/solve_pd_socp.m` 统一实现
  无损凸化（z/u/σ 变量替换）、ZOH 离散化与泰勒展开线性化，可选用例加入
  Eq.6 常规下滑角约束。无约束基准燃料 360.33 kg；加 Eq.6 后 360.71 kg
  （轨迹贴住锥面，裕度 0.00 m）。
- **同伦避障（Algorithm 1）**：`convex_opt/solve_algorithm1.m` 实现松弛/阶跃
  避障约束 + 同伦参数 δ + 松弛变量 ζ，复现论文两个场景：
  tf=80 实测 365.29 kg（论文 365.18）、tf=100 实测 409.47 kg（论文 409.12）。
- **仰角优化（Algorithm 2）**：`convex_opt/solve_algorithm2.m` 实现轴加权位置
  目标 + 固定燃料约束，α 扫描仰角积分峰值在 α=1.5，落在论文 [0.5,2] 区间。
- **验证与证据**：验证脚本逐场景对照论文数字，结果数据与图保存在
  `results/`；详细记录见 `mars_paper/TECHNICAL_REPORT.md` §8。
- **自主拓展**：NN 加速（`nn_accel/`）方法论待修正（开环轨迹 ≠ 反馈策略），
  暂不可用；RL 对比（PPO/SAC）待实现。

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
│   │   ├── DETAILED_PAPER_GUIDE.md #   ★唯一指导文件：论文×代码分阶段学习指南
│   │   └── mars_landing_viz.html   #   3D 交互可视化
│   └── mars_project/
│       ├── main.m                  #   主脚本
│       ├── mars_params.m           #   所有参数
│       ├── dynamics/
│       │   └── mars_dynamics.m     # [论文] 动力学方程 (Eq.1)
│       ├── convex_opt/
│       │   ├── forward_sim.m       # [论文] 前向仿真（ZOH 精确离散，可选 RK4）
│       │   ├── solve_pd_socp.m      # [论文] 核心 SOCP 求解器 (Problem 4/5/6/7)
│       │   ├── solve_fuel_optimal.m # [论文] Problem 4 (可选 Eq.6)
│       │   ├── solve_algorithm1.m   # [论文] Algorithm 1 (同伦+松弛)
│       │   ├── solve_algorithm2.m   # [论文] Algorithm 2 (仰角优化)
│       │   ├── zoh_matrices.m       # [论文] Ad/Bd/Bd_g 幂零闭式 (Eq.23-24)
│       │   ├── trajectory_metrics.m # 燃料/裕度/仰角积分指标
│       │   └── plot_results.m      # [论文] 可视化
│       ├── obstacle/
│       │   └── obstacle_constraint.m # [论文] 避障约束 (Eq.6/7/8, 含同伦参数)
│       ├── run_milestone1.m        # 里程碑1: Eq.6 下滑角约束
│       ├── run_milestone2.m        # 里程碑2: Algorithm 1
│       ├── run_milestone3.m        # 里程碑3: Algorithm 2
│       ├── export_figures.m        # 由 .mat 结果导出 PNG
│       ├── results/                # 结果 .mat 与 PNG
│       └── nn_accel/
│           ├── README.md                # [拓展] 方法论待修正，勿用
│           ├── train_nn_approximator.m  # [拓展] NN 加速 (DL Toolbox)
│           └── train_nn_simple.m        # [拓展] NN 加速 (简易版)
│
└── OCRL/                           # OCR 学习资料
    ├── HW/                         #   作业 (HW0-4)
    ├── HW_Solution/                #   作业解答
    └── lecture-notebooks/          #   课程讲义 (不含PDF)
```

> simulink-agentic-toolkit 需从 https://github.com/matlab/simulink-agentic-toolkit 单独克隆。

## 学习路线

课程（CMU 16-745 OCRL）× 论文 × 项目 的**总大纲**见 [ROADMAP.md](ROADMAP.md)：
以论文指南的阶段为主键，标注每阶段对应学哪些课程、做哪些作业，以及**你自己的学习状态**。
**论文学习唯一入口**：[DETAILED_PAPER_GUIDE.md](mars_guidance/mars_tutorial/DETAILED_PAPER_GUIDE.md)
（只针对论文、按阶段推进：每阶段"读哪页 → 对应公式 → 对应代码 → 验证效果 → 给代码注释"）。
进度统一记录在下方学习笔记。

## 论文 vs 拓展

| 部分 | 来源 | 状态 |
|------|------|------|
| 动力学建模 (Eq.1) | 论文 Section 2.1 | 已实现 |
| 无损凸化 + SOCP 求解 (Section 3.1-3.2) | 论文核心 | 已实现（基准 360.33 kg） |
| 常规下滑角约束 Eq.6 | 论文 Section 2.3 | 已实现（tf=80: 360.71 kg，见技术报告 8.1） |
| 同伦迭代避障 (Section 3.3-3.4, Algorithm 1) | 论文 | 已实现（365.29/409.47 vs 365.18/409.12） |
| 仰角最大化 (Section 4, Algorithm 2) | 论文创新点 | 已实现（α 峰值 1.5 ∈ [0.5,2]） |
| NN 加速 (nn_accel/) | **自主拓展** | 方法论需修正（开环轨迹≠反馈策略），暂不可用 |
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

论文公式 ↔ 代码文件的对应关系见 [ROADMAP.md](ROADMAP.md) 贯通总表。

## 提交记录

> **2026-08-03 14:38 封存**：项目第一阶段已完成，等待初试后学习，已封存。

### 2026-08-03 · 审查整改轮（未提交，待拆分提交）

- **隐私清理**：AGENTS.md 与 matlab_mcp 工作脚本去掉硬编码绝对路径；OCRL 55 个 notebook 输出清理（nbstripout），`HW0/.vscode/settings.json` 删除
- **正确性**：新增松弛紧性事后检查（max|σ-‖u‖₂| 与原始推力界 T_min≤m·σ≤T_max）；forward_sim 改为与求解器一致的 ZOH 精确离散（可选 RK4 连续对照）；Algorithm 1/2 收敛注释按论文第 12 步修正；run_milestone3 的 H=5 重试污染修复（副本重试）
- **数值**：新增 zoh_matrices.m 幂零闭式（Ad/Bd/Bd_g，与 expm/数值积分差 ~1e-15）
- **文档**：TECHNICAL_REPORT 参考文献年份修正为 2026；指南与技术报告同步紧性证据链
- **项目现状**：论文三部分全部复现并验证——Problem 4 基准 360.33 kg、加 Eq.6 后 360.71 kg；Algorithm 1 tf=80: 365.29（论文 365.18）、tf=100: 409.47（论文 409.12）；Algorithm 2 α 峰值 1.5 ∈ [0.5,2]；松弛紧性 ~1e-8，原始推力界满足

### 2026-08-02 · 论文全量复现（已提交，含 b163fda / 2e879b1 / d93de76 / 9a7ea8a / 91487ac）

- 项目结构整理（三层架构 + .gitignore）；README 路径与结构修正
- 统一 Problem 4/5/6/7 求解器，实现 Algorithm 1（同伦+松弛）与 Algorithm 2（仰角优化）
- 里程碑脚本、结果 .mat/PNG、技术报告与分阶段学习指南
- **项目现状**：完成论文全链路实现，核心数字对照论文（360.33 / 360.71 / 365.29 / 409.47 / α=1.5）

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
