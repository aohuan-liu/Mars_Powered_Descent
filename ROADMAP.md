# 学习路线：课程 × 论文 × 项目（ROADMAP = 大纲）

本文档是 **OCRL 课程线程** 与 **Mars 项目线程** 的共享总纲：
以 [DETAILED_PAPER_GUIDE.md](mars_guidance/mars_tutorial/DETAILED_PAPER_GUIDE.md) 的阶段为主键，
每个阶段标注"读论文哪里 → 对应课程学哪些 → 做什么作业 → 项目代码在哪 → 学习状态"。
进度统一记录在 README.md 的学习笔记里。

## 状态的含义

表格中的**状态列 = 你自己的学习状态**，不是代码实现状态。
代码已全部实现并验证（见下），但只有你按指南走完五步（读页 → 抄公式 →
找代码 → 验证 → 亲手写注释）并通过该阶段"验收"标准，才能打 ✅。

- ⬜ 未开始
- 🔄 学习中（课程 / 论文 / 代码三件套任一进行中）
- ✅ 已验收（通过指南"验收"标准，能自己讲出来）

## 项目代码基线（2026-08-03，仅供参考，不算学习进度）

论文全部算法已实现并验证：
- Problem 4（可选 Eq.6 下滑角）：基准 360.33 kg，加约束 360.71 kg
- Algorithm 1（同伦+松弛）：365.29 / 409.47 kg（论文 365.18 / 409.12，偏差 <0.1%）
- Algorithm 2（仰角目标）：α 峰值 1.5，落在论文 [0.5,2] 区间
- 2026-08-03 审查整改轮新增：松弛紧性事后检查（max|σ-‖u‖₂|≈1e-8，原始推力界满足）、
  Ad/Bd/Bd_g 幂零闭式（`zoh_matrices.m`）、forward_sim 精确 ZOH 离散（可选 RK4 对照）
- 详细验证记录见 [TECHNICAL_REPORT.md](mars_guidance/mars_paper/TECHNICAL_REPORT.md) §8；
  项目改动时间线见 [README.md](README.md) 提交记录

## 贯通总表

| 阶段（论文指南） | 论文位置 | 对应课程（OCRL） | 作业 / 笔记本 | 项目代码（对照） | 学习状态 |
|---|---|---|---|---|---|
| 阶段一 动力学建模 | §2.1-2.2，Eq.1-5 | Lecture 1-3（动力学、积分器、离散时间仿真） | HW0 primer、HW1 Q1、Lecture 2 integrators.ipynb | `mars_dynamics.m`、`forward_sim.m` | 🔄 |
| 阶段二 障碍约束 | §2.3-2.4，Eq.6-8、Problem 1 | Lecture 4-6（求根、极小化、约束问题） | HW1 Q2（牛顿法+线搜索） | `obstacle_constraint.m` | ⬜ |
| 阶段三 无损凸化 | §3.1，Eq.9-19 | Lecture 6-7（内点法、对偶、merit） | HW1 Q3（QP 内点法） | `solve_pd_socp.m` 变量区与 z_l/z_u 段 | ⬜ |
| 阶段四 离散化/Problem 4 | §3.2，Eq.20-28 | Lecture 8-10（LQR、DP、MPC）；复习 L2-3 | HW2（LQR/DP）、Lecture 10 mpc.ipynb | `solve_pd_socp.m` cvx 段 | ⬜ |
| 阶段五 Algorithm 1 | §3.3-3.4，Eq.29-36 | Lecture 11-14（iLQR/DDP、SQP、直接配点） | HW3（iLQR） | `solve_algorithm1.m` | ⬜ |
| 阶段六 Algorithm 2 | §4，Eq.37-42 | Lecture 10-12（目标函数/权重设计） | Lecture 12 笔记本 | `solve_algorithm2.m`、`trajectory_metrics.m` | ⬜ |
| 阶段七 对照论文实验 | §5，Tables 1-5 | 无新课程（L10 仿真验证思路） | — | `run_milestone1/2/3.m`、`results/` | ⬜ |
| 后续 MPC 化 | — | Lecture 10 | mpc.ipynb | 待写 | ⬜ |
| 后续 RL 对比 | — | Lecture 19-24（ILC、RL、"How to land"） | — | 待写 | ⬜ |
| 可选 6-DoF 闭环 | — | Lecture 15-17（四元数、姿态） | — | 待写 | ⬜ |

## 当前学习进度（2026-08-02）

- 论文：Abstract + Introduction 已读（7.26）
- 课程：Lecture 1-2 已上；HW0 primer 进行中（8.2）
- 对应状态：阶段一 🔄，其余 ⬜

## 协作约定

- OCRL 线程：按本表"对应课程"列推进，学完某阶段的课程后，回项目线程做该阶段的指南验证。
- Mars 项目线程：只做论文精读、MATLAB 验证与代码注释，不处理 Julia 作业。
- 每完成一个阶段：在 README.md 学习笔记记日期，并把本表状态改为 ✅。
