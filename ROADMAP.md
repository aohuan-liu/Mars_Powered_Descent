# 学习路线：课程 × 论文 × 项目（贯通主线）

本文档是 **OCRL 课程线程** 与 **Mars 项目线程** 的共享锚点。
两个线程各自推进，进度统一记录在 README.md 的学习笔记里。

## 项目背景

- 论文：Gao et al., "Obstacle avoidance guidance for Mars powered descent using convex optimization and elevation angle", *Acta Astronautica* 248 (2026) 296-313，通讯作者郭延宁（guoyn@hit.edu.cn）
- 项目：`mars_guidance/mars_project/`（无损凸化 + CVX/SOCP，Phase 1 可运行）
- 课程：CMU 16-745 Optimal Control and Reinforcement Learning（OCRL），资料在 `OCRL/`
- 基础：本科自动控制原理 + 现代控制理论，最优控制学习中

## 当前审查结论（Mars 项目线）

| 检查项 | 状态 |
|---|---|
| 无损凸化 + ZOH 离散化（Problem 2-4 主体） | ✅ 已实现，实测燃料 360.34 kg |
| 论文 Eq.6 常规下滑角约束 | ❌ 缺失，实测轨迹违规 -47.33 m |
| Eq.7/8 松弛/阶跃避障约束 | ⚠️ 仅独立检查函数，未进求解器 |
| Algorithm 1 同伦迭代避障 | ❌ 未实现 |
| Section 4 / Algorithm 2 仰角优化 | ❌ 未实现 |
| Section 5 论文场景验证（505.49 / 365.18 / 409.12 kg） | ❌ 未复现 |
| NN 加速拓展 | ⚠️ 方法论需修正（开环轨迹≠反馈策略） |

## 贯通里程碑

| 课程节点 | 论文读到哪 | 项目动作 |
|---|---|---|
| [ ] Lecture 1-3（动力学/积分器）+ HW0 | Section 2.1-2.2（Eq.1-5） | 读懂 `forward_sim.m` 欧拉法；跑 Step 2 悬停试验；可选升级 RK4 |
| [ ] Lecture 4-5（求根/极小化/牛顿法）+ HW1 Q1-Q2 | Section 2.3-2.4（Eq.6-8、Problem 1 非凸性） | 独立跑通 `obstacle_constraint.m`，画锥形走廊 vs 轨迹 |
| [ ] Lecture 6-7（内点法/对偶/merit）+ HW1 Q3 | Section 3.1（无损凸化 Eq.10-15） | 逐行读懂 `solve_fuel_optimal.m` 的 cvx 段，能讲清变量与约束规模 |
| [ ] Lecture 8-10（LQR/DP/MPC）| Section 3.2-3.3（ZOH Eq.20-24、Problem 4） | 补 Eq.6 下滑角约束对照 ~505 kg 基准；求解器 MPC 化（制导闭环） |
| [ ] Lecture 11-14（iLQR/DDP/SQP/配点）+ HW3 | Section 3.4（Algorithm 1 同伦） | 实现 Algorithm 1 + Problem 5，复现 365.18 / 409.12 kg，画 ζ 收敛图 |
| [ ] Lecture 15-18（姿态/混合动力/ILC）| 可选 | 可选：6-DoF 闭环验证（Simulink/MCP） |
| [ ] Lecture 19-24（ILC/RL）| 回读 Introduction 学习型方法批判 | NN 加速（MPC 滚动采样闭环数据）+ PPO/SAC 对比 |
| [ ] 穿插 | Section 4（仰角 Eq.37-40、Algorithm 2） | 实现 Problem 6/7 + α 扫描，复现仰角积分曲线 |

## 协作约定

- OCRL 线程：只做课程学习、Julia 作业、概念答疑，不改 MATLAB 项目代码。
- Mars 项目线程：只做论文精读、MATLAB 实现与验证，不处理 Julia 作业。
- 每完成一个里程碑，在 README.md 学习笔记追加一条日期记录（格式沿用 7.26 / 8.2）。
