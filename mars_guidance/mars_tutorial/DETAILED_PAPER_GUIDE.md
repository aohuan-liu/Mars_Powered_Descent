# 论文 × 代码 学习指南（按阶段推进）

> 本指南**只针对论文本身**，不涉及课程内容。
> 论文：Gao et al., "Obstacle avoidance guidance for Mars powered descent
> using convex optimization and elevation angle",
> *Acta Astronautica* 248 (2026) 296-313，通讯作者郭延宁（guoyn@hit.edu.cn）
> 论文 PDF：`mars_guidance/mars_paper/paper.pdf`（页脚有期刊页码 296-313）
> 配套：`mars_paper/TECHNICAL_REPORT.md`（技术解析与验证结果）、
> `README.md`（项目说明与进度笔记）

## 怎么用这份指南

每一阶段固定五件事，按顺序推进，验收通过再进下一阶段：

1. **① 读论文哪里** —— 期刊页码 + 公式编号 + 图号
2. **② 对应公式** —— 把公式抄下来，翻译成人话
3. **③ 对应代码** —— 在哪个文件、哪一段
4. **④ 验证效果** —— 跑什么脚本、预期看到什么数字/图像
5. **⑤ 给代码注释** —— 在代码里亲手写注释，写不出来就是没懂

全程约 2~3 周（每天 1~2 小时）。需要的基础数学（不求精通，会用就行）：
凸集与凸函数、泰勒展开、零阶保持器离散化、二阶锥约束 ‖x‖ ≤ t。

---

## 0. 论文全局地图

| 章节 | 期刊页码 | 内容 | 对应代码 |
|---|---|---|---|
| 摘要 + 引言 | 296-297 | 问题与方法概述 | — |
| §2.1-2.2 | 297 | 坐标系、动力学 Eq.1-3、边界/推力约束 Eq.4-5 | `mars_params.m`、`dynamics/mars_dynamics.m`、`convex_opt/forward_sim.m` |
| §2.3-2.4 | 298 | 三种障碍约束 Eq.6-8、Problem 1（为什么难） | `obstacle/obstacle_constraint.m` |
| §3.1 | 298-299 | 无损凸化 Eq.9-19（z/u/σ 替换 + 泰勒展开） | `convex_opt/solve_pd_socp.m` 变量区、z_l/z_u 段 |
| §3.2 | 299-300 | ZOH 离散化 Eq.20-28、Problem 4 | `solve_pd_socp.m` 的 Ad/Bd/Bd_g 与 cvx 段 |
| §3.3-3.4 | 300-301 | 迭代细化 Eq.29-30、同伦避障 Eq.31-36、Algorithm 1 | `convex_opt/solve_algorithm1.m` |
| §4 | 301-303 | 仰角优化 Eq.37-42、Algorithm 2、α 选择 | `convex_opt/solve_algorithm2.m`、`trajectory_metrics.m` |
| §5 | 306-311 | 数值实验、Tables 1-5 | `run_milestone1/2/3.m`、`results/` |

## 1. 符号总表（读论文时随时回来查）

| 符号 | 含义 | 代码中的名字 |
|---|---|---|
| r = [rx, ry, rz] | 位置（x 向上、y 东、z 北） | `X(1:3,:)`、`r0/rf` |
| v = [vx, vy, vz] | 速度 | `X(4:6,:)`、`v0/vf` |
| m | 质量 | `X(7,:)`、`m_wet/m_dry` |
| T | 推力矢量 | `U_opt`×质量 |
| g_m | 火星重力 3.7114 m/s²（沿 -x） | `params.g_mars` |
| λ | 燃料消耗系数 = 1/(Isp·g0)，单位 s/m | `params.lambda` |
| Γ | 推力松弛上界 ‖T‖ ≤ Γ | （隐含在 σ 中） |
| z | ln(m)，对数质量 | `X(7,:)` |
| u | T/m，比推力 | `U_opt` |
| σ | Γ/m，比推力松弛 | `sigma_opt` |
| γ_gs | 常规下滑角 10° | `params.gamma_gs` |
| l1/l2/h | 障碍参数 100/500/500 m | `params.l1/l2/h_safe` |
| γ_new | 松弛锥角，tan=h/(l2-l1) | `params.gamma_new` |
| l/h（Eq.8） | 阶跃约束半径/高度 500/500 m | `params.l_step/h_step` |
| N / Δt | 离散点数 30 / 步长 tf/(N+1) | `params.N/dt` |
| t_f | 总飞行时间 80 s | `params.tf` |
| α | 仰角权重（默认 1） | `params.alpha_elev`、`opts.alpha` |
| W_ζ | 松弛罚权重 10³ | `params.W_zeta` |
| δ | 同伦系数（0→1） | `solve_algorithm1/2` 内循环变量 |
| ζ | 障碍松弛变量 | `zeta`（solve_pd_socp 内） |
| H / ε_ζ | 同伦次数 3 / 容差 10⁻⁶ | `params.H`、`eps_zeta` |
| m_fuel | Algorithm 2 固定油量 | `opts.m_fuel` |

---

## 阶段一：动力学建模（论文 §2.1-2.2，Eq.1-5）

### ① 读论文哪里
- 期刊第 297 页：图 1（坐标系）、§2.1（Eq.1-3）、§2.2（Eq.4-5）
- 期刊第 306 页：Table 1（着陆器参数表）

### ② 对应公式
- **Eq.1 动力学**：ṙ=v，v̇=g_m+T/m，ṁ=-λ‖T‖
  —— 位置变化=速度；速度变化=重力+推力/质量；质量下降=烧油。
- **Eq.2**：λ=1/(Isp·g0)（比冲换算成消耗系数）
- **Eq.3**：仰角 θ=arcsin(rx/‖r‖)（本阶段只需认识，阶段六再用）
- **Eq.4**：初末状态固定（r0、v0、m_wet；rf、vf、m≥m_dry）
- **Eq.5**：推力幅值约束 T_min ≤ ‖T‖ ≤ T_max（发动机不能关）

### ③ 对应代码
- `mars_params.m`：所有参数，对照 Table 1/2 逐行找
- `dynamics/mars_dynamics.m`：Eq.1 的右端函数（9 行）
- `convex_opt/forward_sim.m`：欧拉法积分（验证动力学用）

### ④ 验证效果
跑 `main`，看 Step 2 输出：初始高度 1500 m，恒推力（=初始重力）下最终
高度 -3779.4 m、速度 87.50 m/s。**这不是 bug**——一边烧油质量变轻，
同样的推力产生的加速度变大，所以加速下坠。再把 `mars_params.m` 里
`v0` 的 z 分量从 70 改成 -70，预测轨迹偏移方向后重新跑。

### ⑤ 给代码注释（在 `mars_dynamics.m` 里照抄一遍）
```matlab
function dx = mars_dynamics(t, x, T, g, lambda)
    r = x(1:3);   % 位置 [rx;ry;rz]，Eq.1 的 r
    v = x(4:6);   % 速度，Eq.1 的 v
    m = x(7);     % 质量，Eq.1 的 m
    g_vec = [-g; 0; 0];          % 重力沿 -x（Eq.1 的 g_m）
    dx = [v;                     % dr/dt = v        ← Eq.1 第一行
          g_vec + T / m;         % dv/dt = g + T/m   ← Eq.1 第二行
          -lambda * norm(T)];    % dm/dt = -λ‖T‖     ← Eq.1 第三行
end
```

### 验收
不看代码写出 Eq.1 并解释每条；能说出"为什么 T_min>0 会给后面找麻烦"
（最小推力下界让可行域变成非凸，阶段三解决它）。

---

## 阶段二：障碍约束（论文 §2.3-2.4，Eq.6-8 + Problem 1）

### ① 读论文哪里
- 期刊第 298 页：图 2（三种约束示意图）、§2.3（Eq.6/7/8）、§2.4（Problem 1）

### ② 对应公式
- **Eq.6 常规下滑角**：tan(γ_gs)·√(ry²+rz²) ≤ rx
  —— 必须待在一个 10° 的倒锥走廊里（凸的，能直接进求解器）。
- **Eq.7 松弛下滑角**：分段——水平距离 ≤l1 时 rx≥0；l1<d<l2 时
  rx≥(d-l1)·tanγ_new；d≥l2 时 rx≥h。离落点近时允许低飞。
- **Eq.8 阶跃约束**：d≤l 时 rx≥0，d≥l 时 rx≥h。
  —— "烟囱"：水平距离 >500 m 必须保持 500 m 高，进入 500 m 内才能下降。
- **Problem 1**：燃料最优问题（最小化 ṁ 的积分），约束里有 Eq.6 或 7 或 8。
  难点：Eq.7/8 是"台阶形"，可行域非凸，不能直接放进凸优化。

### ③ 对应代码
- `obstacle/obstacle_constraint.m`：三种约束的"这个位置至少要多高"
  （输入位置，输出 F 和裕度 rx-F）

### ④ 验证效果
1. 画三条约束曲线，亲眼看到"锥"和"台阶"的区别：
   ```matlab
   params = mars_params;
   d = linspace(0, 1500, 300);
   for k = 1:numel(d)
       F1(k) = obstacle_constraint([0;0;d(k)], params, 'glide', 1);
       F2(k) = obstacle_constraint([0;0;d(k)], params, 'relaxed', 1);
       F3(k) = obstacle_constraint([0;0;d(k)], params, 'stepwise', 1);
   end
   plot(d, F1, d, F2, d, F3, 'LineWidth', 1.5);
   legend('Eq.6','Eq.7','Eq.8'); xlabel('水平距离 (m)'); ylabel('最低高度 (m)');
   ```
2. 跑 `run_milestone1`：无约束最省油 **360.33 kg**，但违反 Eq.6 最多
   **-46.42 m**；加 Eq.6 后 **360.71 kg**，轨迹正好贴住锥面（裕度 0.00 m）。

### ⑤ 给代码注释（在 `obstacle_constraint.m` 的 switch 里补）
```matlab
case 'glide'    % Eq.6：锥面。F = tan(γ_gs)·d，随水平距离线性升高
    F = tan(params.gamma_gs_rad) * d_h;
case 'relaxed'  % Eq.7：先 0，再斜坡 (d-l1)·tanγ_new，最后平顶 h
    ...          % （对照 Box I 三分段补注释）
case 'stepwise' % Eq.8：台阶，d≤l 是 0，d>l 是 h
    ...
```

### 验收
能画出三条曲线并解释形状；能说出 Eq.6 凸、Eq.7/8 非凸的原因；
能复述 Problem 1 的两个非凸来源（T/m 非线性、T≥T_min 下界）。

**防坑**：论文 Table 3 的 505.49 kg 是 Fig.18(d)（tf≈138 s）场景的数字，
且超过 500 kg 燃料容量（m_wet-m_dry），是论文内部不一致
（详见 TECHNICAL_REPORT §8.1）。**不要在 tf=80 场景期待 505 kg。**

---

## 阶段三：无损凸化（论文 §3.1，Eq.9-19）

### ① 读论文哪里
- 期刊第 298-299 页：§3.1 全部（Eq.9-19、Problem 2/3）。
  跳过推导细节，抓住"三个替换 + 一个泰勒展开"。

### ② 对应公式
- **Eq.10-12 三个替换**：
  z=ln m（质量动力学线性化）、u=T/m（推力项线性化）、
  σ=Γ/m 且 ‖u‖≤σ（把"推力下界"非凸约束放宽成凸约束）。
- **Eq.15**：目标 = min ∫σ dt（等价于燃料最少）。
- **Eq.16-17**：σ ≥ T_min·e^(-z) 里的 e^(-z) 仍非线性，
  用泰勒展开近似（下界用二阶、上界用一阶）。
- **Eq.18**：两个参考轨迹 z_l（全程最小推力，质量最高）、
  z_u（全程最大推力，质量最低），作为泰勒展开的基准点。
- 为什么"无损"：最省油时 σ 必然压到 ‖u‖，放宽不改变最优值。

### ③ 对应代码
- `convex_opt/solve_pd_socp.m`：
  - `variable X/u/sig` 声明区（Eq.11 的替换）
  - `z_l`/`z_u` 计算段（Eq.18/27）
  - cvx 段 `norm(u(:,k)) <= sig(k)`（Eq.12）
  - cvx 段里 `Tmin*exp(-zl)*(1-dzl+0.5*dzl^2) <= sig(k)`（Eq.17 下界）

### ④ 验证效果
以读代码为主：打开 `solve_pd_socp.m`，把上面四处位置一一找到，
并在注释里写上对应的 Eq 编号。跑 `run_milestone1` 观察基准 360.33 kg。

### ⑤ 给代码注释（示例，抄到 cvx 段对应行）
```matlab
variable X(7, N+2)        % 状态：r/v/z，Eq.11 的 z=ln m 已在 X 第7行
variable u(3, N+1)        % 控制：u=T/m，Eq.11
variable sig(1, N+1)      % 松弛：σ=Γ/m，Eq.11
...
norm(u(:, k)) <= sig(k);  % ‖u‖≤σ，Eq.12 的凸松弛
Tmin*exp(-zl)*(1-dzl+0.5*dlz^2) <= sig(k);  % σ≥T_min·e^{-z} 的泰勒近似，Eq.17
sig(k) <= Tmax*exp(-zu)*(1-dzu);            % σ≤T_max·e^{-z} 的一阶近似，Eq.17
```

### 验收
能解释 z/u/σ 各自解决什么非线性；能说清"为什么松弛是无损的"；
能在代码里指出三个替换和泰勒展开的位置。

---

## 阶段四：离散化与 Problem 4（论文 §3.2，Eq.20-28）

### ① 读论文哪里
- 期刊第 299-300 页：§3.2 全部（Eq.20-28、Problem 4）。

### ② 对应公式
- **Eq.20-24**：连续系统离散化。取 N+1=31 个控制时刻（k=0..30）、
  N+2=32 个状态时刻；"零阶保持器"假设两次指令间推力不变；
  X(k+1)=Ad·X(k)+Bd·[u;σ]+Bd_g，其中 Ad=e^(Ac·Δt)、Bd 为积分项。
- **Eq.25**：初末状态 + 质量下限的离散形式。
- **Eq.26-27**：推力上下界 + 常规下滑角约束的离散形式。
- **Eq.28（Problem 4）**：最终要求解的问题——min λΔtΣσ。

### ③ 对应代码（全在 `convex_opt/solve_pd_socp.m`）
- `Ad = expm(A_c*dt)`、`Bd`/`Bd_g` 数值积分段（Eq.23-24）
- `cvx_begin quiet` ~ `cvx_end`：完整的 Problem 4

cvx 段结构（背下来，这是面试级能力）：
```
变量：X(7×32), u(3×31), sig(1×31), zeta(1×31，障碍用)
目标：min sum(sig)*dt            ← Eq.15/28 燃料最少
约束：
  1. X(:,k+1)==Ad*X(:,k)+Bd*[u;sig]+Bd_g   ← Eq.23 动力学
  2. 初末状态固定                        ← Eq.25
  3. norm(u)<=sig                       ← Eq.12
  4. 泰勒化推力上下界                     ← Eq.26
  5. X(7,:)>=log(m_dry)                 ← Eq.25 质量下限
  6. （可选）Eq.6 下滑角                 ← run_milestone1 开启
```

### ④ 验证效果
跑 `run_milestone1`，把输出的燃料数字与 cvx 段约束一一对应。
改 `params.N`（如 60）重跑，观察燃料和耗时变化，体会"离散化越细越慢"。

### ⑤ 给代码注释
把上面的 cvx 段结构图抄成注释，插到 `solve_pd_socp.m` 的
`cvx_begin` 前后，每一行约束都标注对应 Eq 编号。

### 验收
能逐行讲 cvx_begin~cvx_end，每行都能说出对应论文哪个式子。

---

## 阶段五：同伦避障 Algorithm 1（论文 §3.3-3.4，Eq.29-36）

### ① 读论文哪里
- 期刊第 300-301 页：§3.3（Eq.29-30）、§3.4（Eq.31-36）、
  Box I（Eq.32 的分段定义）、Algorithm 1 完整步骤。

### ② 对应公式
- **Eq.29-30 迭代细化**：泰勒展开的参考点从 Eq.18 的"假想轨迹"
  换成"上一次求出的解" z̃。参考点越接近真解，近似误差越小。
- **Eq.31-33 统一约束**：三种约束统一写成 rx ≥ F(X,δ)；
  同伦系数 δ 把整个约束"按比例缩小"（δ=0 无障碍，δ=1 完整障碍）。
- **Eq.34 松弛**：rx ≥ F + ζ，允许暂时违反。
- **Eq.35-36（Problem 5）**：目标 min λΔtΣσ + W_ζ·‖ζ‖₂，
  罚项逼 ζ 收敛到 0。
- **Algorithm 1**：δ=min(j,H)/H（H=3，共 4 次迭代）；
  收敛条件 ‖ζ‖₂ ≤ 10⁻⁶ 且 j≥H+1。

### ③ 对应代码
- `convex_opt/solve_algorithm1.m`：Algorithm 1 的循环
- `solve_pd_socp.m` 的 `taylor_ref='iterate'` 分支（Eq.30）和
  `X(1,k) >= F_ref(k) + zeta(k)`（Eq.34）

### ④ 验证效果
跑 `run_milestone2`，看输出：
- 松弛约束场景：燃料 **365.29 kg**（论文 365.18）、**409.47 kg**
  （论文 409.12），4 次迭代收敛，‖ζ‖₂ 逐次降到 1e-15 量级；
- 阶跃约束场景：380.16 kg（论文未给数值，图 Fig.19 只看收敛行为）。
对照论文 Fig.19-21，解释"为什么第 3 次迭代燃料跳变明显"。

### ⑤ 给代码注释（在 `solve_algorithm1.m` 的循环里补）
```matlab
delta = min(j, H) / H;      % 同伦系数：1/3 → 2/3 → 1 → 1
F_ref(k) = obstacle_constraint(ref_X(:,k), params, ctype, delta);
                            % 在"上一次轨迹"上算障碍高度 F（Eq.32/33）
% spec.taylor_ref = 'iterate'  → 用上一次的 z 做泰勒展开（Eq.30）
% slack=true                   → 加 ζ 变量与 W_ζ·‖ζ‖₂ 罚项（Eq.34-35）
```

### 验收
能解释"为什么不能把 Eq.7/8 直接塞进 SOCP"、"δ 和 ζ 各起什么作用"、
"为什么要 4 次迭代"。

---

## 阶段六：仰角优化 Algorithm 2（论文 §4，Eq.37-42）

### ① 读论文哪里
- 期刊第 301-303 页：§4.1（Eq.37-40、Problem 6/7、Algorithm 2）、
  §4.2（α 的选择分析，Fig.6-8）。

### ② 对应公式
- **Eq.37-38 新目标**：min -α·Σrx + Σ|ry| + Σ|rz|
  —— 不再求最省油，而是"尽量高、水平距离尽量小"；
  α 是"多想要高度"的旋钮。
- **Eq.40（Problem 7）**：目标再加 W_ζ·‖ζ‖₂，
  m_dry = m_wet - m_fuel（油量固定）。
- **Eq.42 仰角积分**：I=Σθ_k·(-v_x,k)·Δt，θ=arcsin(rx/‖r‖)。
  这是评价"全程视野好不好"的指标；乘 -v_x 表示只有下降才贡献。
- **§4.2 结论**：α 太小轨迹贴障碍、α 太大末段反而贴地；
  α∈[0.5,2] 合适，默认 1。

### ③ 对应代码
- `convex_opt/solve_algorithm2.m`：Algorithm 2 的循环
- `solve_pd_socp.m` 的 `objective='elevation'` 分支（Eq.38/40）
- `convex_opt/trajectory_metrics.m`：仰角积分 I（Eq.42）

### ④ 验证效果
跑 `run_milestone3`，看输出：
- α 扫描（m_fuel=400 kg，阶跃约束）：仰角积分在 **α=1.5 最高（1285.7）**，
  符合论文"α∈[0.5,2] 合适"；
- 固定燃料扫描：仰角积分随 tf 单峰（365.18 kg 峰值在 tf=70，
  409.12 kg 峰值在 tf=80），对应论文 Fig.25/29。

### ⑤ 给代码注释（在 `trajectory_metrics.m` 里补）
```matlab
theta = asin(rk(1) / norm(rk));   % 仰角 θ = arcsin(rx/‖r‖)，Eq.3
I = I + theta * (-v(1,k)) * dt;   % 仰角积分，Eq.42：只有下降(-vx>0)才累加
```

### 验收
能解释目标函数三项各是什么；能解释 α 的双面性（为什么太大反而差）；
能解释仰角积分为什么乘 -v_x。

---

## 阶段七：对照论文实验（论文 §5）

### ① 读论文哪里
- 期刊第 306-311 页：§5.1（Tables 1-2）、§5.2（Fig.18-21、Table 3）、
  §5.3（Fig.22-30）、§5.4-5.5（Fig.31-34、Tables 4-5）。

### ② 对应公式
无新公式。把 §5 每个数字和前面阶段一~六的公式/约束对应起来。

### ③ 对应代码
- `run_milestone1/2/3.m` + `results/`（.mat 数据与 PNG 图）

### ④ 验证效果（实测 vs 论文对照表）

| 场景 | 论文 | 本项目实测 | 状态 |
|---|---|---|---|
| 无约束燃料（tf=80） | 未明确给出 | 360.33 kg | 复现 |
| Eq.6 约束（tf=80） | Table 3 的 505.49 是 Fig.18d 场景 | 360.71 kg | 见技术报告 8.1 |
| Algorithm 1 松弛 tf=80 | 365.18 kg | 365.29 kg | ✅ 偏差 0.03% |
| Algorithm 1 松弛 tf=100 | 409.12 kg | 409.47 kg | ✅ 偏差 0.09% |
| Algorithm 1 阶跃 tf=80 | 未给数值（Fig.19） | 380.16 kg | 定性一致 |
| Algorithm 2 α 扫描 | 峰值在 [0.5,2] | 峰值 α=1.5 | ✅ 定性一致 |
| Algorithm 2 固定燃料扫描 | 仰角积分单峰 | tf=70/80 单峰 | ✅ 定性一致 |

### ⑤ 给代码注释
无需新注释。任务是把上表**背下来**，能说出每个数字的实验条件
（哪个场景、哪个 tf、哪个约束）。

### 验收
能对着论文 §5 的每个图，说出"这是什么场景、用了哪个约束、跑了哪个脚本"。

---

## 术语表（随时回来查）

| 术语 | 一句话解释 |
|---|---|
| 凸优化 | 目标与可行域都是"碗形"的问题，保证全局最优 |
| 二阶锥约束 | 形如 ‖x‖ ≤ t，图形是锥，凸优化的标准零件 |
| SOCP | 二阶锥规划，本论文最终让 CVX 解的问题类型 |
| 无损凸化 | 放宽非凸问题但最优值不变（省油时必然取等） |
| 泰勒展开 | 用多项式近似函数；论文用它近似 e^(-z) |
| ZOH | 零阶保持器：两次指令之间推力保持上次值 |
| SCP | 序列凸优化：解→更新参考点→再解，逐步逼近 |
| 同伦 | 从简单版本连续变形到困难版本，逐步逼近 |
| 松弛变量 | 允许暂时违反约束的缓冲量，被罚项逼到 0 |
| 可行域 | 所有满足约束的（燃料,时间）组合 |
| 仰角 | 着陆器看向目标的视线与地面夹角，越大视野越好 |
| bang-bang | 最优推力只在 T_min/T_max 间切换的曲线 |

## 常见坑 FAQ

**Q1：论文说 505.49 kg，我跑出来 360.71 kg，错了吗？**
没有。505.49 是 Fig.18(d)（tf≈138 s）的数字，且超过 500 kg 燃料容量，
是论文内部不一致（技术报告 §8.1 有证据）。

**Q2：main 跑出最终高度 -3779 m，动力学错了吗？**
没有。Step 2 故意用"恒推力=初始重力"验证，质量变轻后加速度变大，
加速下坠是正确行为。优化轨迹在 Step 3。

**Q3：z_l 和 z_u 谁是高质量轨迹？**
z_l=ln(m_wet-λT_min·t)（最小推力→质量最高）；
z_u=ln(m_wet-λT_max·t)（最大推力→质量最低）。早期代码写反过，已修正。

**Q4：σ 和 ‖u‖ 什么关系？**
约束是 ‖u‖≤σ；最省油时恰好 σ=‖u‖。平时可把 σ 当"推力大小/质量"理解。

**Q5：为什么 tf≥140 s 单次求解不可行？**
一是燃料需求超过 500 kg 容量（物理限制）；二是 Taylor 误差随 tf 累积
（这正是 Algorithm 1 迭代细化要解决的）。

**Q6：CVX 报 "Inaccurate/Solved" 是什么？**
有解但数值精度不完美，常出现在可行域边界（如 Algorithm 2 的
tf=100、m_fuel=409.12）。本项目视为不可行。

---

## 学完之后，下一步

1. **MPC 化**：把求解器改成滚动时域闭环（每个控制周期重新求解）。
2. **NN 加速（方法论修正后）**：`nn_accel/` 现有代码用开环轨迹训练，
   不是反馈策略；正确做法是 MPC 闭环滚动采样数据。
3. **6-DoF Simulink 验证**（可选）：把 3-DoF 轨迹接到姿态环。
