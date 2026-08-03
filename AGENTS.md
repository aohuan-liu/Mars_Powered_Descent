When working in this repository on MATLAB or Simulink tasks:

1. Treat the user's requirement as an engineering objective with measurable acceptance criteria.
2. If criteria are ambiguous, propose concrete metrics and assumptions before major model changes.
3. Prefer Simulink MCP tools for model inspection and editing:
   - `model_overview`
   - `model_read`
   - `model_edit`
   - `model_query_params`
   - `model_resolve_params`
   - `model_test` when behavioral tests are available
4. Use an iterative loop for closed-loop design tasks:
   - translate the requirement into target metrics
   - inspect the current model structure and parameters
   - modify the model or parameter set
   - run simulation or tests
   - compare results against the target metrics
   - continue until the metrics are met or a clear blocker is found
5. After each iteration, summarize:
   - what changed
   - what metric was measured
   - whether the design moved closer to or farther from the target
6. Do not stop at "model created" if the user explicitly asked for tuning or performance targets; continue iterating until the target is met or blocked.
7. Prefer the configured auto-start MATLAB MCP flow first; Codex should attempt to launch MATLAB through the configured `simulink` MCP server before asking the user to start MATLAB manually.
8. Only fall back to manual MATLAB initialization if MCP startup fails repeatedly. In that case, instruct the user to run:
   - `addpath('matlab_mcp\simulink-agentic-toolkit'); satk_initialize`

## 文档职责（总则）

1. **README.md 与 ROADMAP.md 由总审查会话维护**：论文 / OCRL 线程完成改动后，由总审查会话在 README「提交记录」追加时间戳条目（项目修改 + 项目现状），并同步 ROADMAP「项目代码基线」。
2. README「学习笔记」只记录用户本人的学习进展，由用户手写，任何会话不得代写。
3. ROADMAP「学习状态」列 = 用户自己的学习状态（⬜/🔄/✅），与代码实现状态分开，不得混用。
4. TECHNICAL_REPORT.md 与 DETAILED_PAPER_GUIDE.md 归论文线程维护；总审查会话只审查一致性，不直接编辑（除非用户明确要求）。
