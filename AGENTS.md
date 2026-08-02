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
   - `addpath('C:\Users\17978\Documents\New project\simulink-agentic-toolkit'); satk_initialize`
