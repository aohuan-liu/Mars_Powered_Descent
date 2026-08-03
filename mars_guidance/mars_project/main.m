 %% MAIN: Mars powered descent convex optimization guidance
 % Project workflow:
 %   Step 1: Load simulation parameters from paper Tables 1-2
 %   Step 2: Verify forward dynamics (constant thrust test)
 %   Step 3: Fuel-optimal convex optimization (core solver)
 %   Step 4: Visualize results (6 plots)
 %
 % Paper: Gao et al. "Obstacle avoidance guidance for Mars powered descent
 %        using convex optimization and elevation angle"
 %        Acta Astronautica 248 (2026) 296-313
 % Corresponding author: Yanning Guo (guoyn@hit.edu.cn)
 
 clear; clc; close all;
 addpath(genpath(pwd));
 
 fprintf('========================================\n');
 fprintf('Mars Powered Descent - Convex Guidance\n');
 fprintf('Acta Astronautica, 2026\n');
 fprintf('========================================\n\n');
 
 %%
 % =========================================================================
 % STEP 1: Load parameters
 % =========================================================================
 % The mars_params() function returns a struct containing all physical
 % and numerical parameters from the paper's Tables 1 and 2.
 % This includes: Mars gravity, lander mass, thrust limits, initial/final
 % conditions, discretization parameters, obstacle geometry, etc.
 params = mars_params;
 
 %%
 % =========================================================================
 % STEP 2: Forward simulation verification
 % =========================================================================
 % Purpose: Validate that the dynamics model (Eq.1) is implemented correctly.
 % We apply a constant thrust equal to the spacecraft weight at initial mass.
 % If the dynamics are correct, this thrust should approximately hover the
 % lander (small velocity change, altitude stays roughly constant).
 %
 % In the real scenario, initial velocity is [-75,0,70] m/s, so the lander
 % will accelerate downward because mass decreases (thrust becomes > weight)
 % and initial velocity is negative. This is expected behavior.
 %
 % forward_sim uses the same exact ZOH discretization as the solver
 % (paper Eq.23-24), so an optimized trajectory can be replayed exactly;
 % use forward_sim(T_seq, params, 'rk4') for a continuous RK4 cross-check.
 
 fprintf('\n--- Step 2: Forward Simulation (Constant Thrust) ---\n');
 
 % Constant thrust = initial weight (m_wet * g_mars), pointing upward (+x)
 % This should roughly cancel gravity at the initial moment
 T_const = [params.m_wet * params.g_mars; 0; 0];
 T_seq = repmat(T_const, 1, params.N);
 
 [t_sim, x_sim] = forward_sim(T_seq, params);
 
 fprintf('Initial altitude: %.1f m\n', x_sim(1, 1));
 fprintf('Final altitude: %.1f m\n', x_sim(1, end));
 fprintf('Final velocity: %.2f m/s\n', norm(x_sim(4:6, end)));
 
 figure('Name', 'Forward Simulation');
 subplot(1,2,1);
 plot(t_sim, x_sim(1,:), 'b-', 'LineWidth', 2);
 xlabel('Time (s)'); ylabel('Altitude (m)');
 title('Altitude (Constant Thrust)'); grid on;
 
 subplot(1,2,2);
 v_norm = sqrt(sum(x_sim(4:6,:).^2, 1));
 plot(t_sim, v_norm, 'r-', 'LineWidth', 2);
 xlabel('Time (s)'); ylabel('Velocity (m/s)');
 title('Velocity (Constant Thrust)'); grid on;
 
 %%
 % =========================================================================
 % STEP 3: Fuel-optimal convex optimization (Paper Problem 2/3/4)
 % =========================================================================
 % This calls the core CVX-based SOCP solver. The solver:
 %   1. Discretizes the continuous dynamics using ZOH (Eq.20-23)
 %   2. Applies lossless convexification (Eq.10-12)
 %   3. Formulates a Second-Order Cone Program (Problem 4)
 %   4. Solves using SDPT3 or SeDuMi (CVX default solvers)
 %
 % The result is a fuel-optimal trajectory from the initial entry condition
 % to the target landing site, satisfying all physical constraints.
 
 fprintf('\n--- Step 3: Fuel-Optimal Convex Optimization ---\n');
 
 [X_opt, U_opt, sigma_opt, exit_flag] = solve_fuel_optimal(params);
 
 %%
 % =========================================================================
 % STEP 4: Visualization
 % =========================================================================
 if strcmp(exit_flag, 'Solved')
     fprintf('\n--- Step 4: Plotting Results ---\n');
     plot_results(X_opt, U_opt, params);
     fprintf('Project Phase 1 Complete!\n');
 else
     fprintf('\nOptimization did not converge.\n');
     fprintf('Common issues:\n');
     fprintf('  1. Is CVX installed and on the path?\n');
     fprintf('  2. Try increasing N (discretization points)\n');
     fprintf('  3. Check if tf (total time) is sufficient\n');
 end
 
 fprintf('\n========================================\n');
 fprintf('Next steps:\n');
 fprintf('  1. Already implemented - run run_milestone2 (Algorithm 1),\n');
 fprintf('     run_milestone3 (Algorithm 2) to verify paper numbers\n');
 fprintf('  2. MPC closed-loop guidance (rolling horizon)\n');
 fprintf('  3. NN acceleration (after methodology fix, see nn_accel/README)\n');
 fprintf('  4. RL comparison (PPO/SAC baseline)\n');
 fprintf('========================================\n');
