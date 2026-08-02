 %% FUEL-OPTIMAL CONVEX GUIDANCE (Paper: Problem 2/3/4)
 % =========================================================================
 % Implements the lossless convexification + SOCP formulation for Mars
 % powered descent guidance. This is the core solver of the project.
 %
 % Paper reference: Gao et al. "Obstacle avoidance guidance for Mars powered
 % descent using convex optimization and elevation angle"
 % Acta Astronautica 248 (2026) 296-313, Eqs. 10-30
 %
 % Key idea: The original fuel-optimal landing problem is non-convex (thrust
 % lower bound, mass division in dynamics). Through variable substitution and
 % Taylor expansion, we transform it into a Second-Order Cone Program (SOCP)
 % that CVX can solve to global optimality.
 %
 % INPUT:  params - struct with all simulation parameters (from mars_params.m)
 % OUTPUT: X_opt  - optimal state trajectory [7 x N+2], X = [r; v; ln(m)]
 %         U_opt  - optimal specific thrust [3 x N+1], u = T/m
 %         sigma_opt - slack variable [1 x N+1], sigma = Gamma/m
 %         exit_flag - solver status string (e.g. 'Solved', 'Infeasible')
 
 function [X_opt, U_opt, sigma_opt, exit_flag] = solve_fuel_optimal(params)
     
     % =====================================================================
     % STEP 1: Unpack parameters
     % =====================================================================
     N = params.N;
     dt = params.dt;
     g = params.g_mars;        % Mars gravity (positive, acts in -x direction)
     lam = params.lambda;      % Fuel consumption coefficient: 1/(Isp*g0) [1/m]
     T_min = params.T_min;     % Minimum thrust (engines cannot be turned off)
     T_max = params.T_max;     % Maximum thrust
     m_wet = params.m_wet;     % Initial wet mass (lander + fuel)
     m_dry = params.m_dry;     % Dry mass (no fuel)
     
     r0 = params.r0;
     v0 = params.v0;
     rf = params.rf;
     vf = params.vf;
     
     % =====================================================================
     % STEP 2: Lossless convexification - variable substitution (Paper Eq.11)
     % =====================================================================
     % The non-convexities in Problem 1 come from:
     %   (a) Thrust divided by mass: T(t)/m(t) is nonlinear
     %   (b) Thrust lower bound: ||T|| >= T_min is non-convex
     %
     % Solution: substitute variables to remove these nonlinearities:
     %   z(t) = ln(m(t))          -> log-mass, dynamics become linear in z
     %   u(t) = T(t)/m(t)         -> specific thrust (acceleration from engine)
     %   sigma(t) = Gamma(t)/m(t) -> slack variable for thrust bound relaxation
     %   Gamma(t) >= ||T(t)||     -> slack variable (lossless relaxation)
     %
     % After substitution, the mass dynamics z_dot = -lambda*sigma is LINEAR.
     % The constraint ||u|| <= sigma is a convex SECOND-ORDER CONE constraint.
     
     z0 = log(m_wet);           % Initial ln(mass)
     z_min = log(m_dry);        % Minimum ln(mass) constraint boundary
     
     % =====================================================================
     % STEP 3: ZOH discretization (Paper Eqs. 20-23)
     % =====================================================================
     % The continuous system is discretized using Zero-Order Hold (ZOH):
     %   X = [r_x; r_y; r_z; v_x; v_y; v_z; z]  (7x1 state vector)
     %   Control = [u_x; u_y; u_z; sigma]         (4x1 control vector)
     %
     % Continuous dynamics (Eq.12 rewritten as state-space):
     %   dX/dt = A_c * X + B_c * [u; sigma] + g_effect
     %
     % A_c captures the kinematic coupling (r depends on v, v on u):
     %   [r_dot]   [0  I  0] [r]   [0  0] [u]   [  0  ]
     %   [v_dot] = [0  0  0] [v] + [I  0] [s] + [ g_m ]
     %   [z_dot]   [0  0  0] [z]   [0 -L] [s]   [  0  ]
     %                                              where L = lambda
     
     A_c = [zeros(3,3), eye(3), zeros(3,1);
            zeros(3,3), zeros(3,3), zeros(3,1);
            zeros(1,6), 0];
     
     B_c = [zeros(3,3), zeros(3,1);
            eye(3), zeros(3,1);
            zeros(1,3), -lam];
     
     % Exact discretization using matrix exponential (Eq.23):
     %   X_{k+1} = expm(A_c*dt) * X_k + Bd * [u_k; sigma_k] + Bd_g
     %
     % Ad = expm(A_c * dt)  (zero-input response, state transition matrix)
     % Bd = integral_0^dt expm(A_c*(dt-tau)) * B_c dtau  (ZOH control input)
     % Bd_g = integral_0^dt expm(A_c*(dt-tau)) * g_vec dtau  (gravity effect)
     
     Ad = expm(A_c * dt);
     
     % Numerical integration using midpoint rule for Bd and Bd_g
     n_quad = 10;
     tau_vals = linspace(0, dt, n_quad);
     wts = dt / n_quad * ones(1, n_quad);
     Bd = zeros(7, 4);
     for i = 1:n_quad
         tau = tau_vals(i);
         phi = expm(A_c * (dt - tau));  % State transition from time tau to dt
         Bd = Bd + wts(i) * phi * B_c;
     end
     
     % Gravity acts only on the velocity equation (in -x direction)
     g_vec = [0; 0; 0; -g; 0; 0; 0];
     Bd_g = zeros(7, 1);
     for i = 1:n_quad
         tau = tau_vals(i);
         phi = expm(A_c * (dt - tau));
         Bd_g = Bd_g + wts(i) * phi * g_vec;
     end
     
     % =====================================================================
     % STEP 4: Taylor expansion for thrust bounds (Paper Eqs. 16-18)
     % =====================================================================
     % The convexified thrust constraints involve exp(-z), which is nonlinear.
     % We linearize around reference trajectories z_l(t) and z_u(t) using
     % first-order Taylor expansion (Eq.17):
     %   exp(-z) ~= exp(-z_ref) * (1 - (z - z_ref) + 0.5*(z - z_ref)^2)
     %
     % The reference trajectories are the min/max possible ln(mass) assuming
     % the spacecraft uses max or min thrust throughout (Eq.18):
     %   z_l(t) = ln(m_wet - lambda * T_max * t)  [lower bound, max fuel burn]
     %   z_u(t) = ln(m_wet - lambda * T_min * t)  [upper bound, min fuel burn]
     
     z_l = zeros(N+2, 1);
     z_u = zeros(N+2, 1);
     for k = 0:N+1
         tk = k * dt;
         % Lower bound: mass when burning at MAX thrust (fastest fuel consumption)
         val_l = m_wet - lam * T_max * tk;
         if val_l > 0
             z_l(k+1) = log(val_l);
         else
             z_l(k+1) = log(m_dry * 1.05);  % Clamp to slightly above dry mass
         end
         % Upper bound: mass when burning at MIN thrust (slowest consumption)
         val_u = m_wet - lam * T_min * tk;
         if val_u > 0
             z_u(k+1) = log(val_u);
         else
             z_u(k+1) = log(m_wet);
         end
     end
     
     % =====================================================================
     % STEP 5: Solve CVX optimization (Paper Problem 4 / Eq. 23-30)
     % =====================================================================
     % This is the core SOCP. CVX automatically handles the second-order cone
     % and linear constraints using SDPT3 or SeDuMi.
     %
     % VARIABLES:
     %   X(:,k) = [r_k; v_k; z_k]  - state at time step k (N+2 total points)
     %   u(:,k) = T_k/m_k          - specific thrust (control)
     %   sig(k) = Gamma_k/m_k      - slack for thrust relaxation
     %
     % OBJECTIVE: minimize total fuel consumption = integral of sigma dt
     %   (min fuel = max final mass = max z(tf) = min integral sigma dt)
     %
     % CONSTRAINTS:
     %   1. Discrete dynamics: X_{k+1} = Ad*X_k + Bd*[u_k; sig_k] + Bd_g
     %   2. Initial state fixed at entry conditions
     %   3. Terminal state = soft landing (position=0, velocity=0)
     %   4. Norm(u) <= sigma (relaxed thrust upper bound, SOC constraint)
     %   5. Taylor-expanded thrust lower/upper bounds
     %   6. Mass >= dry mass (as linear constraint: z >= ln(m_dry))
     
     cvx_begin quiet
     
         % ---- Variables ----
         variable X(7, N+2)          % Full state at each time step
         variable u(3, N+1)          % Specific thrust (acceleration from engine)
         variable sig(1, N+1)        % Slack variable for thrust bound relaxation
         
         % ---- Objective: minimize fuel consumption (Eq.15) ----
         % Equivalent to: minimize integral of sigma(t)*Lam*dt
         % Since sigma = Gamma/m and mass consumption = Lam*Gamma,
         % minimizing integral sigma minimizes total fuel used.
         obj = 0;
         for k = 1:N+1
             obj = obj + sig(k) * dt;  % Riemann sum approximation
         end
         minimize(obj)
         
         % ---- Constraints ----
         
         % 1. Dynamics constraints (Eq.23): X_{k+1} must equal the discrete
         %    propagation from X_k under control [u_k; sig_k].
         for k = 1:N+1
             X(:, k+1) == Ad * X(:, k) + Bd * [u(:, k); sig(k)] + Bd_g;
         end
         
         % 2. Initial conditions (Eq.4): must match entry interface conditions
         X(1:3, 1) == r0;
         X(4:6, 1) == v0;
         X(7, 1) == z0;
         
         % 3. Terminal conditions: soft landing at target site
         X(1:3, N+2) == rf;    % Exactly at target position
         X(4:6, N+2) == vf;    % Zero velocity at touchdown
         
         % 4. Thrust constraints (convexified, Eq.12, 16-17)
         for k = 1:N+1
             % SOC constraint: ||u||_2 <= sigma
             % This is the convex relaxation of ||T|| <= Gamma (Eq.12)
             % Physical meaning: the actual engine thrust cannot exceed
             % the slack variable. This relaxation is LOSSLESS for fuel-optimal
             % problems (i.e., the optimal solution always has ||T|| = Gamma).
             norm(u(:, k)) <= sig(k);
             
             % Thrust LOWER bound (Eq.16, Taylor expanded Eq.17):
             % T_min * exp(-z_k) <= sig_k
             % Physical: engines have non-zero minimum thrust (can't turn off)
             z_k = X(7, k);
             z_kl = z_l(k);
             dz = z_k - z_kl;
             T_min * exp(-z_kl) * (1 - dz + 0.5 * dz^2) <= sig(k);
             
             % Thrust UPPER bound (Eq.16, linearized):
             % sig_k <= T_max * exp(-z_k)
             % Physical: engines have maximum thrust limit
             z_ku = z_u(k);
             dz_u = z_k - z_ku;
             sig(k) <= T_max * exp(-z_ku) * (1 - dz_u);
         end
         
         % 5. Mass constraint: cannot burn more fuel than carried
         %    z >= ln(m_dry)  =>  exp(z) >= m_dry  =>  m >= m_dry
         for k = 1:N+2
             X(7, k) >= log(m_dry);
         end
         
     cvx_end
     
     % =====================================================================
     % STEP 6: Extract and display results
     % =====================================================================
     exit_flag = cvx_status;
     X_opt = X;
     U_opt = u;
     sigma_opt = sig;
     
     fprintf('CVX status: %s\n', cvx_status);
     if strcmp(cvx_status, 'Solved')
         % Convert back to physical quantities
         m_opt = exp(X_opt(7, :));    % Mass from ln(m)
         T_opt = zeros(3, N+1);
         for k = 1:N+1
             T_opt(:, k) = u(:, k) * m_opt(k);  % Thrust from specific thrust * mass
         end
         fuel_used = m_wet - m_opt(end);
         pos_err = norm(X_opt(1:3, end) - rf);
         vel_err = norm(X_opt(4:6, end) - vf);
         fprintf('Fuel used: %.2f kg\n', fuel_used);
         fprintf('Position error: %.4f m\n', pos_err);
         fprintf('Velocity error: %.4f m/s\n', vel_err);
     else
         warning('Solver did not converge. Check parameters or constraints.');
     end
 end
