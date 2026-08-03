%% ALGORITHM 1: IMPROVED LOSSLESS CONVEXIFICATION + HOMOTOPY OBSTACLE
%  Paper: Gao et al., Acta Astronautica 248 (2026) 296-313, Section 3.3-3.4.
%
%  Procedure (Algorithm 1):
%    1. Solve Problem 4 with gamma_gs = 0 -> initial reference trajectory
%    2. Loop j = 1..H+1 with delta = min(j,H)/H:
%         - evaluate F(X_ref, delta) (Eq.32 or 33, Box I)
%         - solve Problem 5 with Taylor expansion around z_ref (Eq.30) and
%           obstacle constraint rx >= F + zeta (Eq.34), objective
%           min lambda*dt*sum(sigma) + W_zeta*||zeta||_2  (Eq.35/36)
%         - stop when all constraints hold, ||zeta||_2 <= eps_zeta, j >= H+1
%    3. Return the last iterate as the obstacle-avoiding fuel-optimal solution
%
%  INPUT
%    params : struct from mars_params.m
%    opts   : optional struct
%      constraint : 'relaxed' (default) | 'stepwise' | 'glide'
%                   ('glide' uses the same iterative Taylor refinement with
%                    the convex Eq.6 cone; no homotopy/slack is needed)
%      H          : homotopy iteration count (default 3, paper Section 5.1)
%      eps_zeta   : slack tolerance (default 1e-6, paper Section 5.1)
%      W_zeta     : slack penalty weight (default 1e3, Table 2)
%      tf         : override terminal time [s]
%      r0, v0     : override initial position/velocity
%      verbose    : print per-iteration log (default true)
%
%  OUTPUT
%    out : struct
%      X, u, sigma, zeta, fuel, status, min_margin, elev_integral
%      hist : per-iteration table (delta, slack_norm, fuel, status)

function out = solve_algorithm1(params, opts)
    if nargin < 2
        opts = struct();
    end
    if isfield(opts, 'tf')
        params.tf = opts.tf;
        params.dt = params.tf / (params.N + 1);
    end
    if isfield(opts, 'r0')
        params.r0 = opts.r0;
    end
    if isfield(opts, 'v0')
        params.v0 = opts.v0;
    end

    ctype    = getfield_default(opts, 'constraint', 'relaxed');
    H        = getfield_default(opts, 'H', 3);
    eps_zeta = getfield_default(opts, 'eps_zeta', 1e-6);
    W_zeta   = getfield_default(opts, 'W_zeta', 1e3);
    verbose  = getfield_default(opts, 'verbose', true);

    N = params.N;

    if verbose
        fprintf('\n=== Algorithm 1 (%s constraint, H=%d) ===\n', ctype, H);
    end

    % --------------------------------------------------------------
    % Initial trajectory: Problem 4 with gamma_gs = 0 (no constraint)
    % --------------------------------------------------------------
    spec0 = struct('objective', 'fuel', 'constraint', 'none', ...
                   'taylor_ref', 'bounds', 'slack', false, ...
                   'W_zeta', W_zeta, 'm_dry', params.m_dry);
    sol0 = solve_pd_socp(params, spec0);
    if ~strcmp(sol0.status, 'Solved')
        warning('Algorithm 1: initial Problem 4 did not solve (%s)', sol0.status);
        out = sol0;
        out.hist = struct('iter', {}, 'delta', {}, 'slack_norm', {}, ...
                          'fuel', {}, 'status', {});
        out.iterations = 0;
        out.converged = false;
        out.delta_final = nan;
        out.slack_norm_final = nan;
        return;
    end
    ref_X = sol0.X;
    ref_z = sol0.X(7, :);

    if verbose
        fprintf('Init (gamma=0): fuel = %.2f kg, slack norm = inf\n', sol0.fuel);
    end

    % --------------------------------------------------------------
    % Homotopy loop
    % --------------------------------------------------------------
    hist = struct('iter', {}, 'delta', {}, 'slack_norm', {}, ...
                  'fuel', {}, 'status', {});
    sol = sol0;
    for j = 1:H+1
        delta = min(j, H) / H;

        % F(X_ref, delta) on the reference trajectory (Eq.32/33);
        % not needed for the convex cone 'glide' constraint.
        F_ref = zeros(1, N+1);
        if strcmp(ctype, 'relaxed') || strcmp(ctype, 'stepwise')
            for k = 1:N+1
                F_ref(k) = obstacle_constraint(ref_X(:, k), params, ctype, delta);
            end
        end

        sol_prev = sol;
        spec = struct('objective', 'fuel', 'constraint', ctype, ...
                      'taylor_ref', 'iterate', 'slack', true, ...
                      'W_zeta', W_zeta, 'delta', delta, ...
                      'ref_X', ref_X, 'ref_z', ref_z, ...
                      'm_dry', params.m_dry);
        sol = solve_pd_socp(params, spec);

        slack_norm = norm(sol.zeta, 2);
        hist(j).iter = j;
        hist(j).delta = delta;
        hist(j).slack_norm = slack_norm;
        hist(j).fuel = sol.fuel;
        hist(j).status = sol.status;
        if verbose
            fprintf('  j=%d  delta=%.3f  ||zeta||_2=%.3e  fuel=%.2f kg  %s\n', ...
                    j, delta, slack_norm, sol.fuel, sol.status);
        end

        if strcmp(sol.status, 'Solved')
            % Constraint check with the FULL (delta=1) obstacle constraint
            marg = zeros(1, N+2);
            for k = 1:N+2
                [~, marg(k)] = obstacle_constraint(sol.X(1:3, k), params, ctype, 1);
            end
            all_ok = min(marg) >= -1e-3;
            % Paper Algorithm 1 step 12: convergence is checked every round;
            % termination additionally requires j >= H+1, i.e. the full
            % constraint (delta=1, reached at j=H) is solved at least twice
            % with the updated reference. A solution that meets the criteria
            % earlier is still refined once more, per the paper's schedule.
            if all_ok && slack_norm <= eps_zeta && j >= H+1
                break;
            end
            % Update reference for the next iteration
            ref_X = sol.X;
            ref_z = sol.X(7, :);
        else
            warning('Algorithm 1: subproblem infeasible at j=%d (delta=%.3f)', j, delta);
            sol = sol_prev;   % keep the last feasible iterate
            break;
        end
    end

    out = sol;
    out.hist = hist;
    out.iterations = numel(hist);
    out.delta_final = hist(end).delta;
    out.slack_norm_final = norm(sol.zeta, 2);
    % Honest convergence flag: re-validate the RETURNED solution against the
    % full (delta=1) constraint, so a retained early iterate that already
    % converged is not misreported if a later subproblem failed.
    if strcmp(out.status, 'Solved')
        marg = zeros(1, N+2);
        for k = 1:N+2
            [~, marg(k)] = obstacle_constraint(out.X(1:3, k), params, ctype, 1);
        end
        out.converged = min(marg) >= -1e-3 && out.slack_norm_final <= eps_zeta;
    else
        out.converged = false;
    end

    if verbose
        fprintf('Algorithm 1 done after %d iterations, fuel = %.2f kg\n', ...
                out.iterations, sol.fuel);
        fprintf('  tightness max|sigma-||u||| = %.3e, raw thrust m*sigma = [%.2f, %.2f] N\n', ...
                out.tightness, out.thrust_min, out.thrust_max);
    end
end

function v = getfield_default(s, name, default)
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
