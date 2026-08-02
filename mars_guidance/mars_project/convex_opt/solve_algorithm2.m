%% ALGORITHM 2: ELEVATION-ANGLE-BASED CONVEX OPTIMIZATION
%  Paper: Gao et al., Acta Astronautica 248 (2026) 296-313, Section 4.
%
%  Procedure (Algorithm 2):
%    1. (Feasible region from Algorithm 1 is assumed known; the caller picks
%        m_fuel and tf inside it.)
%    2. Solve Problem 6 with gamma_gs = 0 -> initial reference trajectory
%    3. Loop j = 1..H+1 with delta = min(j,H)/H:
%         - evaluate F(X_ref, delta) (Eq.32 or 33)
%         - solve Problem 7 with Taylor expansion around z_ref (Eq.30),
%           obstacle constraint rx >= F + zeta (Eq.34) and elevation objective
%           min -alpha*sum(rx) + sum(|ry|) + sum(|rz|) + W_zeta*||zeta||_2
%           (Eq.40).  m_dry = m_wet - m_fuel (fixed fuel budget).
%         - stop when all constraints hold, ||zeta||_2 <= eps_zeta, j >= H+1
%    4. Return the last iterate as the elevation-optimal trajectory
%
%  INPUT
%    params : struct from mars_params.m
%    opts   : struct
%      m_fuel     : fixed fuel consumption [kg] (required)
%      constraint : 'relaxed' (default) | 'stepwise'
%      alpha      : elevation weight (default 1, Table 2)
%      H          : homotopy iteration count (default 3)
%      eps_zeta   : slack tolerance (default 1e-6)
%      W_zeta     : slack penalty weight (default 1e3)
%      tf         : override terminal time [s]
%      r0, v0     : override initial position/velocity
%      verbose    : print per-iteration log (default true)
%
%  OUTPUT
%    out : struct with X, u, sigma, zeta, fuel, status, min_margin,
%          elev_integral (Eq.42), obj_value (Eq.38/40), hist

function out = solve_algorithm2(params, opts)
    if nargin < 2 || ~isfield(opts, 'm_fuel')
        error('solve_algorithm2 requires opts.m_fuel');
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

    m_fuel  = opts.m_fuel;
    alpha   = getfield_default(opts, 'alpha', 1);
    ctype   = getfield_default(opts, 'constraint', 'relaxed');
    H       = getfield_default(opts, 'H', 3);
    eps_zeta = getfield_default(opts, 'eps_zeta', 1e-6);
    W_zeta  = getfield_default(opts, 'W_zeta', 1e3);
    verbose = getfield_default(opts, 'verbose', true);

    m_dry = params.m_wet - m_fuel;
    N = params.N;

    if verbose
        fprintf('\n=== Algorithm 2 (%s constraint, m_fuel=%.1f kg, alpha=%.2f, H=%d) ===\n', ...
                ctype, m_fuel, alpha, H);
    end

    % --------------------------------------------------------------
    % Initial trajectory: Problem 6 with gamma_gs = 0 (no constraint)
    % --------------------------------------------------------------
    spec0 = struct('objective', 'elevation', 'constraint', 'none', ...
                   'taylor_ref', 'bounds', 'slack', false, ...
                   'alpha', alpha, 'W_zeta', W_zeta, 'm_dry', m_dry);
    sol0 = solve_pd_socp(params, spec0);
    if ~strcmp(sol0.status, 'Solved')
        warning('Algorithm 2: initial Problem 6 did not solve (%s)', sol0.status);
        out = sol0;
        out.hist = struct('iter', {}, 'delta', {}, 'slack_norm', {}, ...
                          'fuel', {}, 'obj', {}, 'elev', {}, 'status', {});
        out.iterations = 0;
        out.converged = false;
        out.delta_final = nan;
        out.slack_norm_final = nan;
        return;
    end
    ref_X = sol0.X;
    ref_z = sol0.X(7, :);

    if verbose
        fprintf('Init (gamma=0): fuel = %.2f kg, elev integral = %.2f\n', ...
                sol0.fuel, sol0.elev_integral);
    end

    % --------------------------------------------------------------
    % Homotopy loop
    % --------------------------------------------------------------
    hist = struct('iter', {}, 'delta', {}, 'slack_norm', {}, ...
                  'fuel', {}, 'obj', {}, 'elev', {}, 'status', {});
    sol = sol0;
    for j = 1:H+1
        delta = min(j, H) / H;

        F_ref = zeros(1, N+1);
        for k = 1:N+1
            F_ref(k) = obstacle_constraint(ref_X(:, k), params, ctype, delta);
        end

        sol_prev = sol;
        spec = struct('objective', 'elevation', 'constraint', ctype, ...
                      'taylor_ref', 'iterate', 'slack', true, ...
                      'alpha', alpha, 'W_zeta', W_zeta, 'delta', delta, ...
                      'ref_X', ref_X, 'ref_z', ref_z, 'm_dry', m_dry);
        sol = solve_pd_socp(params, spec);

        slack_norm = norm(sol.zeta, 2);
        hist(j).iter = j;
        hist(j).delta = delta;
        hist(j).slack_norm = slack_norm;
        hist(j).fuel = sol.fuel;
        hist(j).obj = sol.obj_value;
        hist(j).elev = sol.elev_integral;
        hist(j).status = sol.status;
        if verbose
            fprintf('  j=%d  delta=%.3f  ||zeta||_2=%.3e  fuel=%.2f kg  obj=%.1f  elev=%.2f  %s\n', ...
                    j, delta, slack_norm, sol.fuel, sol.obj_value, sol.elev_integral, sol.status);
        end

        if strcmp(sol.status, 'Solved')
            marg = zeros(1, N+2);
            for k = 1:N+2
                [~, marg(k)] = obstacle_constraint(sol.X(1:3, k), params, ctype, 1);
            end
            all_ok = min(marg) >= -1e-3;
            % Paper Algorithm 2 step 13: terminate only when j >= H+1
            % (delta reaches 1 at j=H; one extra refinement pass at j=H+1).
            % Every round is checked, but early termination before the full
            % constraint has been solved twice is intentionally not allowed.
            if all_ok && slack_norm <= eps_zeta && j >= H+1
                break;
            end
            ref_X = sol.X;
            ref_z = sol.X(7, :);
        else
            warning('Algorithm 2: subproblem infeasible at j=%d (delta=%.3f)', j, delta);
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
        fprintf('Algorithm 2 done after %d iterations, fuel = %.2f kg, elev integral = %.2f\n', ...
                out.iterations, sol.fuel, sol.elev_integral);
    end
end

function v = getfield_default(s, name, default)
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
