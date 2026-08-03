%% FUEL-OPTIMAL CONVEX GUIDANCE (Paper Problem 4)
%  Wrapper around solve_pd_socp.m that reproduces the Phase-1 baseline
%  (Problem 4 without obstacle) and optionally adds the conventional
%  glide-slope constraint Eq.(6).
%
%  INPUT
%    params : struct from mars_params.m
%    opts   : optional struct
%      constraint : 'none' (default, Problem 4) | 'glide' (Eq.6)
%      gamma_gs   : glide-slope angle [deg] (default params.gamma_gs)
%      tf         : override terminal time [s]
%
%  OUTPUT
%    X_opt     - optimal state [7 x N+2], X = [r; v; ln m]
%    U_opt     - optimal specific thrust [3 x N+1], u = T/m
%    sigma_opt - slack variable [1 x N+1], sigma = Gamma/m
%    exit_flag - solver status string ('Solved'/'Infeasible'/...)
%    info      - full solution struct from solve_pd_socp

function [X_opt, U_opt, sigma_opt, exit_flag, info] = solve_fuel_optimal(params, opts)
    if nargin < 2
        opts = struct();
    end
    if isfield(opts, 'tf')
        params.tf = opts.tf;
        params.dt = params.tf / (params.N + 1);
    end

    ctype   = getfield_default(opts, 'constraint', 'none');
    gamma_gs = getfield_default(opts, 'gamma_gs', params.gamma_gs);

    spec = struct('objective', 'fuel', ...
                  'constraint', ctype, ...
                  'gamma_gs', gamma_gs, ...
                  'taylor_ref', 'bounds', ...
                  'slack', false, ...
                  'W_zeta', 1e3, ...
                  'm_dry', params.m_dry);
    sol = solve_pd_socp(params, spec);

    X_opt     = sol.X;
    U_opt     = sol.u;
    sigma_opt = sol.sigma;
    exit_flag = sol.status;
    info      = sol;

    if strcmp(sol.status, 'Solved')
        fprintf('CVX status: %s\n', sol.status);
        fprintf('Fuel used: %.2f kg\n', sol.fuel);
        fprintf('Position error: %.4f m\n', sol.pos_err);
        fprintf('Velocity error: %.4f m/s\n', sol.vel_err);
        fprintf('Slack tightness max|sigma-||u|||: %.3e\n', sol.tightness);
        fprintf('Raw thrust m*sigma: [%.2f, %.2f] N (paper: [%.2f, %.2f])\n', ...
                sol.thrust_min, sol.thrust_max, params.T_min, params.T_max);
        if strcmp(ctype, 'glide')
            fprintf('Glide-slope min margin: %.2f m\n', sol.min_margin);
        end
    else
        fprintf('CVX status: %s\n', sol.status);
        warning('Solver did not converge. Check parameters or constraints.');
    end
end

function v = getfield_default(s, name, default)
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
