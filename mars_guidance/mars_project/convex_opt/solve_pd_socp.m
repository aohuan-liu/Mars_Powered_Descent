%% CORE SOCP SOLVER (Paper Problems 4/5/6/7)
%  Gao et al., "Obstacle avoidance guidance for Mars powered descent using
%  convex optimization and elevation angle", Acta Astronautica 248 (2026)
%  296-313.
%
%  Builds and solves the discrete-time convex (SOCP) problems:
%    Problem 4: fuel-optimal, optional conventional glide-slope (Eq.6)
%    Problem 5: fuel-optimal + homotopy obstacle constraint + slack
%               (Eq.30, Eq.32/33, Eq.34)
%    Problem 6: elevation-angle objective, basic constraints
%    Problem 7: elevation-angle objective + obstacle constraint + slack
%
%  INPUT
%    params : struct from mars_params.m
%    spec   : problem specification (struct)
%      objective  : 'fuel' (default) | 'elevation'
%      constraint : 'none' (default) | 'glide' | 'relaxed' | 'stepwise'
%      gamma_gs   : glide-slope angle [deg] for constraint='glide'
%      delta      : homotopy parameter in [0,1] (default 1)
%      ref_X      : [7 x N+2] reference trajectory for evaluating F (Eq.32/33)
%      taylor_ref : 'bounds' (Eq.26/27, default) | 'iterate' (Eq.30, uses ref_z)
%      ref_z      : [1 x N+2] reference log-mass for taylor_ref='iterate'
%      slack      : logical, include slack variable zeta (Eq.34) in the
%                   obstacle constraint and the objective (Problem 5/7)
%      W_zeta     : slack penalty weight (default 1e3, Table 2)
%      alpha      : elevation weight for objective='elevation' (default 1)
%      m_dry      : terminal mass floor [kg] (default params.m_dry)
%
%  OUTPUT
%    out : struct with fields
%      X, u, sigma, zeta, status, fuel, pos_err, vel_err, mass,
%      min_margin (obstacle margin, 'none' -> +inf), elev_integral (Eq.42),
%      obj_value (objective as written in the paper)

function out = solve_pd_socp(params, spec)
    if nargin < 2
        spec = struct();
    end

    % ------------------------------------------------------------------
    % Unpack parameters
    % ------------------------------------------------------------------
    N    = params.N;
    dt   = params.dt;
    g    = params.g_mars;       % scalar magnitude (acts in -x direction)
    lam  = params.lambda;       % fuel consumption coefficient [s/m]
    Tmin = params.T_min;
    Tmax = params.T_max;
    m_wet = params.m_wet;
    m_dry = params.m_dry;
    if isfield(spec, 'm_dry') && ~isempty(spec.m_dry)
        m_dry = spec.m_dry;
    end
    r0 = params.r0; v0 = params.v0;
    rf = params.rf; vf = params.vf;
    z0 = log(m_wet);

    objective = getfield_default(spec, 'objective', 'fuel');
    ctype     = getfield_default(spec, 'constraint', 'none');
    gamma_gs  = getfield_default(spec, 'gamma_gs', params.gamma_gs);
    delta     = getfield_default(spec, 'delta', 1);
    taylor    = getfield_default(spec, 'taylor_ref', 'bounds');
    use_slack = getfield_default(spec, 'slack', false);
    W_zeta    = getfield_default(spec, 'W_zeta', 1e3);
    alpha     = getfield_default(spec, 'alpha', 1);

    % ------------------------------------------------------------------
    % ZOH discretization (Eq.21-24), exact closed form (A_c^2 = 0)
    % ------------------------------------------------------------------
    [Ad, Bd, Bd_g] = zoh_matrices(params);

    % ------------------------------------------------------------------
    % Reference log-mass bounds (Eq.27): z_l = min-thrust path (highest
    % mass), z_u = max-thrust path (lowest mass).  [Paper convention,
    % Eq.(18).]
    % ------------------------------------------------------------------
    z_l = zeros(1, N+2);
    z_u = zeros(1, N+2);
    for k = 0:N+1
        tk = k * dt;
        z_l(k+1) = log(max(m_wet - lam * Tmin * tk, 1));  % high-mass path
        z_u(k+1) = log(max(m_wet - lam * Tmax * tk, 1));  % low-mass path
    end

    % ------------------------------------------------------------------
    % Evaluate the obstacle constraint RHS on the reference trajectory
    % (constant w.r.t. the optimization variables).
    % ------------------------------------------------------------------
    F_ref = [];
    if strcmp(ctype, 'relaxed') || strcmp(ctype, 'stepwise')
        if ~isfield(spec, 'ref_X') || isempty(spec.ref_X)
            error('spec.ref_X (reference trajectory) required for constraint %s', ctype);
        end
        F_ref = zeros(1, N+1);
        for k = 1:N+1
            F_ref(k) = obstacle_constraint(spec.ref_X(:, k), params, ctype, delta);
        end
    end

    % ------------------------------------------------------------------
    % CVX model
    % ------------------------------------------------------------------
    cvx_begin quiet
        variable X(7, N+2)          % state: [r; v; z], k = 0..N+1
        variable u(3, N+1)          % specific thrust u = T/m, k = 0..N
        variable sig(1, N+1)        % slack sigma = Gamma/m, k = 0..N
        variable zeta(1, N+1)       % obstacle slack (Eq.34), unused if slack=false

        if strcmp(objective, 'elevation')
            % Eq.(38/40): k = 1..N  -> state columns 2..N+1
            kk = 2:N+1;
            obj = -alpha * sum(X(1, kk)) + sum(abs(X(2, kk))) + sum(abs(X(3, kk)));
        else
            % Problem 4/5: min lambda*dt*sum(sigma); lambda constant dropped
            obj = sum(sig) * dt;
        end
        % Slack penalty (Eq.35/40). When slack is not used the variable is
        % unconstrained but penalized, so the optimum sets zeta = 0.
        obj = obj + W_zeta * norm(zeta, 2);
        minimize(obj)

        % Dynamics (Eq.23)
        for k = 1:N+1
            X(:, k+1) == Ad * X(:, k) + Bd * [u(:, k); sig(k)] + Bd_g;
        end
        % Initial and terminal conditions (Eq.25)
        X(1:3, 1) == r0;
        X(4:6, 1) == v0;
        X(7, 1)   == z0;
        X(1:3, N+2) == rf;
        X(4:6, N+2) == vf;

        % Thrust constraints (Eq.12, Eq.26 or Eq.30)
        for k = 1:N+1
            norm(u(:, k)) <= sig(k);              % ||u|| <= sigma
            if strcmp(taylor, 'iterate')
                % Eq.(30): expand around the previous iterate z_tilde
                zr = spec.ref_z(k);
                dz = X(7, k) - zr;
                Tmin * exp(-zr) * (1 - dz + 0.5 * dz^2) <= sig(k);
                sig(k) <= Tmax * exp(-zr) * (1 - dz);
            else
                % Eq.(26): expand around the min/max-thrust paths (Eq.27)
                zl = z_l(k);
                dzl = X(7, k) - zl;
                Tmin * exp(-zl) * (1 - dzl + 0.5 * dzl^2) <= sig(k);
                zu = z_u(k);
                dzu = X(7, k) - zu;
                sig(k) <= Tmax * exp(-zu) * (1 - dzu);
            end
        end

        % Mass floor (Eq.25): z >= ln(m_dry)
        for k = 1:N+2
            X(7, k) >= log(m_dry);
        end

        % Obstacle constraints
        if strcmp(ctype, 'glide')
            % Eq.(6): tan(gamma_gs) * sqrt(ry^2+rz^2) <= rx, k = 0..N
            tgs = tan(deg2rad(gamma_gs));
            for k = 1:N+1
                tgs * norm(X(2:3, k)) <= X(1, k);
            end
        elseif strcmp(ctype, 'relaxed') || strcmp(ctype, 'stepwise')
            % Eq.(34): rx >= F(ref, delta) + zeta
            for k = 1:N+1
                if use_slack
                    X(1, k) >= F_ref(k) + zeta(k);
                else
                    X(1, k) >= F_ref(k);
                end
            end
        end
    cvx_end

    % ------------------------------------------------------------------
    % Post-processing
    % ------------------------------------------------------------------
    out.X = X;
    out.u = u;
    out.sigma = sig;
    out.zeta = zeta;
    out.status = cvx_status;
    out.dt = dt;

    if strcmp(cvx_status, 'Solved')
        out.mass = exp(X(7, :));
        out.fuel = m_wet - out.mass(end);
        out.pos_err = norm(X(1:3, end) - rf);
        out.vel_err = norm(X(4:6, end) - vf);

        % Slack-tightness post-check (lossless relaxation evidence):
        %   tightness = max|sigma - ||u||_2|  (should be ~0 for fuel-optimal)
        %   raw thrust = m * sigma at control points (paper Eq.5: T_min <= T <= T_max)
        m_ctrl = exp(X(7, 1:N+1));            % mass at control times t_0..t_N
        thrust = m_ctrl .* sig;               % m*sigma ~ actual thrust magnitude
        out.tightness = max(abs(sig - vecnorm(u)));
        out.thrust_min = min(thrust);
        out.thrust_max = max(thrust);

        % Obstacle margin of the obtained trajectory (full constraint, delta=1)
        if strcmp(ctype, 'none')
            out.min_margin = inf;
        else
            marg = zeros(1, N+2);
            for k = 1:N+2
                [~, marg(k)] = obstacle_constraint(X(1:3, k), params, ctype, 1);
            end
            out.min_margin = min(marg);
        end

        % Elevation integral (Eq.42) and objective value (Eq.38)
        I = 0;
        for k = 2:N+1
            rk = X(1:3, k);
            theta = asin(rk(1) / norm(rk));
            I = I + theta * (-X(4, k)) * dt;
        end
        out.elev_integral = I;
        kk = 2:N+1;
        out.obj_value = -alpha * sum(X(1, kk)) + sum(abs(X(2, kk))) + sum(abs(X(3, kk)));
        if use_slack
            out.obj_value = out.obj_value + W_zeta * norm(zeta, 2);
        end
    else
        out.mass = []; out.fuel = nan; out.pos_err = nan; out.vel_err = nan;
        out.min_margin = nan; out.elev_integral = nan; out.obj_value = nan;
        out.tightness = nan; out.thrust_min = nan; out.thrust_max = nan;
    end
end

function v = getfield_default(s, name, default)
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
