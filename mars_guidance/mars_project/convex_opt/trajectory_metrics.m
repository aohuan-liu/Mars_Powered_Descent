%% TRAJECTORY METRICS (paper targets vs measured)
%  Computes the key metrics used to compare against the paper:
%    - fuel used [kg]
%    - minimum obstacle-constraint margin [m] (negative = violation)
%    - elevation-angle integral I (Eq.42)
%    - elevation objective value (Eq.38, alpha-free part)
%
%  INPUT
%    X      : state matrix [7 x N+2] = [r; v; ln m]
%    params : struct from mars_params.m
%    type   : 'glide' | 'relaxed' | 'stepwise' | 'none'
%    u      : (optional) specific thrust [3 x N+1]
%    sigma  : (optional) slack [1 x N+1]; enables the slack-tightness and
%             raw-thrust-bound post-check (lossless relaxation evidence)

function m = trajectory_metrics(X, params, type, u, sigma)
    if nargin < 3
        type = 'none';
    end
    N = params.N;
    dt = params.dt;
    r = X(1:3, :);
    v = X(4:6, :);
    mass = exp(X(7, :));

    m.fuel = params.m_wet - mass(end);
    m.final_mass = mass(end);

    if strcmp(type, 'none')
        m.min_margin = inf;
    else
        marg = zeros(1, N+2);
        for k = 1:N+2
            [~, marg(k)] = obstacle_constraint(r(:, k), params, type, 1);
        end
        m.min_margin = min(marg);
        m.margins = marg;
    end

    % Elevation-angle integral, Eq.(42): I = sum_k theta_k * (-vx_k) * dt
    I = 0;
    for k = 2:N+1
        rk = r(:, k);
        theta = asin(rk(1) / norm(rk));
        I = I + theta * (-v(1, k)) * dt;
    end
    m.elev_integral = I;

    % Objective value, Eq.(38): -sum(rx) + sum(|ry|) + sum(|rz|), k=1..N
    m.obj_value = -sum(r(1, 2:N+1)) + sum(abs(r(2, 2:N+1))) + sum(abs(r(3, 2:N+1)));

    % Slack-tightness post-check (paper Eq.5/12: sigma ~= ||u|| and
    % T_min <= m*sigma <= T_max)
    if nargin >= 5 && ~isempty(u) && ~isempty(sigma)
        m_ctrl = exp(X(7, 1:N+1));          % mass at control times t_0..t_N
        thrust = m_ctrl .* sigma;           % raw thrust magnitude m*sigma
        m.slack_tightness = max(abs(sigma - vecnorm(u)));
        m.thrust_min = min(thrust);
        m.thrust_max = max(thrust);
    else
        m.slack_tightness = nan;
        m.thrust_min = nan;
        m.thrust_max = nan;
    end
end
