%% FORWARD SIMULATION: exact ZOH propagation (consistent with the solver)
%  Given a thrust sequence T_seq, integrates the lander dynamics (Eq.1)
%  using the SAME zero-order-hold discretization as the CVX solver
%  (Eq.23-24):  X(k+1) = Ad*X(k) + Bd*[u(k); sigma(k)] + Bd_g, with
%  u = T/m and sigma = ||T||/m. For piecewise-constant thrust this is the
%  EXACT solution of the continuous ODE, so forward_sim reproduces the
%  solver's discrete states up to machine precision.
%
%  Alternatively, method='rk4' integrates the continuous ODE with a
%  classical 4th-order Runge-Kutta step per interval as a cross-check.
%
%  INPUT:
%    T_seq  - thrust sequence [3 x N], column k applied over [t_k, t_{k+1}]
%    params - struct from mars_params.m
%    method - 'zoh' (default) | 'rk4'
%  OUTPUT:
%    t_hist - time vector [(N+1) x 1]
%    x_hist - state history [7 x (N+1)], x = [r; v; m] with mass in kg

function [t_hist, x_hist] = forward_sim(T_seq, params, method)
    if nargin < 3
        method = 'zoh';
    end
    g   = params.g_mars;
    lam = params.lambda;
    dt  = params.dt;
    N   = params.N;

    t_hist = (0:N)' * dt;
    x_hist = zeros(7, N+1);
    x_hist(:, 1) = [params.r0; params.v0; params.m_wet];

    switch method
        case 'zoh'
            % Same exact ZOH discretization as solve_pd_socp (Eq.23-24)
            [Ad, Bd, Bd_g] = zoh_matrices(params);
            X = [params.r0; params.v0; log(params.m_wet)];  % [r; v; z]
            for k = 1:N
                m_k   = exp(X(7));
                u_k   = T_seq(:, k) / m_k;          % specific thrust u = T/m
                sig_k = norm(T_seq(:, k)) / m_k;    % sigma = ||T||/m
                X = Ad * X + Bd * [u_k; sig_k] + Bd_g;
                x_hist(:, k+1) = [X(1:6); exp(X(7))];  % output mass in kg
            end
        case 'rk4'
            % Classical RK4 on the continuous ODE (Eq.1), one step per interval
            for k = 1:N
                xk = x_hist(:, k);
                Tk = T_seq(:, k);
                dx = @(xx) [xx(4:6); [-g; 0; 0] + Tk / xx(7); -lam * norm(Tk)];
                k1 = dx(xk);
                k2 = dx(xk + dt / 2 * k1);
                k3 = dx(xk + dt / 2 * k2);
                k4 = dx(xk + dt * k3);
                x_hist(:, k+1) = xk + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
            end
        otherwise
            error('Unknown integrator: %s (use ''zoh'' or ''rk4'')', method);
    end
end
