%% ZOH DISCRETIZATION MATRICES (Paper Eq.23-24), exact closed form
%  A_c is nilpotent with A_c^2 = 0 (position->velocity coupling only), so
%    e^{A_c*dt} = I + dt*A_c                 (exactly)
%    Bd         = dt*B_c + (dt^2/2)*A_c*B_c
%    Bd_g       = dt*g_vec + (dt^2/2)*A_c*g_vec
%  This is the same exact zero-order-hold discretization used by the solver,
%  in closed form (no numerical quadrature needed).
%
%  OUTPUT: Ad (7x7), Bd (7x4), Bd_g (7x1) for
%    X(k+1) = Ad*X(k) + Bd*[u(k); sigma(k)] + Bd_g,  X = [r; v; ln m]

function [Ad, Bd, Bd_g] = zoh_matrices(params)
    dt  = params.dt;
    g   = params.g_mars;
    lam = params.lambda;

    A_c = [zeros(3,3), eye(3), zeros(3,1);
           zeros(3,3), zeros(3,3), zeros(3,1);
           zeros(1,6), 0];
    B_c = [zeros(3,3), zeros(3,1);
           eye(3),    zeros(3,1);
           zeros(1,3), -lam];
    g_vec = [0; 0; 0; -g; 0; 0; 0];

    Ad   = eye(7) + dt * A_c;                       % expm(A_c*dt), exact
    Bd   = dt * B_c + (dt^2 / 2) * (A_c * B_c);
    Bd_g = dt * g_vec + (dt^2 / 2) * (A_c * g_vec);
end
