 function params = mars_params
 % Return struct with all simulation parameters from paper Tables 1-2
 
 % Physics
 params.g_mars = 3.7114;           % Mars gravity [m/s^2]
 params.g0     = 9.81;             % Earth gravity [m/s^2]
 params.Isp    = 200.3;         % Calculated from lambda = 1/(Isp*g0)              % Specific impulse [s]
 params.lambda = 5.09e-4;          % Fuel consumption coefficient [1/m]
 
 % Lander
 params.m_wet = 1905;              % Wet mass (w/ fuel) [kg]
 params.m_dry = 1405;              % Dry mass [kg]
 
 % Thrust
 params.T_max = 13258.17;           % Max thrust [N]
 params.T_min = 4971.82;            % Min thrust [N]
 
 % Initial conditions (Eq.4)
 params.r0 = [1500; 0; 1500];       % Initial position [m] x:(up), y:(east), z:(north)
 params.v0 = [-75; 0; 70];          % Initial velocity [m/s]
 
 % Terminal conditions
 params.rf = [0; 0; 0];             % Target position
 params.vf = [0; 0; 0];             % Target velocity
 
 % Glide slope & obstacle parameters
 params.gamma_gs    = 10;           % Glide slope angle [deg]
 params.gamma_gs_rad = deg2rad(params.gamma_gs);
 params.l1   = 100;                  % Safety radius 1 [m]
 params.l2   = 500;                  % Safety radius 2 [m]
 params.h_safe = 500;                % Safety altitude [m]
 params.l_step = 500;                % Stepwise safety radius [m]
 params.h_step = 500;                % Stepwise safety altitude [m]
 
 % Solver
 params.N  = 30;                     % Discretization points
 params.tf = 80;                     % Total flight time [s]
 params.dt = params.tf / (params.N + 1);             % Time step [s]
 
 % Elevation angle weight
 params.alpha_elev = 1;
 
 % Homotopy iteration
 params.H        = 10;
 params.tol_zeta = 1e-4;
 end

