% 模型保存到本脚本所在目录（matlab_mcp/work/），避免硬编码绝对路径
projectRoot = fileparts(mfilename('fullpath'));
model = 'satellite_orbit_sim';
modelPath = fullfile(projectRoot, [model '.slx']);

% Default scenario: 2D Earth-centered two-body orbit in km and km/s.
mu = 398600.4418;           % Earth gravitational parameter [km^3/s^2]
earthRadius = 6378.137;     % Earth radius [km]
altitude = 500;             % Default circular-orbit altitude [km]
r0 = earthRadius + altitude;
v0 = sqrt(mu / r0);
orbitalPeriod = 2 * pi * sqrt(r0^3 / mu);

if bdIsLoaded(model)
    close_system(model, 0);
end

if exist(modelPath, 'file')
    delete(modelPath);
end

new_system(model);
open_system(model);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [model '/Two-Body Gravity'], ...
    'Position', [235 88 360 162]);
add_block('simulink/Continuous/Integrator', ...
    [model '/Velocity Integrator'], ...
    'Position', [410 95 440 155], ...
    'InitialCondition', sprintf('[0; %.12f]', v0));
add_block('simulink/Continuous/Integrator', ...
    [model '/Position Integrator'], ...
    'Position', [500 95 530 155], ...
    'InitialCondition', sprintf('[%.12f; 0]', r0));
add_block('simulink/Sinks/Scope', ...
    [model '/Position Scope'], ...
    'Position', [645 88 675 122], ...
    'NumInputPorts', '1');
add_block('simulink/Sinks/To Workspace', ...
    [model '/Position To Workspace'], ...
    'Position', [620 145 735 175], ...
    'VariableName', 'position_history', ...
    'SaveFormat', 'Structure With Time');
add_block('simulink/Sinks/To Workspace', ...
    [model '/Velocity To Workspace'], ...
    'Position', [450 188 565 218], ...
    'VariableName', 'velocity_history', ...
    'SaveFormat', 'Structure With Time');

rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', [model '/Two-Body Gravity']);
chart.Script = sprintf([ ...
    'function a = fcn(r)\n' ...
    '%% Two-body orbital acceleration in the orbital plane.\n' ...
    'mu = %.13f;\n' ...
    'norm_r = sqrt(r(1)^2 + r(2)^2);\n' ...
    'a = -mu .* r ./ (norm_r^3);\n' ...
    'end\n'], mu);

add_line(model, 'Two-Body Gravity/1', 'Velocity Integrator/1', 'autorouting', 'on');
add_line(model, 'Velocity Integrator/1', 'Position Integrator/1', 'autorouting', 'on');
add_line(model, 'Position Integrator/1', 'Two-Body Gravity/1', 'autorouting', 'on');
add_line(model, 'Position Integrator/1', 'Position Scope/1', 'autorouting', 'on');
add_line(model, 'Position Integrator/1', 'Position To Workspace/1', 'autorouting', 'on');
add_line(model, 'Velocity Integrator/1', 'Velocity To Workspace/1', 'autorouting', 'on');

set_param(model, ...
    'Solver', 'ode45', ...
    'StopTime', num2str(orbitalPeriod, '%.12f'), ...
    'MaxStep', '10', ...
    'ReturnWorkspaceOutputs', 'on');

save_system(model, modelPath);

simOut = sim(model);
positionHistory = simOut.get('position_history');
velocityHistory = simOut.get('velocity_history');

t = positionHistory.time;
pos = squeeze(positionHistory.signals.values).';
vel = squeeze(velocityHistory.signals.values).';

radius = vecnorm(pos, 2, 2);
speed = vecnorm(vel, 2, 2);
finalPositionError = norm(pos(end, :) - pos(1, :));
finalVelocityError = norm(vel(end, :) - vel(1, :));

fprintf('MODEL_PATH=%s\n', modelPath);
fprintf('ALTITUDE_KM=%.3f\n', altitude);
fprintf('ORBITAL_PERIOD_S=%.6f\n', orbitalPeriod);
fprintf('INITIAL_SPEED_KMPS=%.6f\n', v0);
fprintf('RADIUS_MIN_KM=%.6f\n', min(radius));
fprintf('RADIUS_MAX_KM=%.6f\n', max(radius));
fprintf('SPEED_MIN_KMPS=%.6f\n', min(speed));
fprintf('SPEED_MAX_KMPS=%.6f\n', max(speed));
fprintf('FINAL_POSITION_ERROR_KM=%.6f\n', finalPositionError);
fprintf('FINAL_VELOCITY_ERROR_KMPS=%.6f\n', finalVelocityError);
fprintf('SIM_END_TIME_S=%.6f\n', t(end));
