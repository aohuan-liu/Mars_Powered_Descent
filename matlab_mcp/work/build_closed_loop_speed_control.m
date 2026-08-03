% 模型保存到本脚本所在目录（matlab_mcp/work/），避免硬编码绝对路径
projectRoot = fileparts(mfilename('fullpath'));
model = 'closed_loop_speed_control';

% Assumed speed plant: a normalized first-order motor model.
plant = tf(1, [0.5 1]);

% Search for a PI controller that satisfies the transient requirements
% while keeping steady-state error effectively zero. Among all feasible
% solutions, pick the mildest controller to avoid unnecessary aggressiveness.
best = struct('Kp', NaN, 'Ki', NaN, 'Overshoot', inf, ...
    'SettlingTime', inf, 'SteadyStateError', inf, 'Cost', inf);

kpCandidates = logspace(-1, 1.5, 50);
kiCandidates = logspace(-1, 1.5, 50);

for kp = kpCandidates
    for ki = kiCandidates
        controller = pid(kp, ki);
        closedLoop = feedback(controller * plant, 1);
        info = stepinfo(closedLoop, 'SettlingTimeThreshold', 0.02);
        if isnan(info.SettlingTime) || isinf(info.SettlingTime)
            continue;
        end

        steadyStateError = abs(1 - dcgain(closedLoop));
        isFeasible = info.Overshoot < 5 && info.SettlingTime < 2 && steadyStateError < 1e-3;
        if ~isFeasible
            continue;
        end

        cost = kp + ki;
        if cost < best.Cost
            best.Kp = kp;
            best.Ki = ki;
            best.Overshoot = info.Overshoot;
            best.SettlingTime = info.SettlingTime;
            best.SteadyStateError = steadyStateError;
            best.Cost = cost;
        end
    end
end

if isnan(best.Kp)
    error('No PI controller found that satisfies the design requirements.');
end

if bdIsLoaded(model)
    close_system(model, 0);
end

modelPath = fullfile(projectRoot, [model '.slx']);
if exist(modelPath, 'file')
    delete(modelPath);
end

new_system(model);
open_system(model);

add_block('simulink/Sources/Step', [model '/Reference'], ...
    'Position', [30 80 60 110], 'Time', '0', 'Before', '0', 'After', '1');
add_block('simulink/Math Operations/Sum', [model '/Error Sum'], ...
    'Position', [110 78 140 112], 'Inputs', '+-');
add_block('simulink/Continuous/PID Controller', [model '/PI Controller'], ...
    'Position', [190 70 280 120], ...
    'P', num2str(best.Kp, '%.6g'), ...
    'I', num2str(best.Ki, '%.6g'), ...
    'D', '0', ...
    'InitialConditionForIntegrator', '0');
add_block('simulink/Continuous/Transfer Fcn', [model '/Motor Plant'], ...
    'Position', [340 75 430 115], ...
    'Numerator', '[1]', ...
    'Denominator', '[0.5 1]');
add_block('simulink/Signal Routing/Mux', [model '/Mux'], ...
    'Position', [485 55 490 135], 'Inputs', '2');
add_block('simulink/Sinks/Scope', [model '/Scope'], ...
    'Position', [545 68 575 122]);
add_block('simulink/Sinks/To Workspace', [model '/Speed Output'], ...
    'Position', [545 145 635 175], ...
    'VariableName', 'speed_output', ...
    'SaveFormat', 'Structure With Time');
add_block('simulink/Sinks/To Workspace', [model '/Control Effort'], ...
    'Position', [340 145 430 175], ...
    'VariableName', 'control_effort', ...
    'SaveFormat', 'Structure With Time');

add_line(model, 'Reference/1', 'Error Sum/1', 'autorouting', 'on');
add_line(model, 'Error Sum/1', 'PI Controller/1', 'autorouting', 'on');
add_line(model, 'PI Controller/1', 'Motor Plant/1', 'autorouting', 'on');
add_line(model, 'Motor Plant/1', 'Error Sum/2', 'autorouting', 'on');
add_line(model, 'Reference/1', 'Mux/1', 'autorouting', 'on');
add_line(model, 'Motor Plant/1', 'Mux/2', 'autorouting', 'on');
add_line(model, 'Mux/1', 'Scope/1', 'autorouting', 'on');
add_line(model, 'Motor Plant/1', 'Speed Output/1', 'autorouting', 'on');
add_line(model, 'PI Controller/1', 'Control Effort/1', 'autorouting', 'on');

set_param(model, 'StopTime', '5');
save_system(model, modelPath);

simOut = sim(model, 'StopTime', '5');
speedData = simOut.get('speed_output');
t = speedData.time;
y = speedData.signals.values;
simInfo = stepinfo(y, t, 1, 'SettlingTimeThreshold', 0.02);
simEss = abs(1 - y(end));

fprintf('MODEL_PATH=%s\n', modelPath);
fprintf('Kp=%.6f\n', best.Kp);
fprintf('Ki=%.6f\n', best.Ki);
fprintf('OVERSHOOT=%.6f\n', simInfo.Overshoot);
fprintf('SETTLING_TIME=%.6f\n', simInfo.SettlingTime);
fprintf('STEADY_STATE_ERROR=%.6f\n', simEss);
