%% MILESTONE 3: Algorithm 2 - elevation-angle objective
%  Paper Section 4.2 / 5.3:
%    A) alpha scan: stepwise constraint, Tables 1-2, tf=80, m_fuel=400 kg,
%       alpha = [0.1 0.6 1.5 2.5 5 10 20] -> elevation integral (Fig.8)
%    B) Fixed m_fuel=365.18 kg, relaxed glide-slope, r0=[1500,0,500],
%       v0=[-75,0,-100], varying tf -> integral / objective curves
%    C) Fixed m_fuel=409.12 kg, same scenario, varying tf

clear; clc; close all;
addpath(genpath(pwd));

params = mars_params;

fprintf('============================================================\n');
fprintf('MILESTONE 3 - Algorithm 2 (elevation-angle objective)\n');
fprintf('============================================================\n');

% ------------------------------------------------------------------
% A) alpha scan (Section 4.2, Fig.8 analog)
% ------------------------------------------------------------------
fprintf('\n--- A) alpha scan: stepwise, Tables 1-2, tf=80, m_fuel=400 ---\n');
alphas = [0.1 0.6 1.5 2.5 5 10 20];
scanA = struct();
for i = 1:numel(alphas)
    opts.alpha = alphas(i);
    opts.m_fuel = 400;
    opts.constraint = 'stepwise';
    opts.verbose = false;
    out = solve_algorithm2(params, opts);
    scanA(i).alpha = alphas(i);
    scanA(i).fuel = out.fuel;
    scanA(i).elev = out.elev_integral;
    scanA(i).obj = out.obj_value;
    scanA(i).min_margin = out.min_margin;
    scanA(i).X = out.X;
    scanA(i).iterations = out.iterations;
    scanA(i).status = out.status;
    fprintf('alpha=%5.2f  fuel=%7.2f kg  elev_int=%8.2f  obj=%10.1f  margin=%8.2f m  iters=%d\n', ...
            alphas(i), out.fuel, out.elev_integral, out.obj_value, out.min_margin, out.iterations);
end

% ------------------------------------------------------------------
% B) Fixed m_fuel=365.18 kg, relaxed constraint, varying tf
% ------------------------------------------------------------------
fprintf('\n--- B) m_fuel=365.18 kg, relaxed, r0=[1500,0,500], v0=[-75,0,-100] ---\n');

% Feasible tf range: from the Algorithm-1 fuel-vs-tf curve (Fig.22 analog).
% Points with fuel_optimal <= m_fuel are inside the feasible region.
m_fuelB = 365.18;
if exist(fullfile('results', 'results_milestone2.mat'), 'file')
    S2 = load(fullfile('results', 'results_milestone2.mat'));
    tf_grid = [S2.scanD.tf];
    fuel_grid = [S2.scanD.fuel];
    conv_grid = [S2.scanD.converged];
else
    tf_grid = 60:5:110;
    fuel_grid = nan(size(tf_grid));
    conv_grid = false(size(tf_grid));
    for i = 1:numel(tf_grid)
        optsT.constraint = 'relaxed';
        optsT.r0 = [1500; 0; 500];
        optsT.v0 = [-75; 0; -100];
        optsT.tf = tf_grid(i);
        optsT.verbose = false;
        outT = solve_algorithm1(params, optsT);
        fuel_grid(i) = outT.fuel;
        conv_grid(i) = outT.converged;
    end
end
% Feasibility tolerance: the Algorithm-1 fuel estimate carries ~0.1-0.35 kg
% Taylor/discretization error. 0.2 kg keeps the paper's tf=80 point (diff
% 0.11 kg) while excluding tf=100 for m_fuel=409.12 (diff 0.35 kg, where the
% elevation problem is genuinely infeasible).
tfB = tf_grid(conv_grid & fuel_grid <= m_fuelB + 0.2);
fprintf('Feasible tf range for m_fuel=%.2f: [%d, %d]\n', m_fuelB, min(tfB), max(tfB));

scanB = struct();
for i = 1:numel(tfB)
    opts.alpha = 1;
    opts.m_fuel = m_fuelB;
    opts.constraint = 'relaxed';
    opts.tf = tfB(i);
    opts.r0 = [1500; 0; 500];
    opts.v0 = [-75; 0; -100];
    opts.verbose = false;
    out = solve_algorithm2(params, opts);
    if ~strcmp(out.status, 'Solved') || ~out.converged
        opts.H = 5;   % finer homotopy retry for marginal points
        out = solve_algorithm2(params, opts);
    end
    scanB(i).tf = tfB(i);
    scanB(i).fuel = out.fuel;
    scanB(i).elev = out.elev_integral;
    scanB(i).obj = out.obj_value;
    scanB(i).min_margin = out.min_margin;
    scanB(i).status = out.status;
    scanB(i).X = out.X;
    scanB(i).iterations = out.iterations;
    fprintf('tf=%3d  fuel=%7.2f kg  elev_int=%8.2f  obj=%10.1f  margin=%8.2f m  iters=%d\n', ...
            tfB(i), out.fuel, out.elev_integral, out.obj_value, out.min_margin, out.iterations);
end

% ------------------------------------------------------------------
% C) Fixed m_fuel=409.12 kg, relaxed constraint, varying tf
% ------------------------------------------------------------------
fprintf('\n--- C) m_fuel=409.12 kg, relaxed, same scenario ---\n');
m_fuelC = 409.12;
tfC = tf_grid(conv_grid & fuel_grid <= m_fuelC + 0.2);
fprintf('Feasible tf range for m_fuel=%.2f: [%d, %d]\n', m_fuelC, min(tfC), max(tfC));

scanC = struct();
for i = 1:numel(tfC)
    opts.alpha = 1;
    opts.m_fuel = m_fuelC;
    opts.constraint = 'relaxed';
    opts.tf = tfC(i);
    opts.r0 = [1500; 0; 500];
    opts.v0 = [-75; 0; -100];
    opts.verbose = false;
    out = solve_algorithm2(params, opts);
    if ~strcmp(out.status, 'Solved') || ~out.converged
        opts.H = 5;   % finer homotopy retry for marginal points
        out = solve_algorithm2(params, opts);
    end
    scanC(i).tf = tfC(i);
    scanC(i).fuel = out.fuel;
    scanC(i).elev = out.elev_integral;
    scanC(i).obj = out.obj_value;
    scanC(i).min_margin = out.min_margin;
    scanC(i).status = out.status;
    scanC(i).X = out.X;
    scanC(i).iterations = out.iterations;
    fprintf('tf=%3d  fuel=%7.2f kg  elev_int=%8.2f  obj=%10.1f  margin=%8.2f m  iters=%d\n', ...
            tfC(i), out.fuel, out.elev_integral, out.obj_value, out.min_margin, out.iterations);
end

if ~exist('results', 'dir'), mkdir('results'); end
save(fullfile('results', 'results_milestone3.mat'), 'params', 'scanA', 'scanB', 'scanC');

% ------------------------------------------------------------------
% Figures
% ------------------------------------------------------------------
figure('Name', 'Milestone 3A: alpha scan');
subplot(1,2,1);
plot([scanA.alpha], [scanA.elev], 'bo-', 'LineWidth', 1.5);
xlabel('alpha'); ylabel('Integral of elevation angle');
title('Fig.8 analog: stepwise, m_fuel=400'); grid on;
subplot(1,2,2);
plot([scanA.alpha], [scanA.obj], 'ro-', 'LineWidth', 1.5);
xlabel('alpha'); ylabel('Objective value');
title('Objective vs alpha'); grid on;

figure('Name', 'Milestone 3BC: fixed fuel');
subplot(2,2,1);
plot([scanB.tf], [scanB.elev], 'bo-', 'LineWidth', 1.5); hold on;
plot([scanC.tf], [scanC.elev], 'ro-', 'LineWidth', 1.5);
xlabel('tf (s)'); ylabel('Integral of elevation angle');
legend('m_fuel=365.18', 'm_fuel=409.12'); grid on;
title('Fig.25/29 analog');
subplot(2,2,2);
plot([scanB.tf], [scanB.obj], 'bo-', 'LineWidth', 1.5); hold on;
plot([scanC.tf], [scanC.obj], 'ro-', 'LineWidth', 1.5);
xlabel('tf (s)'); ylabel('Objective value');
legend('m_fuel=365.18', 'm_fuel=409.12'); grid on;
title('Fig.26/30 analog');

t = (0:params.N+1) * params.dt;
subplot(2,2,3);
for i = 1:numel(scanA)
    plot(t, scanA(i).X(1,:), 'LineWidth', 1.2); hold on;
end
xlabel('Time (s)'); ylabel('Altitude (m)');
legend(cellstr(num2str([scanA.alpha]', 'alpha=%.1f')), 'Location', 'best');
title('Alpha scan trajectories (Fig.6 analog)'); grid on;

subplot(2,2,4);
d = sqrt(scanB(1).X(2,:).^2 + scanB(1).X(3,:).^2);
plot(d, scanB(1).X(1,:), 'b-', 'LineWidth', 1.5); hold on;
l1 = params.l1; l2 = params.l2; h = params.h_safe;
dd = linspace(0, l2, 200);
plot(dd, max(0, (dd - l1) * h / (l2 - l1)), 'k--', 'LineWidth', 1.5);
plot(dd, h * ones(size(dd)), 'k--', 'LineWidth', 1.5);
xlabel('Horiz. distance (m)'); ylabel('Altitude (m)');
title('Relaxed corridor, tf=80, m_fuel=365.18'); grid on;
