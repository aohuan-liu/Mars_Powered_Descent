%% MILESTONE 1: Eq.6 conventional glide-slope constraint
%  Baseline (Problem 4, no obstacle) vs Problem 4 + Eq.(6).
%  NOTE: the paper's Table 3 (~505.49 kg) is the Fig.18(d) scenario
%  (tf ~ 138 s) and requires more fuel than the 500 kg capacity implied by
%  Table 1 (m_dry=1405) -- see TECHNICAL_REPORT.md section 8.1. The tf=80
%  comparison in this script is against the reproduced baseline only.
%
%  Run:  matlab -batch "cd('...mars_project'); run run_milestone1.m"

clear; clc; close all;
addpath(genpath(pwd));

params = mars_params;

fprintf('============================================================\n');
fprintf('MILESTONE 1 - Eq.6 conventional glide-slope constraint\n');
fprintf('============================================================\n');

% ------------------------------------------------------------------
% 1) Baseline: Problem 4, no obstacle constraint
% ------------------------------------------------------------------
fprintf('\n--- 1) Baseline (Problem 4, no constraint) ---\n');
[X0, U0, sig0, st0, info0] = solve_fuel_optimal(params);
m0 = trajectory_metrics(X0, params, 'glide');
fprintf('Fuel used          : %.2f kg\n', m0.fuel);
fprintf('Eq.6 min margin   : %.2f m  (negative = violation)\n', m0.min_margin);

% ------------------------------------------------------------------
% 2) Problem 4 + Eq.6 conventional glide-slope (gamma_gs = 10 deg)
% ------------------------------------------------------------------
fprintf('\n--- 2) Problem 4 + Eq.6 (gamma_gs = 10 deg) ---\n');
opts.constraint = 'glide';
[X1, U1, sig1, st1, info1] = solve_fuel_optimal(params, opts);
m1 = trajectory_metrics(X1, params, 'glide');
fprintf('Fuel used          : %.2f kg\n', m1.fuel);
fprintf('Eq.6 min margin   : %.2f m\n', m1.min_margin);

fprintf('\n------------------------------------------------------------\n');
fprintf('Summary (Milestone 1)\n');
fprintf('  Baseline fuel             : %.2f kg  (paper: n/a)\n', m0.fuel);
fprintf('  Baseline Eq.6 violation   : %.2f m\n', m0.min_margin);
fprintf('  Eq.6-constrained fuel     : %.2f kg  (tf=80; paper 505.49 is Fig.18d/tf~138 s, see report 8.1)\n', m1.fuel);
fprintf('  Eq.6 min margin           : %.2f m\n', m1.min_margin);
fprintf('  Eq.6 slack tightness      : %.3e  (max|sigma-||u|||)\n', info1.tightness);
fprintf('  Eq.6 raw thrust m*sigma   : [%.2f, %.2f] N  (paper: [%.2f, %.2f])\n', ...
        info1.thrust_min, info1.thrust_max, params.T_min, params.T_max);
fprintf('------------------------------------------------------------\n');

if ~exist('results', 'dir'), mkdir('results'); end
save(fullfile('results', 'results_milestone1.mat'), 'params', 'X0', 'U0', 'info0', 'm0', ...
     'X1', 'U1', 'info1', 'm1');

% Simple comparison figure
figure('Name', 'Milestone 1: Eq.6 glide-slope');
t = (0:params.N+1) * params.dt;
subplot(1,2,1);
plot(t, X0(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, X1(1,:), 'r-', 'LineWidth', 1.5);
ylabel('Altitude (m)'); xlabel('Time (s)');
legend('Baseline', 'Eq.6 constrained', 'Location', 'best');
title('Altitude vs time'); grid on;
subplot(1,2,2);
d0 = sqrt(X0(2,:).^2 + X0(3,:).^2);
d1 = sqrt(X1(2,:).^2 + X1(3,:).^2);
bound = @(d) tan(params.gamma_gs_rad) * d;
dd = linspace(0, 1600, 200);
plot(dd, bound(dd), 'k--', 'LineWidth', 1.5); hold on;
plot(d0, X0(1,:), 'b-', 'LineWidth', 1.5);
plot(d1, X1(1,:), 'r-', 'LineWidth', 1.5);
xlabel('Horiz. distance (m)'); ylabel('Altitude (m)');
legend('Eq.6 cone', 'Baseline', 'Eq.6 constrained', 'Location', 'best');
title('Glide-slope corridor'); grid on;
