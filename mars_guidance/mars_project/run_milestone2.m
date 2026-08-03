%% MILESTONE 2: Algorithm 1 - homotopy obstacle avoidance
%  Cases from the paper (Section 5.2):
%    A) Stepwise constraint (Eq.8), Tables 1-2 (r0=[1500,0,1500],
%       v0=[-75,0,70]), tf=80 s  -> convergence in ~4 iterations (Fig.19)
%    B) Relaxed glide-slope (Eq.7), r0=[1500,0,500], v0=[-75,0,-100],
%       tf=80 s  -> fuel target 365.18 kg (Fig.20)
%    C) Same as B but tf=100 s -> fuel target 409.12 kg (Fig.21)

clear; clc; close all;
addpath(genpath(pwd));

params = mars_params;

fprintf('============================================================\n');
fprintf('MILESTONE 2 - Algorithm 1 (homotopy + slack)\n');
fprintf('============================================================\n');

% ------------------------------------------------------------------
% Case A: stepwise constraint, Tables 1-2
% ------------------------------------------------------------------
fprintf('\n--- A) Stepwise constraint, Tables 1-2, tf=80 ---\n');
optsA.constraint = 'stepwise';
outA = solve_algorithm1(params, optsA);
mA = trajectory_metrics(outA.X, params, 'stepwise');
fprintf('Fuel: %.2f kg | min margin: %.2f m | iterations: %d | tightness: %.2e\n', ...
        mA.fuel, mA.min_margin, outA.iterations, outA.tightness);

% ------------------------------------------------------------------
% Case B: relaxed glide-slope, Fig.20 scenario, tf=80
% ------------------------------------------------------------------
fprintf('\n--- B) Relaxed glide-slope, r0=[1500,0,500], v0=[-75,0,-100], tf=80 ---\n');
optsB.constraint = 'relaxed';
optsB.r0 = [1500; 0; 500];
optsB.v0 = [-75; 0; -100];
outB = solve_algorithm1(params, optsB);
mB = trajectory_metrics(outB.X, params, 'relaxed');
fprintf('Fuel: %.2f kg (paper: 365.18) | min margin: %.2f m | iterations: %d | tightness: %.2e\n', ...
        mB.fuel, mB.min_margin, outB.iterations, outB.tightness);

% ------------------------------------------------------------------
% Case C: relaxed glide-slope, same ICs, tf=100
% ------------------------------------------------------------------
fprintf('\n--- C) Relaxed glide-slope, tf=100 ---\n');
optsC = optsB;
optsC.tf = 100;
outC = solve_algorithm1(params, optsC);
mC = trajectory_metrics(outC.X, params, 'relaxed');
fprintf('Fuel: %.2f kg (paper: 409.12) | min margin: %.2f m | iterations: %d | tightness: %.2e\n', ...
        mC.fuel, mC.min_margin, outC.iterations, outC.tightness);

% ------------------------------------------------------------------
% Case D: feasible region, fuel vs terminal time (Fig.22 analog)
%  Algorithm 1, relaxed glide-slope, r0=[1500,0,500], v0=[-75,0,-100]
% ------------------------------------------------------------------
fprintf('\n--- D) Feasible region: fuel vs tf (Fig.22 analog) ---\n');
tfD = 60:5:110;
scanD = struct();
for i = 1:numel(tfD)
    optsD.constraint = 'relaxed';
    optsD.r0 = [1500; 0; 500];
    optsD.v0 = [-75; 0; -100];
    optsD.tf = tfD(i);
    optsD.verbose = false;
    outD = solve_algorithm1(params, optsD);
    scanD(i).tf = tfD(i);
    scanD(i).fuel = outD.fuel;
    scanD(i).converged = outD.converged;
    scanD(i).margin = outD.min_margin;
    scanD(i).status = outD.status;
    fprintf('tf=%3d  fuel=%8.2f kg  converged=%d  margin=%8.2f m\n', ...
            tfD(i), outD.fuel, outD.converged, outD.min_margin);
end

if ~exist('results', 'dir'), mkdir('results'); end
save(fullfile('results', 'results_milestone2.mat'), 'params', 'outA', 'mA', 'outB', 'mB', ...
     'outC', 'mC', 'scanD');

% ------------------------------------------------------------------
% Convergence figure: slack norm per iteration (analog of Fig.19)
% ------------------------------------------------------------------
figure('Name', 'Milestone 2: Algorithm 1 convergence');
subplot(1,2,1);
plot([outA.hist.iter], [outA.hist.slack_norm], 'ko-', 'LineWidth', 1.5);
set(gca, 'YScale', 'log');
xlabel('SCP iteration j'); ylabel('||zeta||_2');
title('Stepwise: slack norm (Fig.19 analog)'); grid on;
subplot(1,2,2);
plot([outB.hist.iter], [outB.hist.slack_norm], 'bo-', 'LineWidth', 1.5); hold on;
plot([outC.hist.iter], [outC.hist.slack_norm], 'ro-', 'LineWidth', 1.5);
set(gca, 'YScale', 'log');
xlabel('SCP iteration j'); ylabel('||zeta||_2');
legend('tf=80 (365.18 target)', 'tf=100 (409.12 target)');
title('Relaxed: slack norm'); grid on;

figure('Name', 'Milestone 2D: feasible region');
plot([scanD.tf], [scanD.fuel], 'ko-', 'LineWidth', 1.5); hold on;
yline(500, 'r--', 'fuel capacity');
xlabel('tf (s)'); ylabel('Fuel-optimal consumption (kg)');
title('Fuel vs tf, relaxed constraint (Fig.22 analog)'); grid on;

figure('Name', 'Milestone 2: trajectories');
t = (0:params.N+1) * params.dt;
subplot(1,2,1);
plot(t, outB.X(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, outC.X(1,:), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Altitude (m)');
legend('tf=80', 'tf=100'); title('Relaxed: altitude'); grid on;
subplot(1,2,2);
d = sqrt(outB.X(2,:).^2 + outB.X(3,:).^2);
plot(d, outB.X(1,:), 'b-', 'LineWidth', 1.5); hold on;
d = sqrt(outC.X(2,:).^2 + outC.X(3,:).^2);
plot(d, outC.X(1,:), 'r-', 'LineWidth', 1.5);
l1 = params.l1; l2 = params.l2; h = params.h_safe;
dd = linspace(0, l2, 200);
plot(dd, max(0, (dd - l1) * h / (l2 - l1)), 'k--', 'LineWidth', 1.5);
plot(dd, h * ones(size(dd)), 'k--', 'LineWidth', 1.5);
xlabel('Horiz. distance (m)'); ylabel('Altitude (m)');
legend('tf=80', 'tf=100', 'Relaxed bound', 'Location', 'best');
title('Relaxed glide-slope corridor'); grid on;
