%% EXPORT FIGURES from saved milestone results (no CVX needed)
%  Loads results_milestone1/2/3.mat and writes PNG files to results/.

clear; clc; close all;
addpath(genpath(pwd));

outdir = fullfile(pwd, 'results');
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

% ------------------------------------------------------------------
% Milestone 1: Eq.6 glide-slope
% ------------------------------------------------------------------
if exist(fullfile('results', 'results_milestone1.mat'), 'file')
    S1 = load(fullfile('results', 'results_milestone1.mat'));
    params = S1.params;
    t = (0:params.N+1) * params.dt;

    f = figure('Visible', 'off', 'Position', [100 100 1200 450]);
    subplot(1,2,1);
    plot(t, S1.X0(1,:), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, S1.X1(1,:), 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Altitude (m)');
    legend(sprintf('Baseline (%.2f kg)', S1.m0.fuel), ...
           sprintf('Eq.6 (%.2f kg)', S1.m1.fuel), 'Location', 'best');
    title('Altitude vs time'); grid on;
    subplot(1,2,2);
    dd = linspace(0, 1600, 200);
    plot(dd, tan(params.gamma_gs_rad) * dd, 'k--', 'LineWidth', 1.5); hold on;
    d0 = sqrt(S1.X0(2,:).^2 + S1.X0(3,:).^2);
    d1 = sqrt(S1.X1(2,:).^2 + S1.X1(3,:).^2);
    plot(d0, S1.X0(1,:), 'b-', 'LineWidth', 1.5);
    plot(d1, S1.X1(1,:), 'r-', 'LineWidth', 1.5);
    xlabel('Horiz. distance (m)'); ylabel('Altitude (m)');
    legend('Eq.6 cone', 'Baseline', 'Eq.6 constrained', 'Location', 'best');
    title('Glide-slope corridor'); grid on;
    sgtitle(sprintf('M1: Eq.6 (fuel %.2f vs %.2f kg)', S1.m0.fuel, S1.m1.fuel));
    saveas(f, fullfile(outdir, 'm1_glide_slope.png'));
    close(f);
end

% ------------------------------------------------------------------
% Milestone 2: Algorithm 1
% ------------------------------------------------------------------
if exist(fullfile('results', 'results_milestone2.mat'), 'file')
    S2 = load(fullfile('results', 'results_milestone2.mat'));
    params = S2.params;

    f = figure('Visible', 'off', 'Position', [100 100 1200 450]);
    subplot(1,2,1);
    plot([S2.outA.hist.iter], [S2.outA.hist.slack_norm], 'ko-', 'LineWidth', 1.5);
    set(gca, 'YScale', 'log');
    xlabel('SCP iteration j'); ylabel('||\zeta||_2');
    title('Stepwise: slack norm (Fig.19 analog)'); grid on;
    subplot(1,2,2);
    plot([S2.outB.hist.iter], [S2.outB.hist.slack_norm], 'bo-', 'LineWidth', 1.5); hold on;
    plot([S2.outC.hist.iter], [S2.outC.hist.slack_norm], 'ro-', 'LineWidth', 1.5);
    set(gca, 'YScale', 'log');
    xlabel('SCP iteration j'); ylabel('||\zeta||_2');
    legend(sprintf('tf=80 (%.2f kg)', S2.mB.fuel), ...
           sprintf('tf=100 (%.2f kg)', S2.mC.fuel));
    title('Relaxed: slack norm'); grid on;
    sgtitle('M2: Algorithm 1 convergence');
    saveas(f, fullfile(outdir, 'm2_convergence.png'));
    close(f);

    f = figure('Visible', 'off', 'Position', [100 100 1200 450]);
    subplot(1,2,1);
    t = (0:params.N+1) * params.dt;
    plot(t, S2.outB.X(1,:), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, S2.outC.X(1,:), 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Altitude (m)');
    legend('tf=80', 'tf=100'); title('Relaxed: altitude'); grid on;
    subplot(1,2,2);
    dB = sqrt(S2.outB.X(2,:).^2 + S2.outB.X(3,:).^2);
    dC = sqrt(S2.outC.X(2,:).^2 + S2.outC.X(3,:).^2);
    plot(dB, S2.outB.X(1,:), 'b-', 'LineWidth', 1.5); hold on;
    plot(dC, S2.outC.X(1,:), 'r-', 'LineWidth', 1.5);
    l1 = params.l1; l2 = params.l2; h = params.h_safe;
    dd = linspace(0, l2, 200);
    plot(dd, max(0, (dd - l1) * h / (l2 - l1)), 'k--', 'LineWidth', 1.5);
    plot(dd, h * ones(size(dd)), 'k--', 'LineWidth', 1.5);
    xlabel('Horiz. distance (m)'); ylabel('Altitude (m)');
    legend('tf=80', 'tf=100', 'Relaxed bound', 'Location', 'best');
    title('Relaxed glide-slope corridor'); grid on;
    sgtitle('M2: obstacle-avoiding trajectories (Fig.20/21 analogs)');
    saveas(f, fullfile(outdir, 'm2_trajectories.png'));
    close(f);

    if isfield(S2, 'scanD')
        f = figure('Visible', 'off', 'Position', [100 100 700 450]);
        plot([S2.scanD.tf], [S2.scanD.fuel], 'ko-', 'LineWidth', 1.5); hold on;
        yline(500, 'r--', 'fuel capacity');
        yline(365.18, 'b--', '365.18 kg');
        yline(409.12, 'm--', '409.12 kg');
        xlabel('tf (s)'); ylabel('Fuel-optimal consumption (kg)');
        title('M2D: fuel vs tf, relaxed constraint (Fig.22 analog)'); grid on;
        saveas(f, fullfile(outdir, 'm2_feasible_region.png'));
        close(f);
    end
end

% ------------------------------------------------------------------
% Milestone 3: Algorithm 2
% ------------------------------------------------------------------
if exist(fullfile('results', 'results_milestone3.mat'), 'file')
    S3 = load(fullfile('results', 'results_milestone3.mat'));
    params = S3.params;

    f = figure('Visible', 'off', 'Position', [100 100 1100 450]);
    subplot(1,2,1);
    plot([S3.scanA.alpha], [S3.scanA.elev], 'bo-', 'LineWidth', 1.5);
    xlabel('\alpha'); ylabel('Integral of elevation angle');
    title('M3A: alpha scan (Fig.8 analog)'); grid on;
    subplot(1,2,2);
    t = (0:params.N+1) * params.dt;
    for i = 1:numel(S3.scanA)
        plot(t, S3.scanA(i).X(1,:), 'LineWidth', 1.2); hold on;
    end
    xlabel('Time (s)'); ylabel('Altitude (m)');
    legend(cellstr(num2str([S3.scanA.alpha]', '\alpha=%.1f')), 'Location', 'best');
    title('M3A: alpha scan trajectories (Fig.6 analog)'); grid on;
    saveas(f, fullfile(outdir, 'm3_alpha_scan.png'));
    close(f);

    f = figure('Visible', 'off', 'Position', [100 100 1100 450]);
    subplot(1,2,1);
    plot([S3.scanB.tf], [S3.scanB.elev], 'bo-', 'LineWidth', 1.5); hold on;
    plot([S3.scanC.tf], [S3.scanC.elev], 'ro-', 'LineWidth', 1.5);
    xlabel('tf (s)'); ylabel('Integral of elevation angle');
    legend('m_fuel=365.18', 'm_fuel=409.12'); grid on;
    title('M3: elev integral vs tf (Fig.25/29 analog)');
    subplot(1,2,2);
    plot([S3.scanB.tf], [S3.scanB.obj], 'bo-', 'LineWidth', 1.5); hold on;
    plot([S3.scanC.tf], [S3.scanC.obj], 'ro-', 'LineWidth', 1.5);
    xlabel('tf (s)'); ylabel('Objective value');
    legend('m_fuel=365.18', 'm_fuel=409.12'); grid on;
    title('M3: objective vs tf (Fig.26/30 analog)');
    saveas(f, fullfile(outdir, 'm3_fixed_fuel_curves.png'));
    close(f);
end

fprintf('Figures written to %s\n', outdir);
