 %% Plot optimization results
 function plot_results(X_opt, U_opt, params)
     N = params.N;
     dt = params.dt;
     t_hist = (0:N+1)' * dt;
     
     r = X_opt(1:3, :);
     v = X_opt(4:6, :);
     m = exp(X_opt(7, :));
     
     T_norm = zeros(1, N+1);
     for k = 1:N+1
         T_norm(k) = norm(U_opt(:, k)) * m(k);
     end
     
     %% Figure 1: 3D trajectory
     figure('Position', [100 100 1400 900]);
     
     subplot(2, 3, 1);
     plot3(r(2,:), r(3,:), r(1,:), 'b-', 'LineWidth', 2); hold on;
     plot3(r(2,1), r(3,1), r(1,1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
     plot3(r(2,end), r(3,end), r(1,end), 'r^', 'MarkerSize', 10, 'LineWidth', 2);
     xlabel('y (m)'); ylabel('z (m)'); zlabel('x - altitude (m)');
     title('3D Descent Trajectory');
     legend('Trajectory', 'Start', 'End', 'Location', 'best');
     grid on; axis equal; view(45, 30);
     
     %% Figure 2: Altitude
     subplot(2, 3, 2);
     plot(t_hist, r(1,:), 'b-', 'LineWidth', 2);
     xlabel('Time (s)'); ylabel('Altitude (m)');
     title('Altitude vs Time');
     grid on;
     
     %% Figure 3: Velocity
     subplot(2, 3, 3);
     v_norm = sqrt(sum(v.^2, 1));
     plot(t_hist, v_norm, 'r-', 'LineWidth', 2);
     xlabel('Time (s)'); ylabel('Velocity (m/s)');
     title('Velocity vs Time');
     grid on;
     
     %% Figure 4: Thrust magnitude
     subplot(2, 3, 4);
     plot((0:N)*dt, T_norm, 'm-', 'LineWidth', 2); hold on;
     yline(params.T_max, 'r--', 'T_{max}');
     yline(params.T_min, 'g--', 'T_{min}');
     xlabel('Time (s)'); ylabel('Thrust (N)');
     title('Thrust Magnitude');
     legend('Thrust', 'Upper bound', 'Lower bound');
     grid on; ylim([0, params.T_max * 1.2]);
     
     %% Figure 5: Mass
     subplot(2, 3, 5);
     plot(t_hist, m, 'g-', 'LineWidth', 2); hold on;
     yline(params.m_dry, 'r--', 'Dry mass');
     xlabel('Time (s)'); ylabel('Mass (kg)');
     title('Mass Variation');
     grid on;
     
     %% Figure 6: Glide slope check
     subplot(2, 3, 6);
     r_horiz = sqrt(r(2,:).^2 + r(3,:).^2);
     glide_bound = tan(params.gamma_gs_rad) * r_horiz;
     plot(r_horiz, r(1,:), 'b-', 'LineWidth', 2); hold on;
     plot(r_horiz, glide_bound, 'r--', 'LineWidth', 1.5);
     xlabel('Horiz. Distance (m)'); ylabel('Altitude (m)');
     title('Glide Slope Check');
     legend('Trajectory', 'Glide Slope Bound');
     grid on;
     
     sgtitle('Mars Powered Descent - Convex Guidance Results');
 end
