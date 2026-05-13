function Plot_animation(t)
% Plot_animation - Visualise EKF SLAM results at time t
%   t : current time step

global num_landmarks real_landmarks index_seen index_fov fusion_flag ...
       pos_landmark1 pos_landmark2 pos_landmark_SF ...
       cov_landmark1 cov_landmark2 cov_landmark_SF ...
       pos_car1 pos_car2 real_car1 real_car2 ...
       meas_landmark1 meas_landmark2 max_read_distance
global block width   % for road

clf;
hold on;

% Plot the roads
if exist('Plot_road', 'file')
    Plot_road;
end

% Plot the true landmarks
h_true = plot(real_landmarks(1,:), real_landmarks(2,:), 'b*', 'DisplayName', 'True Landmark');

% Plot estimated landmarks and their uncertainty ellipses
for l = 1:num_landmarks
    % Car 1 landmarks
    if index_seen(1,l,t)
        plot(pos_landmark1(1,l,t), pos_landmark1(2,l,t), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
        draw_ellipse(pos_landmark1(:,l,t), cov_landmark1(:,:,l,t), 2, 'k');
    end
    % Car 2 landmarks
    if index_seen(2,l,t)
        plot(pos_landmark2(1,l,t), pos_landmark2(2,l,t), 'ko', 'MarkerSize', 6);
        draw_ellipse(pos_landmark2(:,l,t), cov_landmark2(:,:,l,t), 2, 'k');
    end
    % Fused landmarks (if fusion active)
    if fusion_flag > 0 && index_seen(3,l,t)
        plot(pos_landmark_SF(1,l,t), pos_landmark_SF(2,l,t), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        draw_ellipse(pos_landmark_SF(:,l,t), cov_landmark_SF(:,:,l,t), 2, 'g');
    end
end

% Dummy handles for legend
h_est1 = plot(NaN, NaN, 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Estimated Landmark (Car1/2)');
if fusion_flag > 0
    h_fused = plot(NaN, NaN, 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Fused Landmark');
end

% Plot true car trajectories
h_traj1 = plot(real_car1(1,1:t), real_car1(2,1:t), 'r-', 'LineWidth', 1.5, 'DisplayName', 'True Trajectory Car1');
plot(real_car2(1,1:t), real_car2(2,1:t), 'r-', 'LineWidth', 1.5);
% Mark current positions
plot(real_car1(1,t), real_car1(2,t), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(real_car2(1,t), real_car2(2,t), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(real_car1(1,t)+1, real_car1(2,t)+1, 'Car1', 'FontSize', 10);
text(real_car2(1,t)+1, real_car2(2,t)+1, 'Car2', 'FontSize', 10);

% Plot estimated car positions
h_est_car1 = plot(pos_car1(1,t), pos_car1(2,t), 'rs', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Estimated Car1');
h_est_car2 = plot(pos_car2(1,t), pos_car2(2,t), 'rs', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Estimated Car2');

% Show sensor measurements as arrows (gray)
for l = 1:num_landmarks
    if index_fov(1, l, t)
        range = meas_landmark1(1, l, t);
        bearing = wrapToPi(meas_landmark1(2, l, t) + real_car1(3, t));
        x_end = real_car1(1, t) + range * cos(bearing);
        y_end = real_car1(2, t) + range * sin(bearing);
        plot([real_car1(1, t), x_end], [real_car1(2, t), y_end], 'Color', 0.6*ones(1,3), 'LineWidth', 0.5);
    end
    if index_fov(2, l, t)
        range = meas_landmark2(1, l, t);
        bearing = wrapToPi(meas_landmark2(2, l, t) + real_car2(3, t));
        x_end = real_car2(1, t) + range * cos(bearing);
        y_end = real_car2(2, t) + range * sin(bearing);
        plot([real_car2(1, t), x_end], [real_car2(2, t), y_end], 'Color', 0.6*ones(1,3), 'LineWidth', 0.5);
    end
end

% Show sensor field of view (circle)
theta = linspace(0, 2*pi, 20);
circ = max_read_distance * [cos(theta); sin(theta)];
plot(real_car1(1,t)+circ(1,:), real_car1(2,t)+circ(2,:), '--', 'Color', 0.5*ones(1,3));
plot(real_car2(1,t)+circ(1,:), real_car2(2,t)+circ(2,:), '--', 'Color', 0.5*ones(1,3));

% Build legend
legend_handles = [h_true, h_est1];
legend_names = {'True Landmark', 'Estimated Landmark'};
if fusion_flag > 0
    legend_handles = [legend_handles, h_fused];
    legend_names{end+1} = 'Fused Landmark';
end
legend_handles = [legend_handles, h_traj1, h_est_car1, h_est_car2];
legend_names = [legend_names, {'True Trajectory', 'Estimated Car1', 'Estimated Car2'}];
legend(legend_handles, legend_names, 'Location', 'best');

grid on;
xlabel('x (m)');
ylabel('y (m)');
axis([-30, 30, -30, 30]);
drawnow;
end

% ------------------------------------------------------------------------
function draw_ellipse(center, cov, n_sigma, color)
% Draw an ellipse representing n_sigma times the covariance matrix
    [V, D] = eig(cov);
    theta = linspace(0, 2*pi, 50);
    a = n_sigma * sqrt(D(1,1));
    b = n_sigma * sqrt(D(2,2));
    ell = [a * cos(theta); b * sin(theta)];
    ell_rot = V * ell;
    x = center(1) + ell_rot(1,:);
    y = center(2) + ell_rot(2,:);
    plot(x, y, color, 'LineWidth', 1);
end