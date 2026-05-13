global real_car1 real_car2 pos_car1 pos_car2 time timesteps ...
       real_landmarks pos_landmark1 pos_landmark2 pos_landmark_SF ...
       cov_landmark1 cov_landmark2 cov_landmark_SF fusion_flag num_landmarks
%% Plot_analysis.m - Analysis of EKF SLAM results
% This script assumes the following variables exist in the workspace:
%   real_car1, real_car2, pos_car1, pos_car2, time, timesteps,
%   real_landmarks, pos_landmark1, pos_landmark2, pos_landmark_SF,
%   cov_landmark1, cov_landmark2, cov_landmark_SF, fusion_flag, num_landmarks

%% Car Trajectory
figure; hold on;
title('Car Trajectories');
plot(real_car1(1,:), real_car1(2,:), 'r', 'LineWidth', 1.5);
plot(real_car2(1,:), real_car2(2,:), 'b', 'LineWidth', 1.5);
plot(pos_car1(1,:), pos_car1(2,:), 'r--', 'LineWidth', 1.5);
plot(pos_car2(1,:), pos_car2(2,:), 'b--', 'LineWidth', 1.5);
grid on; xlabel('x (m)'); ylabel('y (m)');
axis([-30, 30, -30, 30]);
legend('True Car1', 'True Car2', 'Estimated Car1', 'Estimated Car2', 'Location', 'best');

%% Car Position Error
error_car1 = pos_car1 - real_car1;
error_car2 = pos_car2 - real_car2;
rmse_car1 = sqrt(sum(error_car1.^2, 1));
rmse_car2 = sqrt(sum(error_car2.^2, 1));

figure;
set(gcf, 'position', [0 0 500 1000]);
subplot(3,1,1); hold on;
title('Car Estimation Error');
plot(time, rmse_car1, 'r', 'LineWidth', 1.5);
plot(time, rmse_car2, 'b', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('RMSE [m]'); legend('Car1', 'Car2', 'Location', 'best');

fprintf('Average Car1 Position Estimation Error: %.4f m\n', mean(rmse_car1));
fprintf('Average Car2 Position Estimation Error: %.4f m\n', mean(rmse_car2));

%% Landmark Position Error
error_landmark1 = pos_landmark1 - real_landmarks;
error_landmark2 = pos_landmark2 - real_landmarks;
error_landmark1 = reshape(error_landmark1, [2*num_landmarks, timesteps]);
error_landmark2 = reshape(error_landmark2, [2*num_landmarks, timesteps]);
rmse_landmark1 = sqrt(sum(error_landmark1.^2, 1));
rmse_landmark2 = sqrt(sum(error_landmark2.^2, 1));

subplot(3,1,2); hold on;
title('Landmark Estimation Error');
plot(time, rmse_landmark1, 'r', 'LineWidth', 1.5);
plot(time, rmse_landmark2, 'b', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('RMSE [m]'); legend('Car1', 'Car2', 'Location', 'best');

fprintf('Average Landmark Estimation Error from Car1: %.4f m\n', mean(rmse_landmark1));
fprintf('Average Landmark Estimation Error from Car2: %.4f m\n', mean(rmse_landmark2));

if fusion_flag > 0
    error_landmark_SF = pos_landmark_SF - real_landmarks;
    error_landmark_SF = reshape(error_landmark_SF, [2*num_landmarks, timesteps]);
    rmse_landmark_SF = sqrt(sum(error_landmark_SF.^2, 1));
    plot(time, rmse_landmark_SF, 'g', 'LineWidth', 1.5);
    legend('Car1', 'Car2', 'Fusion', 'Location', 'best');
    fprintf('Average Landmark Estimation Error from Fusion: %.4f m\n', mean(rmse_landmark_SF));
end

%% Covariance Matrix (RMS of all entries in landmark covariances)
cov_landmark1 = reshape(cov_landmark1, [4*num_landmarks, timesteps]);
cov_landmark2 = reshape(cov_landmark2, [4*num_landmarks, timesteps]);
sum_cov_landmark1 = sqrt(sum(cov_landmark1.^2, 1));
sum_cov_landmark2 = sqrt(sum(cov_landmark2.^2, 1));

subplot(3,1,3); hold on;
title('Landmark Covariance (RMS of all covariance entries)');
plot(time, sum_cov_landmark1, 'r', 'LineWidth', 1.5);
plot(time, sum_cov_landmark2, 'b', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('RMS Covariance [m^2]'); legend('Car1', 'Car2', 'Location', 'best');

if fusion_flag > 0
    cov_landmark_SF = reshape(cov_landmark_SF, [4*num_landmarks, timesteps]);
    sum_cov_landmark_SF = sqrt(sum(cov_landmark_SF.^2, 1));
    plot(time, sum_cov_landmark_SF, 'g', 'LineWidth', 1.5);
    legend('Car1', 'Car2', 'Fusion', 'Location', 'best');
end