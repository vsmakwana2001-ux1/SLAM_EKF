%% Car Trajectory
figure; hold on;
title('Car Trajectories');
plot(real_car1(1,:),real_car1(2,:),'r');
plot(real_car2(1,:),real_car2(2,:),'b');
plot(pos_car1(1,:),pos_car1(2,:),'r:');
plot(pos_car2(1,:),pos_car2(2,:),'b:');
grid on; xlabel('x (m)'); ylabel('y (m)');
axis([-30, 30, -30, 30]);
legend('True Car1','True Car2','Estimated Car1','Estimated Car2');

%% Car Position Error
error_car1 = pos_car1-real_car1;
error_car2 = pos_car2-real_car2;
rmse_car1 = sqrt(sum(error_car1.^2,1));
rmse_car2 = sqrt(sum(error_car2.^2,1));

figure; 
set(gcf,'position',[0 0 500 1000]);
subplot(311); hold on;
title('Car Estimation Error');
plot(time,rmse_car1,'r');
plot(time,rmse_car2,'b');
grid on; xlabel('Time (s)'); ylabel('RMSE [m]'); legend('Car1','Car2');

disp(['Average Car1 Position Estimation Error: ',num2str(mean(rmse_car1))]);
disp(['Average Car2 Position Estimation Error: ',num2str(mean(rmse_car2))]);

%% Landmark Position Error
error_landmark1 = pos_landmark1-real_landmarks;
error_landmark2 = pos_landmark2-real_landmarks;
error_landmark1 = reshape(error_landmark1(:),[2*num_landmarks,timesteps]);
error_landmark2 = reshape(error_landmark2(:),[2*num_landmarks,timesteps]);
rmse_landmark1 = sqrt(sum(error_landmark1.^2,1));
rmse_landmark2 = sqrt(sum(error_landmark2.^2,1));

subplot(312); hold on; 
title('Landmark Estimation Error');
plot(time,rmse_landmark1,'r');
plot(time,rmse_landmark2,'b');
grid on; xlabel('Time (s)'); ylabel('RMSE [m]'); legend('Car1','Car2');


disp(['Average Landmark Estimation Error from Car1: ',num2str(mean(rmse_landmark1))]);
disp(['Average Landmark Estimation Error from Car2: ',num2str(mean(rmse_landmark2))]);

if fusion_flag>0
    error_landmark_SF = pos_landmark_SF-real_landmarks;
    error_landmark_SF = reshape(error_landmark_SF(:),[2*num_landmarks,timesteps]);
    rmse_landmark_SF = sqrt(sum(error_landmark_SF.^2,1));
    plot(time,rmse_landmark_SF,'g');
    legend('Car1','Car2','Fusion');
    disp(['Average Landmark Estimation Error from Fusion: ',num2str(mean(rmse_landmark_SF))]);
end

%% Covariance Matrix
cov_landmark1 = reshape(cov_landmark1(:),[4*num_landmarks,timesteps]);
cov_landmark2 = reshape(cov_landmark2(:),[4*num_landmarks,timesteps]);
sum_cov_landmark1 = sqrt(sum(cov_landmark1.^2,1));
sum_cov_landmark2 = sqrt(sum(cov_landmark2.^2,1));

subplot(313); hold on; 
title('Landmark Covariance');
plot(time,sum_cov_landmark1,'r');
plot(time,sum_cov_landmark2,'b');
grid on; xlabel('Time (s)'); ylabel('Covariance [m^2]'); legend('Car1','Car2');

if fusion_flag>0
    cov_landmark_SF = reshape(cov_landmark_SF(:),[4*num_landmarks,timesteps]);
    sum_cov_landmark_SF = sqrt(sum(cov_landmark_SF.^2,1));
    plot(time,sum_cov_landmark_SF,'g');
    legend('Car1','Car2','Fusion');
end


