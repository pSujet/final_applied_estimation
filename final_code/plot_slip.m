%% Clear
clc;
close all;
clear all;

%% Plot Position
figure;
set(gcf,'Position',[100 50 600 600])
subplot(2,1,1)
grid on
title('Estimated Position with Slip detection','fontweight','bold','FontSize',16)
ylabel({'Pos';'[m]'},'FontSize',16)
hold on
plot(t,X_arr1(1,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr1(1,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr1(1,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);
legend('Ground truth','EKF', 'UKF','FontSize',14)

subplot(2,1,2)
grid on
title('Estimated Position without Slip detection','fontweight','bold','FontSize',16)
ylabel({'Pos';'[m]'},'FontSize',16)
xlabel('Time [s]','FontSize',16)
hold on
plot(t,X_arr2(1,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr2(1,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr2(1,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

legend('Ground truth','EKF', 'UKF','FontSize',14)

%% Plot Velocity
figure;
set(gcf,'Position',[100 50 600 600])
subplot(2,1,1)
grid on
title('Estimated Velocity with Slip detection','fontweight','bold','FontSize',16)
ylabel({'Vel';'[m/s]'},'FontSize',16)
hold on
plot(t,X_arr1(2,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr1(2,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr1(2,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);
legend('Ground truth','EKF', 'UKF','FontSize',14)

subplot(2,1,2)
grid on
title('Estimated Velocity without Slip detection','fontweight','bold','FontSize',16)
ylabel({'Vel';'[m/s]'},'FontSize',16)
xlabel('Time [s]','FontSize',16)
hold on
plot(t,X_arr2(2,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr2(2,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr2(2,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

legend('Ground truth','EKF', 'UKF','FontSize',14)

