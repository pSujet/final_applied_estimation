%% Clear
clc;
close all;
clear all;

%% Plot Position
figure;
set(gcf,'Position',[100 50 600 600])
sgtitle('Estimated Position using EKF and UKF','fontweight','bold','FontSize',16)
subplot(5,1,1)
grid on
ylabel({'Pos';'[m]'},'FontSize',16)
hold on
plot(t,X_arr1(1,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr1(1,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr1(1,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

subplot(5,1,2)
grid on
ylabel({'Pos';'[m]'},'FontSize',16)
hold on
plot(t,X_arr2(1,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr2(1,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr2(1,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

subplot(5,1,3)
grid on
ylabel({'Pos';'[m]'},'FontSize',16)
hold on
plot(t,X_arr3(1,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr3(1,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr3(1,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

subplot(5,1,4)
grid on
ylabel({'Pos';'[m]'},'FontSize',16)
hold on
plot(t,X_arr4(1,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr4(1,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr4(1,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

subplot(5,1,5)
grid on
ylabel({'Pos';'[m]'},'FontSize',16)
xlabel('Time [s]','FontSize',16)
hold on
plot(t,X_arr5(1,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr5(1,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr5(1,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);
legend('Ground truth','EKF', 'UKF','FontSize',14)

%% Plot Velocity
figure;
set(gcf,'Position',[100 50 600 600])
sgtitle('Estimated Velocity using EKF and UKF','fontweight','bold','FontSize',16)
subplot(5,1,1)
grid on
ylabel({'Vel';'[m/s]'},'FontSize',16)
hold on
plot(t,X_arr1(2,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr1(2,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr1(2,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

subplot(5,1,2)
grid on
ylabel({'Vel';'[m/s]'},'FontSize',16)
hold on
plot(t,X_arr2(2,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr2(2,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr2(2,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

subplot(5,1,3)
grid on
ylabel({'Vel';'[m/s]'},'FontSize',16)
hold on
plot(t,X_arr3(2,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr3(2,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr3(2,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

subplot(5,1,4)
grid on
ylabel({'Vel';'[m/s]'},'FontSize',16)
hold on
plot(t,X_arr4(2,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr4(2,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr4(2,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

subplot(5,1,5)
grid on
ylabel({'Vel';'[m/s]'},'FontSize',16)
xlabel('Time [s]','FontSize',16)
hold on
plot(t,X_arr5(2,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr5(2,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr5(2,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);
legend('Ground truth','EKF', 'UKF','FontSize',14)
