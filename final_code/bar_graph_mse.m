%% Clear
clc;
clear all;
close all;

%% Plot
figure;
set(gcf,'Position',[100 50 900 450])
MSE = [0.264767 0.2404; 0.252433 0.243633; 0.253967 0.245233; 0.259 0.2435; 0.258233 0.2427];
SD = [0.061953 0.060421; 0.060186 0.058107; 0.061126 0.056926; 0.062159 0.065438; 0.06103 0.062155]*0.5;
inputs={'Constant'; 'Pure sine'; 'Sine w. int.'; 'Step'; 'Square wave'};
b = bar(MSE, 'grouped');
hold on
[ngroups,nbars] = size(MSE);
x = nan(nbars, ngroups);
for i = 1:nbars
    x(i,:) = b(i).XEndPoints;
end
errorbar(x',MSE,SD,'k','linestyle','none');
hold off
set(gca,'xticklabel',inputs)
xlabel(gca, 'Input signal','fontweight','bold','FontSize',16)
ylabel(gca, 'MSE Error','fontweight','bold','FontSize',16)
title(gca, 'Position MSE for EKF and UKF given various inputs', 'fontweight','bold','FontSize',20)
grid on
legend('EKF', 'UKF','FontSize',14)

figure;
set(gcf,'Position',[100 50 900 450])
MSE = [0.273733 0.322833; 0.318 0.391833; 0.3399 0.417133; 0.294233 0.324967; 0.294867 0.329267];
SD = [0.045571 0.042313; 0.062232 0.033547; 0.078057 0.046308; 0.053656 0.054339; 0.051444 0.037898]*0.5;
inputs={'Constant'; 'Pure sine'; 'Sine w. int.'; 'Step'; 'Square wave'};
b = bar(MSE, 'grouped');
hold on
[ngroups,nbars] = size(MSE);
x = nan(nbars, ngroups);
for i = 1:nbars
    x(i,:) = b(i).XEndPoints;
end
errorbar(x',MSE,SD,'k','linestyle','none');
hold off
set(gca,'xticklabel',inputs)
xlabel(gca, 'Input signal','fontweight','bold','FontSize',16)
ylabel(gca, 'MSE Error','fontweight','bold','FontSize',16)
title(gca, 'Velocity MSE for EKF and UKF given various inputs', 'fontweight','bold','FontSize',20)
grid on
legend('EKF', 'UKF','FontSize',14)