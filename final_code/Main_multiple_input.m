%% Final Project Applied Estimation 2021
% Sujet Phodapol 
% Egill Milan Gunnarsson

%% Clear
clc;
% clear all;
close all;

%% PIG parameters
m          = 427;                 % MC-PIG mass [kg]
D          = 16*25.4*1e-3;        % pipe diameter [m]
A_D        = pi/4*D^2;            % pipe area [m^2]
d          = 0.52*D;              % bypass diameter [m]
theta      = 45;                  % valve angle [degree]
k          = 3209.8*theta^-1.851; % pressure drop coefficient
rho        = 14.78759;            % fluid density [kg/m^3]
mu         = 0.15*rho;            % friction coefficient [N/((m/s)^2)]
A          = pi/4*(D^2-d^2);      % MC-PIG area [m^2]
Vel_Flow   = 4.39;                % fluid flow velocity [m/s]
f          = 500;                 % dry friction [N]

%% Initialization
%input choice:
        % 1: V_F = constant
        % 2: V_F = pure sine
        % 3: V_F = sine with interruption (original test input)
        % 4: V_F = step function
        % 5: V_F = square wave
input = 5;

% Experiment time setting
rng(100);         % random seed
t_step  = 0.1;
t_final = 50;
t       = 0:t_step:t_final;
time       = length(t);

% Data Array
X_arr5           = zeros(2,time);
Xhat_arr5        = zeros(2,time);
Z_arr			= zeros(2,time);	% Array for storing measurements
Error_arr       = zeros(2,time);
Error_ukf_arr   = zeros(2,time);
Vel_Flow_arr    = zeros(1,time);

% Initial Condtions
x               = [0;1];    % [Pos,Vel]
X_arr5(:,1)      = x;
xhat            = x;
Xhat_arr5(:,1)   = xhat;

% Sensor data
z				= [0;1];	% [Pos, Vel]
Z_arr(:,1)		= z;
v_meas_arr		= zeros(1,time);

% Sensor data with low pass
z2				= [0;1];	% [Pos, Vel]
Z_arr2(:,1)		= z2;
v_meas_arr2		= zeros(1,time);

% ==== EKF Parameters =====
sigma = eye(2)*100;
R = diag([0.1^2 0.1^2]);    % covariance of process
Q = diag([0.15^2 0.3^2]);   % covariance of measurement 

% Noise
% Ground Truth Noise
wStd_P = 0.1;       % Position process noise
wStd_V = 0.3;		% Velocity process noise
vStd = 0.1;         % Measurement noise

% System Model
Beta               = 1/(2*m)*k*rho*A*(D/d)^4; 
input_array        = Vel_Flow;

% ==== UKF Parameters =====
alpha = 1e-3;
kappa = 0;
beta = 2;
mu_ukf = x;
Sigma = eye(2);
RR = diag([0.1^2 0.1^2]);           % covariance of process
QQ = diag([0.23^2 0.3^2]);          % covariance of measurement  
g = @(x)[x(1)+x(2)*t_step;x(2)+(Beta*(input_array - x(2))^2 + rho*A/m*(input_array-x(2))*(x(2)+input_array/2)...
         -tanh(x(2))*f/m)*t_step];  % nonlinear state equation
h = @(x)[x(1);x(2)];                % measurement equation
Xhat_ukf_arr5(:,1)   = mu_ukf;

Error_arr(:,1)       = x-xhat;
Error_ukf_arr(:,1)   = x-mu_ukf;

%% Simulation
for i = 1:time-1
    switch input
        case 1
            input_array = Vel_Flow * 0.5;
        case 2
            input_array = Vel_Flow * sin(0.01*i) + Vel_Flow;
        case 3
            input_array = Vel_Flow * sin(0.01*i) + Vel_Flow;
            if i*t_step >= 10 && i*t_step<=15
                input_array  = 0;
            end
        case 4
            if i < time/2
                input_array = 0;
            else
                input_array = Vel_Flow;
            end
        case 5
            if mod(round(i*6/time),2) == 0
                input_array = 0;
            else
                input_array = Vel_Flow;
            end
    end

    % Ground truth
    Pos = X_arr5(1,i) + X_arr5(2,i)*t_step + wStd_P*randn(1,1);
    Vel = X_arr5(2,i) + (Beta*(input_array - X_arr5(2,i))^2 + rho*A/m*(input_array-Xhat_arr5(2,i))*(Xhat_arr5(2,i)+input_array/2) ...
          - tanh(X_arr5(2,i))*f/m)*t_step + wStd_V*randn(1,1);
    x = [Pos;Vel];
    X_arr5(:,i+1) = x;

	[Z_arr, v_meas_arr] = getMeas(X_arr5, Z_arr, i+1, v_meas_arr, vStd, t_step);
	z = Z_arr(:,i+1);
    
    [Z_arr2, v_meas_arr2] = getMeas_lowp(X_arr5, Z_arr2, i+1, v_meas_arr2, vStd, t_step);
	z2 = Z_arr2(:,i+1);
 
    % ====== EKF ======
    % Prediction
    Poshat = Xhat_arr5(1,i) + Xhat_arr5(2,i)*t_step;
    Velhat = Xhat_arr5(2,i) + (Beta*(input_array - Xhat_arr5(2,i))^2 + rho*A/m*(input_array-Xhat_arr5(2,i))*(Xhat_arr5(2,i)+input_array/2) ...
             - tanh(Xhat_arr5(2,i))*f/m)*t_step;
    mu = [Poshat;Velhat];
    G = [1 t_step; 0 1 + (2*Beta*(Xhat_arr5(2,i)-input_array) + rho*A/m*(input_array-2*Xhat_arr5(2,i)-input_array/2) ...
        - sech(Xhat_arr5(2,i))^2)*t_step];
    H = [1 0;0 1];
    sigma = G*sigma*G' + R;
    
    % Update
    K = sigma*H'/(H*sigma*H' + Q);
    zhat = mu;
    mu = mu + K*(z-zhat);
    sigma = (eye(2) - K*H)*sigma;
    
    Xhat_arr5(:,i+1) = mu;       
    
    % ====== UKF ======
    [mu_ukf, Sigma] = ukf(mu_ukf,Sigma,z,g,h,RR,QQ,alpha,kappa,beta);             
    Xhat_ukf_arr5(:,i+1) = mu_ukf;                           % save estimate
    
    Error_arr(:,i+1) = x-mu;
    Error_ukf_arr(:,i+1) = x-mu_ukf;
end

%% Plots Estimation
figure;
set(gcf,'Position',[100 50 600 600])
sgtitle('Estimated Position and Velocity using EKF and UKF','fontweight','bold','FontSize',16)
subplot(2,1,1)
grid on
ylabel('Position [m]','FontSize',16)
xlabel('Time [s]','FontSize',16)
hold on
plot(t,X_arr5(1,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr5(1,:),'Color', [0, 0.2, 0.9], 'linewidth', 1.5);
plot(t,Xhat_ukf_arr5(1,:),'Color', [0.1, 0.8, 0.1], 'linewidth', 1.5);

legend('Ground truth','EKF', 'UKF','FontSize',14)

subplot(2,1,2)
grid on
ylabel('Velocity [m/s]','FontSize',16)
xlabel('Time [s]','FontSize',16)
hold on
plot(t,X_arr5(2,:),'k', 'linewidth', 2);
plot(t,Xhat_arr5(2,:),'Color', [0, 0.2, 0.9],'linewidth', 1.5);
plot(t,Xhat_ukf_arr5(2,:),'Color', [0.1, 0.8, 0.1],'linewidth', 1.5);
legend('Ground truth','EKF', 'UKF','FontSize',14)


%% Plots Error
figure;
sgtitle('Error in position and velocity for EKF and UKF','fontweight','bold','FontSize',10)
subplot(2,1,1)
plot(t,Error_arr(1,:),'linewidth', 1.5);
grid on
ylabel('Position [m]','FontSize',8)
xlabel('Time [s]','FontSize',8)
hold on
plot(t,Error_ukf_arr(1,:),'linewidth', 1.5);
legend('EKF', 'UKF')

subplot(2,1,2)
plot(t,Error_arr(2,:),'linewidth', 1.5);
grid on
ylabel('Velocity [m/s]','FontSize',8)
xlabel('Time [s]','FontSize',8)
hold on
plot(t,Error_ukf_arr(2,:),'linewidth', 1.5);
legend('EKF', 'UKF')

% %% Plots Sensor position
% figure;
% set(gcf,'Position',[500 200 900 450])
% title('Comparing Estimated Position ','FontSize',20)
% grid on; hold on;
% ylabel('Position [m]','FontSize',16)
% xlabel('Time [s]','FontSize',16)
% 
% plot(t,Z_arr(1,:),'Color', [0.6, 0.6, 0.6], 'linewidth', 1.5)
% plot(t,X_arr(1,:),'k', 'linewidth', 2.5);
% plot(t,Z_arr2(1,:),'Color', [1, 0, 0],'linewidth', 2)
% legend('Raw','Actual', 'LP-filtrered','FontSize',14)
% 
% %% Plots Sensor velocity
% figure;
% set(gcf,'Position',[500 200 900 450])
% title('Comparing Estimated Velocity','FontSize',20)
% grid on; hold on;
% ylabel('Velocity [m/s]','FontSize',16)
% xlabel('Time [s]','FontSize',16)
% 
% plot(t,Z_arr(2,:),'Color', [0.6, 0.6, 0.6], 'linewidth', 1.5)
% plot(t,X_arr(2,:),'k', 'linewidth', 2.5);
% plot(t,Z_arr2(2,:),'Color', [1, 0, 0],'linewidth', 2)
% legend('Raw','Actual', 'LP-filtrered','FontSize',14)
% 
% %% Plots Sensor velocity EKF UKF
% figure;
% set(gcf,'Position',[500 200 900 450])
% title('Comparing velocity estimates with KF','FontSize',20)
% grid on; hold on;
% ylabel('Velocity [m/s]','FontSize',16)
% xlabel('Time [s]','FontSize',16)
% 
% plot(t,X_arr(2,:),'k', 'linewidth', 2.5);
% plot(t,Z_arr2(2,:),'Color', [1, 0, 0],'linewidth', 2)
% plot(t,Xhat_arr(2,:),'Color', [0, 0.2, 0.9],'linewidth', 2)
% plot(t,Xhat_ukf_arr(2,:),'Color', [0.1, 0.8, 0.1],'linewidth', 2)
% legend('Actual', 'LP-filtrered', 'EKF', 'UKF','FontSize',14)
% 
% %% Plots Input
% figure;
% input_array = zeros(1,time);
% for input = 1:5
%     subplot(5,1,input); grid on;
%     for i = 1:time
%         switch input
%             case 1
%                 input_array(i) = Vel_Flow * 0.5;
%             case 2
%                 input_array(i) = Vel_Flow * sin(0.01*i) + Vel_Flow;
%             case 3
%                 input_array(i) = Vel_Flow * sin(0.01*i) + Vel_Flow;
%                 if i*t_step >= 10 && i*t_step<=15
%                     input_array(i)  = 0;
%                 end
%             case 4
%                 if i < time/2
%                     input_array(i) = 0;
%                 else
%                     input_array(i) = Vel_Flow;
%                 end
%             case 5
%                 if mod(round(i*6/time),2) == 0
%                     input_array(i) = 0;
%                 else
%                     input_array(i) = Vel_Flow;
%                 end
%         end
%     end
% 
%     plot(t,input_array, 'linewidth', 1.5)
%     label_h = ylabel({'Vel';'[m/s]'},'FontSize',14);
%     grid on
% 
%     switch input
%         case 1
%             title('Constant','FontSize',14)
%         case 2
%             title('Sine','FontSize',14)
%         case 3
%             title('Sine with interruption','FontSize',14)
%         case 4
%             title('Step','FontSize',14)
%         case 5
%             title('Square wave','FontSize',14)
%             xlabel('Time [s]','FontSize',14)
%     end
% 
% end
% % sgtitle('Inputs (V_{Flow})','fontweight','bold','FontSize',16)
% 
% 
%% Display mean error
m_x_ekf     = mean(Error_arr(1,:).^2);
m_v_ekf     = mean(Error_arr(2,:).^2);
s_x_ekf    = std(Error_arr(1,:).^2);
s_v_ekf    = std(Error_arr(2,:).^2);

m_x_ukf     = mean(Error_ukf_arr(1,:).^2);
m_v_ukf     = mean(Error_ukf_arr(2,:).^2);
s_x_ukf    = std(Error_ukf_arr(1,:).^2);
s_v_ukf    = std(Error_ukf_arr(2,:).^2);

fprintf('Mean Square Error position (EKF): %.4f\n',m_x_ekf);
fprintf('Standard deviation position (EKF): %.4f\n',s_x_ekf);
fprintf('Mean Square Error position (UKF): %.4f\n',m_x_ukf);
fprintf('Standard deviation position (UKF): %.4f\n\n',s_x_ukf);

fprintf('Mean Square Error speed (EKF): %.4f\n',m_v_ekf);
fprintf('Standard deviation speed (EKF): %.4f\n',s_v_ekf);
fprintf('Mean Square Error speed (UKF): %.4f\n',m_v_ukf);
fprintf('Standard deviation speed (UKF): %.4f\n',s_v_ukf);






