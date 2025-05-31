%% Final Project Applied Estimation 2021
% Sujet Phodapol
% Egill Milan Gunnarsson

%% Clear
% clc;
clear all;
close all;

%% MC-PIG parameters
% Parameters for Rotary Valve MC-PIG 16 inch
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
input = 3;

% Experiment time setting
rng(1);         % random seed
t_step  = 0.1;
t_final = 100;
t       = 0:t_step:t_final;
time       = length(t);

% Data Array
X_arr           = zeros(3,time);
Xhat_arr        = zeros(3,time);
Z_arr			= zeros(2,time);	% Array for storing measurements
Error_arr       = zeros(3,time);
Error_ukf_arr   = zeros(3,time);
Vel_Flow_arr    = zeros(1,time);

% Initial Condtions
x               = [0;1;f];    % [Pos,Vel]
X_arr(:,1)      = x;
xhat            = [0;1;300];
Xhat_arr(:,1)   = xhat;

z				= [0;0];	% [Pos, Vel]
Z_arr(:,1)		= z;
v_meas_arr		= zeros(1,time);

z2				= [0;0;0];	% [Pos, Vel]
Z_arr2(:,1)		= z2;
v_meas_arr2		= zeros(1,time);

sigma = eye(3);
R = diag([0.1^2 0.1^2 10^2]);%diag([0.01^2 0.1^2]);
Q = diag([0.15^2 0.5^2]);
% Noise
% Ground Truth Noise
wStd_P = 0;       % Position process noise
wStd_V = 0;		% Velocity process noise
vStd = 0;         % Measurement noise

% System Model
Beta       = 1/(2*m)*k*rho*A*(D/d)^4; 
input_array        = Vel_Flow;


% ==== UKF Parameters =====
alpha = 1e-3;
kappa = 0;
beta = 2;
mu_ukf = [0;1;300];
Sigma = eye(3);
RR = diag([0.3^2 0.2^2 5^2]);           % covariance of process
QQ = diag([0.1^2 0.1^2]);          % covariance of measurement  
g = @(x)[x(1)+x(2)*t_step;x(2)+(Beta*(input_array - x(2))^2 + rho*A/m*(input_array-x(2))*(x(2)+input_array/2)...
         -tanh(x(2))*x(3)/m)*t_step;x(3)];  % nonlinear equation
h = @(x)[x(1);x(2)];                % measurement equation
Xhat_ukf_arr(:,1)   = mu_ukf;

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
    Pos = X_arr(1,i) + X_arr(2,i)*t_step + wStd_P*randn(1,1);
    Vel = X_arr(2,i) + (Beta*(input_array - X_arr(2,i))^2 + rho*A/m*(input_array-Xhat_arr(2,i))*(Xhat_arr(2,i)+input_array/2) ...
          - tanh(X_arr(2,i))*f/m)*t_step + wStd_V*randn(1,1);
    x = [Pos;Vel;f];
    X_arr(:,i+1) = x;

	[Z_arr, v_meas_arr] = getMeas(X_arr, Z_arr, i+1, v_meas_arr, vStd, t_step);
	z = Z_arr(:,i+1);
%     
%     [Z_arr2, v_meas_arr2] = getMeas2(X_arr, Z_arr2, i+1, v_meas_arr2, vStd, t_step);
% 	z2 = Z_arr2(:,i+1);
   
    % ====== EKF ======
    % Prediction
    Poshat = Xhat_arr(1,i) + Xhat_arr(2,i)*t_step;
    Velhat = Xhat_arr(2,i) + (Beta*(input_array - Xhat_arr(2,i))^2 + rho*A/m*(input_array-Xhat_arr(2,i))*(Xhat_arr(2,i)+input_array/2) ...
             - tanh(Xhat_arr(2,i))*Xhat_arr(3,i)/m)*t_step;
    fhat = Xhat_arr(3,i);
    mu = [Poshat;Velhat;fhat];
    G = [1 t_step 0; 0 1 + (2*Beta*(Xhat_arr(2,i)-input_array) + rho*A/m*(input_array-2*Xhat_arr(2,i)-input_array/2) ...
        - sech(Xhat_arr(2,i))^2*Xhat_arr(3,i)/m)*t_step -tanh(Xhat_arr(2,i))/m*t_step;0 0 1];
    H = [1 0 0;0 1 0];
    sigma = G*sigma*G' + R;
    
    % Update
    K = sigma*H'/(H*sigma*H' + Q);
    zhat = mu(1:2);
    mu = mu + K*(z-zhat);
    sigma = (eye(3) - K*H)*sigma;
    
    Xhat_arr(:,i+1) = mu;       
    
    % ====== UKF ======
%     [mu_ukf, Sigma] = ukf(mu_ukf,Sigma,z,g,h,RR,QQ,alpha,kappa,beta);             
%     Xhat_ukf_arr(:,i+1) = mu_ukf;                           % save estimate
%     
    Error_arr(:,i+1) = x-mu;
%     Error_ukf_arr(:,i+1) = x-mu_ukf;
end

%% Plots Parameter Estimation
figure;
set(gcf,'Position',[100 50 900 450])
hold on
grid on
plot(t,X_arr(3,:), 'k', 'linewidth', 2);
plot(t,Xhat_arr(3,:), 'linewidth', 2);
title('Parameter Estimation using EKF','FontSize',20)
ylabel('Friction Constant [N]','FontSize',16)
xlabel('Time [s]','FontSize',16)

legend('Actual','EKF','FontSize',18)
