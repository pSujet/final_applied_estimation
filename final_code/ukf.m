function [mu,Sigma] = ukf(mu,Sigma,z,g,h,R,Q,alpha,kappa,beta)
% == prediction step ==
% process
[sigma_point,w_m,w_c] = generate_sigma_point(mu,Sigma,alpha,kappa,beta);
[mu_bar,Sigma_bar,T] = ut(sigma_point,w_m,w_c,g,R,2); %2
% measurement
[sigma_point_mea,w_m,w_c] = generate_sigma_point(mu_bar,Sigma_bar,alpha,kappa,beta);
[z_hat,St,Tz] = ut(sigma_point_mea,w_m,w_c,h,Q,2); %2

% == correction step ==
T_bar = sigma_point_mea - repmat(mu,1,size(sigma_point_mea,2)); %2
% update
Sigma_xz = T_bar*diag(w_c)*Tz';
K = Sigma_xz/St;            % Kalman gain
mu = mu_bar + K*(z-z_hat);
Sigma = Sigma_bar - K*St*K';
end