function [sigma_point,w_m,w_c] = generate_sigma_point(mu,Sigma,alpha,kappa,beta)
% == Choosing sigma points ==
n = length(mu);
lambda = alpha^2*(n+kappa)-n;
X = mu;
X_shift = sqrt(n+lambda)*chol(Sigma)';
Xi = repmat(X,1,n);
sigma_point = [X Xi+X_shift Xi-X_shift];

% == Weight sigma points ==
w_m = zeros(1,2*n+1);
w_c = zeros(1,2*n+1);
% compute weight
w_m(1) = lambda/(n+lambda);
w_c(1) = lambda/(n+lambda) + (1 - alpha^2 + beta);
w_m(2:end) = 1/(2*(n + lambda));
w_c(2:end) = w_m(2:end);
end