function [mu,Sigma,T] = ut(sigma_point,w_m,w_c,trans,Cov,n)
k = size(sigma_point,2);
mu = zeros(n,1);
sigma_trans = zeros(n,k);
for i = 1:k
% == Non-linear Transform ==
    sigma_trans(:,i) = trans(sigma_point(:,i));  
end

% == Recover Gaussian ==
% compute mu
for i = 1:k
    mu = mu + w_m(i)*sigma_trans(:,i);       
end
% compute sigma
T = sigma_trans - repmat(mu,1,k);
Sigma = T*diag(w_c)*T' + Cov;
end