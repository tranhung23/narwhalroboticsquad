function [ x, P ] = measurementUpdate( x_old, P_old, u, zm, m, n, noise, measUpdateFunction )
% universal function for doing measurement update
% measUpdateFunction is specific to sensor
% m: dimension of measurement data
% n: dimension of state

% make noise cov matrix
R = diag(noise);

% generate sigma points from state
[Xi, W, Wc] = SigmaPoints(x_old, P_old, 1e-6);
% pass each sigma point thru measUpdateFunction
hXi = zeros(m, 2*n+1);
for k = 1:15
    hXi(:, k) = measUpdateFunction(Xi(:,k), noise);
end
% sum weighted resultant sigma points to get estimated measurement and
% measurement cov
[zp, Pz] = UT(hXi, W, Wc, R);

% calculate cross covariance
Pxz = zeros(n, m);
for k = 1:2*n+1
    Pxz = Pxz + Wc(k)*(Xi(:,k) - x_old)*(hXi(:,k) - zp)';
end

% Kalman gain
K = Pxz*inv(Pz);

% update state and state covariance
x = x_old + K*(zm-zp);
P = P_old - K*Pz*K';

end

