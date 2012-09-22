% UKF (additive noise)
function [xEst,PEst]=aukf(ffun,xEst,PEst,hfun,z,Q,R, dt)
%           xEst: "a priori" state estimate
%           PEst: "a priori" estimated state covariance
%           z: current measurement
%           Q: process noise covariance
%           R: measurement noise covariance
%Init
L=numel(xEst); %numer of states
m=numel(z);    %numer of measurements
[X Wm Wc] = sigma(xEst,PEst);
%Time-Update
xmin = zeros(L, 1);
Xstar = zeros(L, 2 * L + 1);
for k = 1:2 * L + 1
    Xstar(:,k) = ffun(X(:,k),dt);      %3.171
    xmin = xmin + Wm(k) * Xstar(:,k);  %3.172
end
Xtemp = Xstar - xmin(:,ones(1,2 * L + 1)); %3.173
Pminx = Xtemp * diag(Wc) * Xtemp' + Q; %3.173
ymin = zeros(m, 1);
Y = zeros(m, 2 * L + 1);
for k = 1:2 * L + 1
    Y(:,k) = hfun(Xstar(:,k),dt);      %3.175
    ymin = ymin + Wm(k) * Y(:,k);  %3.176
end
%Measurement-Update
Ytemp = Y - ymin(:,ones(1,2 * L + 1)); %3.177
Py = Ytemp * diag(Wc) * Ytemp' + R; %3.177
Pxy = Xtemp * diag(Wc) * Ytemp'; % 3.178
K = Pxy / Py; %3.179
xEst = xmin + K * (z - ymin); %3.180
PEst = Pminx - K * Py * K'; %3-181
end

function [X Wm Wc] = sigma(xEst,PEst)
L=numel(xEst);
alpha = 1e-3;
beta = 2;
kappa = 0;
lambda = alpha^2 * (L + kappa) - L; % 3.11 in merwe phd thesis
Wm = [lambda 0.5 + zeros(1,2*L)]/(L + lambda); %3.12
Wc = Wm; %3.12
Wc(1) = Wc(1) + (1 - alpha^2 + beta); %3.12
Psqrtm = chol((L + lambda)*PEst)'; % 3.170
gamma = xEst(:,ones(1,L)); % 3.170
X = [xEst gamma+Psqrtm gamma-Psqrtm];  % 3.170;
end
