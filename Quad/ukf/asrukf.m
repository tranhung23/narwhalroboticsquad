function [xEst,S]=asrukf(ffun,xEst,S,hfun,z,Sq,Sr,dt)
%           xEst: "a priori" state estimate
%           PEst: "a priori" estimated state covariance
%           z: current measurement
%           Q: process noise covariance
%           R: measurement noise covariance
%Init
L=numel(xEst); %numer of states
m=numel(z);    %numer of measurements
%Sigma points
[X Wm Wc] = sigma(xEst,S);
%Time-Update
xmin = zeros(L, 1);
Xstar = zeros(L, 2 * L + 1);
for k = 1:2 * L + 1
    Xstar(:,k) = ffun(X(:,k),dt);      %3.210
    xmin = xmin + Wm(k) * Xstar(:,k);  %3.211
end
Xtemp = Xstar - xmin(:,ones(1,2 * L + 1));
[~, Smin] = qr([sqrt(Wc(2))*Xtemp(:,2:2*L+1) Sq]',0);    %3.212
sign = '+';
if(Wc(1) < 0)
    sign = '-';
end
Smin = cholupdate(Smin,  Xtemp(:,1), sign);    %3.213

%Measurement-Update
ymin = zeros(m, 1);
Y = zeros(m, 2 * L + 1);
for k = 1:2 * L + 1
    Y(:,k) = hfun(Xstar(:,k),dt);  %3.215
    ymin = ymin + Wm(k) * Y(:,k);  %3.216
end
Ytemp = Y - ymin(:,ones(1,2 * L + 1));
[~, Sy] = qr([sqrt(Wc(2))*Ytemp(:,2:2 * L + 1) Sr]',0);    % 3.217
sign = '+';
if(Wc(1) < 0)
    sign = '-';
end
Sy = cholupdate(Sy,  Ytemp(:,1), sign); % 3.218
Pxy = Xtemp * diag(Wc) * Ytemp'; % 3.219
K = (Pxy / Sy') / Sy; %3.220
xEst = xmin + K * (z - ymin); %3.221
U = K*Sy; % 3.22
for k = 1:m
    S = cholupdate(Smin, U(:,k), '-'); % 3.23
end
end

function [X Wm Wc] = sigma(xEst,Sx)
L=numel(xEst);
alpha = 1e-3;
beta = 2;
kappa = 0;
lambda = alpha^2 * (L + kappa) - L; % 3.11 in merwe phd thesis
Wm = [lambda 0.5 + zeros(1,2*L)]/(L + lambda); %3.12
Wc = Wm; %3.12
Wc(1) = Wc(1) + (1 - alpha^2 + beta); %3.12
Psqrtm = sqrt(L + lambda) * Sx; % 3.209
gamma = xEst(:,ones(1,L)); % 3.209
X = [xEst gamma+Psqrtm gamma-Psqrtm];  % 3.209;
end