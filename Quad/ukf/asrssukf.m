%broken
function [xEst,S]=asrssukf(ffun,xEst,S,hfun,z,Sq,Sr,dt)
%           xEst: "a priori" state estimate
%           PEst: "a priori" estimated state covariance
%           z: current measurement
%           Q: process noise covariance
%           R: measurement noise covariance
%Init
L=numel(xEst); %numer of states
m=numel(z);    %numer of measurements

[X Wm Wc] = sigma(xEst,S);
%Time-Update
xmin = zeros(L, 1);
Xstar = zeros(L, L+2);
for k = 1:L+2
    Xstar(:,k) = ffun(X(:,k),dt);      %3.210
    xmin = xmin + Wm(k) * Xstar(:,k);  %3.211
end
Xtemp = Xstar - xmin(:,ones(1,L+2));
[nu Smin] = qr([sqrt(Wc(2))*Xtemp(:,2:L+2) Sq]',0);    %3.212
sign = '+';
if(Wc(1) < 0)
    sign = '-';
end
Smin = cholupdate(Smin,  Xtemp(:,1), sign);    %3.213
%sigma 
[Xstar Wm Wc] = sigma(Xstar(:,1),Smin);
ymin = zeros(m, 1);
%Measurement-Update
Y = zeros(m, L+2);
for k = 1:L+2
    Y(:,k) = hfun(Xstar(:,k),dt);  %3.215
    ymin = ymin + Wm(k) * Y(:,k);  %3.216
end
Ytemp = Y - ymin(:,ones(1,L+2));
[nu Sy] = qr([sqrt(Wc(2))*Ytemp(:,2:L+2) Sr]',0);    % 3.217
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
alpha = 1e-5;
beta = 2;
kappa = 1e-5;
Wz = kappa;
Wi = (1 - Wz)/(L + 1);
Wm(1) = 1 + (Wz - 1) / (alpha^2);
Wm(2:L+2) = Wi / (alpha^2);
Wc = Wm;
Wc(1) = Wc(1) + (1 - alpha^2 + beta);
Z = zeros(L,L+2);
% Wm(2) = 1;
Z(2,2) = -1/sqrt(2 * Wm(2));
Z(2,3) = 1/sqrt(2 * Wm(2));
% Todo
for j = 3:L
    for i = 1:L+2
        if i == 1
            Z(j,i) = 0;
        elseif i <= j + 1;
            Z(j,i) = -1/sqrt(j*(j+1) * Wm(2));
        elseif i == j + 2
            Z(j,i) = j/sqrt(j*(j+1) * Wm(2));
        end
    end
end
X = xEst(:,ones(1,L+2)) + Sx * Z;  % 3.209;
end