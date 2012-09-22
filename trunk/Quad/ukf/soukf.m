function [xEst,PEst] = soukf(ffun,xEst,PEst,hfun,z,Q,R, dt, kappa)
%           xEst: "a priori" state estimate
%           PEst: "a priori" estimated state covariance
%           z: current measurement
%           Q: process noise covariance
%           R: measurement noise covariance
%           dt: TimeStep
%           kappa: sigma matrix
%           ffun: f-function
%           hfun: h-function
%Init
L = numel(xEst); %numer of states
m = numel(z);    %numer of measurements
w = 1/(L + 1); % only one weight
X = xEst(:,ones(1,L + 1)) + chol(PEst)' * kappa; % sigma

%Time-Update
Psik = zeros(L, L + 1);
for k = 1:L + 1
    Psik(:,k) = ffun(X(:,k),dt); % paper has it wrong here: it's not row vector, probably means column vector
end
wxEst = Psik * w(ones(1,L+1),1);
diffxTemp = Psik - wxEst(:,ones(1,L+1));
Pk = diffxTemp * diag(w) * diffxTemp' + Q;
Zeta = zeros(m, L + 1);
for k = 1:L + 1
    Zeta(:,k) = hfun(Psik(:,k),dt); % paper has it wrong here: it's not row vector, probably means column vector
end
wz = Zeta * w(ones(1,L+1),1);

%Measurement-Update
diffzTemp = Zeta - wz(:,ones(1,L+1));
Pxz = diffxTemp * diag(w) * diffzTemp';
Pz = diffzTemp * diag(w) * diffzTemp' + R;
R = chol(Pz);
Omega = (Pxz / R) / R';
PEst = Pk - Omega * Pz * Omega';
xEst = wxEst + Omega * (z - wz);
end