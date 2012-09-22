% Much of the code below is copyright Yi Cao (Learning the UKF)
function demo()
RandStream.setDefaultStream(RandStream.create('mrg32k3a','seed', 0)); % Set Fixed Seed for reproduction
n = 20;      %number of state
q = 0.1;    %std of process
r = 0.1;    %std of measurement
Q = q^2 * eye(n); % covariance of process
R = r^2 * eye(n); % covariance of measurement
s = zeros(n,1); % initial state
x = s + q * randn(n,1); % initial state with noise
P = eye(n);  % initial state covraiance
N = 1000;  % total dynamic steps
xV = zeros(n,N);   %estimate
sV = zeros(n,N);   %actual
ffunnoise = @(x,t)ffun(x,t) + q*randn(n,1);
hfunnoise = @(x,t)hfun(x,t) + r*randn(n,1);
% Select one of the kalman filters below, uncomment either aukf, ekf, the block until ukf = soukfwithkappa or the square root parts
% ukf = @aukf; % normal additive ukf
% ukf = @ekf; % ekf
% below is schmidt-orthogonal ukf
kappa = zeros(n , n + 1);
for j=1:n
     for i = 1:n+1
         if i == j + 1 
             kappa(j,i) = -((j+1)/j)^(-0.5);
         elseif i <= j
             kappa(j,i) = ((j+1)*j)^(-0.5);         
         end
     end
end
kappa = sqrt(n + 1) * kappa;
soukfwithkappa = @(ffun,xEst,PEst,hfun,z,Q,R, dt) soukf(ffun,xEst,PEst,hfun,z,Q,R, dt,kappa);
ukf = soukfwithkappa;
% square root
% Q = chol(Q); 
% R = chol(R);
% P = chol(P)';
%ukf = @asrukf;
%ukf = @asrssukf;
tic;
for k=1:N
    z = hfunnoise(s,k);            % measurments
    sV(:,k) = s;                             % save actual state
    [x, P] = ukf(@ffun,x,P,@hfun,z,Q,R,k);        % ukf
    xV(:,k) = x;                            % save estimate
    s = ffunnoise(s,k);                % update process
end
toc;
disp(toc);
rmse = zeros(n:1);
for k=1:n                                 % plot results
    subplot(n,1,k)
    plot(1:N, sV(k,:), '-', 1:N, xV(k,:), '--')
    rmse(k) = sqrt(sum((sV(k,:)-xV(k,:)).^2)/N);
end
disp(rmse);
end

function [y] = ffun(x, t)
beta = 0.5;                 % Autoregressive parameter.
y = 1 + sin(4e-2*pi*t) + beta*x;
end

function [y] = hfun(x, t)
if t<=300
    y = (x.^(2))/5;
else
    y = -2 + x/2;
end
end