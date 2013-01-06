function [Xi W Wc] = SigmaPoints(xm, P, kappa)
%
%
n  = numel(xm);
Xi = zeros(n, 2*n+1);         % sigma points = col of Xi
W  = zeros(n, 1);

Xi(:, 1) = xm;
W(1)     = kappa / (n + kappa);
%Wc(1)     = kappa / (n + kappa) + (1 - (1e-3)^2 + 2);
Wc(1)     = kappa / (n + kappa);

%U = chol((n+kappa)*P);         % U'*U = (n+kappa)*P
[U,R] = qr((n+kappa)*P);
for k=1:n
  Xi(:, k+1) = xm + U(:, k);   % row of U
  W(k+1)     = 1 / (2*(n+kappa));
  Wc(k+1)     = 1 / (2*(n+kappa));
end

for k=1:n
  Xi(:, n+k+1) = xm - U(:, k);
  W(n+k+1)      = 1 / (2*(n+kappa));
  Wc(n+k+1)      = 1 / (2*(n+kappa));
end


xm


A = kappa*U;
Y = xm(:,ones(1,numel(xm)));
Xi = [xm Y+A Y-A]; 

W=[kappa/(n+kappa) 0.5/(n+kappa)+zeros(1,2*n)];
W

s = 0;
for k=1:2*n+1
    k
  s = s + Xi(:,k)*W(k);
end
s
end