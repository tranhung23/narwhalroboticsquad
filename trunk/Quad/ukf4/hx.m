
%------------------------------
function yp = hx(x)
% acc_x, acc_y, acc_z, mag_x, mag_y, mag_z
%
yp = zeros(16,1);
q0 = x(1);
q1 = x(2);
q2 = x(3);
q3 = x(4);

% acc
yp(1) = 2*(q1*q3 - q0*q2);
yp(2) =  2*(q2*q3 + q0*q1);
yp(3) = 1 - 2*(q1*q1 + q2*q2);

%mag
% yp(4) = 1 - 2*(q2*q2+q3*q3);
% yp(5) = 2 * (q1*q2-q0*q3);
% yp(6) = 2 * (q0*q2+q1*q3);
% yp = [2*(q1*q3 - q0*q2);
%       2*(q2*q3 + q0*q1);
%       1 - 2*(q1*q1 + q2*q2)];