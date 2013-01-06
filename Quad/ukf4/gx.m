
%-----------magnetometer-------------------
function yp = gx(x)
% acc_x, acc_y, acc_z, mag_x, mag_y, mag_z
%
yp = zeros(16,1);
q0 = x(1);
q1 = x(2);
q2 = x(3);
q3 = x(4);

yp(1) = 1 - 2*(q2*q2+q3*q3);
yp(2) =  2 * (q1*q2-q0*q3);
yp(3) =2 * (q0*q2+q1*q3);