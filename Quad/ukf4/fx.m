
%------------------------------
function out = fx(in, noise, u, dt, GRAVITY)
tmp = zeros(3);
acc = zeros(3);
rate = zeros(3);
mat3x3 = zeros(3,3);
out = in;


%% QUATERNION Gyroscope Rotation
% rate = rate + bias + noise
rate(1) = (u(4) + noise(5,5)) * dt;
rate(2) = (u(5) + noise(6,6)) * dt;
rate(3) = (u(6) + noise(7,7)) * dt;
% rate(1) = u(4);
% rate(2) = u(5);
% rate(3) = u(6);
% rotate
q_in = [in(1); in(2); in(3); in(4)];
q = ukfRotateQuat(q_in, rate, dt);
%q = q / norm(q);
% q = eulerRotateQuat(q_in, rate, dt);
out(1) = q(1);
out(2) = q(2);
out(3) = q(3);
out(4) = q(4);

end

%% function that rotates quaternion by gyro+noise
% @params in: input quaternion
% @params rate: gyro data
% @return original quaternion rotated by gyro data
function out = ukfRotateQuat(in, rate, dt)
out = zeros(4,1);

q0 = in(1);
q1 = in(2);
q2 = in(3);
q3 = in(4);

wx = rate(1);
wy = rate(2);
wz = rate(3);

s = sqrt(wx^2 + wy^2 + wz^2)*0.5;
t = -(0.5 * sin(s) / s);

wx = wx*t;
wy = wy*t;
wz = wz*t;

% create Lagrange factor to control quat's numerical integration errors
qMag = q0^2 + q1^2 + q2^2 + q3^2;
lg = cos(s) + (1-qMag^2) * dt * dt;

out(1) =  lg * q0 + wx * q1 + wy * q2 + wz * q3;
out(2) = -wx * q0 + lg * q1 - wz * q2 + wy * q3;
out(3) = -wy * q0 + wz * q1 + lg * q2 - wx * q3;
out(4) = -wz * q0 - wy * q1 + wx * q2 + lg * q3;
end

%% function that updates quaternion
% @params in: input quaternion
% @params rate: gyro data
% @return original quaternion rotated by gyro data
function out = eulerRotateQuat(in, rate, dt)
q0 = in(1);
q1 = in(2);
q2 = in(3);
q3 = in(4);

wx = rate(1);
wy = rate(2);
wz = rate(3);

 q0 = q0 + (-1/2 * wx * q1 - 1/2 * wy * q2 - 1/2 * wz * q3) * dt;
 q1 = q1 + (1/2 * wx * q0 - 1/2 * wy * q3 + 1/2 * wz * q2) * dt;
 q2 = q2 + (1/2 * wx * q3 + 1/2 * wy * q0 - 1/2 * wz * q1) * dt;
 q3 = q3 + (-1/2 * wx * q2 + 1/2 * wy * q1 + 1/2 * wz * q0) * dt;
 
 quaternion = [q0; q1; q2; q3];
out = quaternion / norm(quaternion);
end
%% function that converts quaternion to dcm
% @params in: quaternion
% @return 3x3 matrix
function out = ukfQuat2Matrix(in, normalize)
out = zeros(3,3);
sqw = in(1)^2;
sqx = in(2)^2;
sqy = in(3)^2;
sqz = in(4)^2;
invs = 1;

if(normalize)
    invs = 1/(sqw + sqx + sqy + sqz);
end

out(1,1) = (sqx - sqy - sqz + sqw) * invs;
out(2,2) = (-sqx + sqy - sqz + sqw) * invs;
out(3,3) = (-sqx - sqy + sqz + sqw) * invs;

tmp1 = in(2) * in(3);
tmp2 = in(4) * in(1);
out(2,1) = 2 * (tmp1 + tmp2) * invs;
out(1,2) = 2 * (tmp1 - tmp2) * invs;

tmp1 = in(2) * in(4);
tmp2 = in(3) * in(1);
out(3,1) = 2 * (tmp1 - tmp2) * invs;
out(1,3) = 2 * (tmp1 + tmp2) * invs;

tmp1 = in(3) * in(4);
tmp2 = in(2) * in(1);
out(3,2) = 2 * (tmp1 + tmp2) * invs;
out(2,3) = 2 * (tmp1 - tmp2) * invs;
end

%% function that rotates a vector by rotation matrix
% @params in: vector to be rotated: 3x1
% @params m: rotation matrix
function out = ukfRotateVecByMatrix(in, m)
out = in;
out(1) = m(1,1) * in(1) + m(1,2) * in(2) * m(1,3) * in(3);
out(2) = m(2,1) * in(1) + m(2,2) * in(2) * m(2,3) * in(3);
out(3) = m(3,1) * in(1) + m(3,2) * in(2) * m(3,3) * in(3);
end