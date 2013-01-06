function [ x, P ] = accUpdate( x_old, P_old, R, accX, accY, accZ, m, n )
% update state and state covariance using acc measurement data
noise = zeros(3,1); % measurement variance
y = zeros(3,1); % measurement
GRAVITY = 9.8;

norm = sqrt(accX*accX + accY*accY + accZ*accZ);
y(1) = accX/norm;
y(2) = accY/norm;
y(3) = accZ/norm;

% % TODO: change hard coding here
% noise(1) = 9.5468e-05 + abs(GRAVITY - norm) * 1.8705e-02;
% noise(2) = noise(1);
% noise(3) = noise(1);
noise(1) = R(1);
noise(2) = R(2);
noise(3) = R(3);

[x, P] = measurementUpdate(x_old, P_old, 0, y, m, n, noise, @accMeasurementUpdate);
end

function y = accMeasurementUpdate(x, noise)
% calculate prediccted acc data from state
y = zeros(3,1);
q0 = x(1);
q1 = x(2);
q2 = x(3);
q3 = x(4);

% acc
y(1) = 2*(q1*q3 - q0*q2) + noise(1);
y(2) =  2*(q2*q3 + q0*q1) + noise(2);
y(3) = 1 - 2*(q1*q1 + q2*q2) + noise(3);

end