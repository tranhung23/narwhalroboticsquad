function [ x, P ] = magUpdate( x_old, P_old, magX, magY, magZ )
% update state and state covariance using magnetometer data
noise = zeros(3,1); % measurement variance
y = zeros(3,1); % measurement

% TODO: change hard coding here
noise(1) = 3.8226e-01;
noise(2) = noise(1);
noise(3) = noise(1);

norm = sqrt(magX*magX + magY*magY + magZ*magZ);
y(1) = magX/norm;
y(2) = magY/norm;
y(3) = magZ/norm;

[x, P] = measurementUpdate(x_old, P_old, 0, y, 3, 17, noise, @magMeasurementUpdate);
end

function y = magMeasurementUpdate(x, noise)
% calculate prediccted acc data from state
y = zeros(3,1);
q0 = x(1);
q1 = x(2);
q2 = x(3);
q3 = x(4);

% mag
y(1) = 1 - 2*(q2*q2+q3*q3) + noise(1);
y(2) =  2 * (q1*q2-q0*q3) + noise(2);
y(3) = 2 * (q0*q2+q1*q3) + noise(3);

end