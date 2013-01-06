function [acc_norm, gyro_fixed, mag_norm, prevAccNorm, prevGyroFixed, prevMagNorm] = readSensor(line, acc_ref, gyro_ref, mag_C, mag_M, prevAccNorm, prevGyroFixed, prevMagNorm)
[m n] = size(line);
% Read/process sensor data and outputs normalized/fixed acc, gyro, mag data
    deg2rad = 0.0174532925;
    
    % Acclermeter, raw value - bias;
    Acc = [line(1), line(2), line(3)];
    acc_norm = (Acc / norm(Acc))';
    
    % Gyroscope, we are using a 12 bit ADC, so 4096 steps at 3.3v.
    % 4096/3300 -> each LSB is 1.2412 milivolts, approx 2 mv per degree/s
    % (1.2412 * RAWGYRO)/2 = degree/second
%     Gyro = (([line(1) line(2) line(3)] - gyro_ref) * ((3300/4096)/2) * deg2rad);
%     wx = -Gyro(2);
%     wy = Gyro(1);
%     wz = Gyro(3);
%     gyro_fixed = [wx, wy, wz]';
    gyro_fixed = [line(4), line(5), line(6)]';
    
    % Magnetomer: using C, M matrices
    Mag = [line(7); line(8); line(9)];
    MagCalibrate = mag_C * (Mag - mag_M);
    mag_norm = (MagCalibrate / norm(MagCalibrate));
    mag_norm = [mag_norm(1); mag_norm(3); mag_norm(2)];
end