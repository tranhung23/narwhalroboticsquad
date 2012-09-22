%Kevin Gu, Mita Yun
%Extended Kalman Matlab IMU Code

%ACCLERATION REFERENCE: (for analog acclemters)
AccAvg = [ 2075.03720930233          2076.99069767442        2076.99069767442];

%Conversion factor between readings and accleration
AccConversionFactor = [
    0
    0
    0];
                    

%GYRO REFERENCE: 
GyroReference = [ 1706.30753138075          1707.25941422594          1695.17154811715];


%Soft and hard iron calibration matrix for gyroscope.
MagCMatrix = [  0.001651600913498   0.000164539043519  -0.000175247708272
                   0   0.001767259192109   0.000056402873714
                   0                   0   0.001577958642801];

MagMMatrix = [ -31.2612766202469
  -10.7910288640105
  -227.2095173856283];


 %generate refererence magnetomer vector
 ae = [0;
        0;
        1];
 me = [ 0.355455195224084
        -0.927039047349604
        -0.119374238750857];


 deg2rad = 0.0174532925;
 
 %%
%SERIAL PORT items
s1 = serial('com30');    % define serial port
s1.BaudRate=115200;               % define baud rate
set(s1, 'terminator', 'CR');    % define the terminator for println
fopen(s1);
%END SERIAL PORT

iteration = 1;
t0 = clock;

%%
%Debug Variables
 GyroIntegrated = [0 0 0];
 quaternion = [0 0 0 1];

 
 %%
 %Kalman Variables
 % initial body offsets
bx = 0;
by = 0;
bz = 0;

% SET PROCESS VAR
process_var = 0.000001 ;
acc_var = 0.2;
mag_var = 0.2;

% predicted estimate covariance
% %P = [process_var*0.00001, 0, 0, 0, 0, 0, 0;
%      0, process_var*0.00001, 0, 0, 0, 0, 0;
%      0, 0, process_var*0.00001, 0, 0, 0, 0;
%      0, 0, 0, process_var*0.00001, 0, 0, 0;
%      0, 0, 0, 0, process_var*0.00001, 0, 0; 
%      0, 0, 0, 0, 0, process_var*0.00001, 0;
%      0, 0, 0, 0, 0, 0, process_var*0.00001];
P = [1, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0.02, 0, 0; 
     0, 0, 0, 0, 0, 0.02, 0;
     0, 0, 0, 0, 0, 0, 0.02];
 
% Innovation (or residual) covariance
% Q = [process_var*0.00001, 0, 0, 0, 0, 0, 0;
%      0, process_var*0.00001, 0, 0, 0, 0, 0;
%      0, 0, process_var*0.00001, 0, 0, 0, 0;
%      0, 0, 0, process_var*0.00001, 0, 0, 0;
%      0, 0, 0, 0, process_var*0.00001, 0, 0; 
%      0, 0, 0, 0, 0, process_var*0.00001, 0;
%      0, 0, 0, 0, 0, 0, process_var*0.00001];

Q = [0.000001, 0, 0, 0, 0, 0, 0;
     0, 0.000001, 0, 0, 0, 0, 0;
     0, 0, 0.000001, 0, 0, 0, 0;
     0, 0, 0, 0.000001, 0, 0, 0;
     0, 0, 0, 0, 0.0001, 0, 0; 
     0, 0, 0, 0, 0, 0.0001, 0;
     0, 0, 0, 0, 0, 0, 0.0001];

% accelerometer covariance matrix
% Rpr = [acc_var*0.00001, 0, 0;
%        0, acc_var*0.00001, 0;
%        0, 0, acc_var*0.00001];

Rpr = [0.01, 0, 0;
       0, 0.01, 0;
       0, 0, 0.01];
% magnetometer covariance matrix
Rmag = [0.3, 0, 0;
       0, 0.3, 0;
       0, 0, 0.3];
   
   
% Quaternion matrix
state = [1 0 0 0 0 0 0]';

%%
%Starting..
 u=udp('127.0.0.1',9091);
 fopen(u);
%%
while(true)
  
    dT = etime(clock,t0);
    t0 = clock;
   
    %Gyro, Acclermeter, Magnotometer
    w=fscanf(s1, '%d %d %d %d %d %d %d %d %d');             % must define the input % d or %s, etc.
  
    %Acclermeter, raw value - bias;
    Acc = [w(4) w(5) w(6)] - AccAvg;
    AccNorm = (Acc/norm(Acc))';
  
    %Gyroscope, we are using a 12 bit ADC, so 4096 steps at 3.3v.
    %4096/3300 -> each LSB is 1.2412 milivolts, approx 2 mv per degree/s
    %(1.2412 * RAWGYRO)/2 = degree/second
    Gyro = (([w(1) w(2) w(3)] - GyroReference) * ((3300/4096)/2) * deg2rad)

    %Gyro
%     z = z + [w(4) w(5) w(6)];
%     z/iteration;
    
    
    wx = -Gyro(2);
    wy = Gyro(1);
    wz = Gyro(3);
    

    Mag = [w(7) w(8) w(9)]';
    MagCalibrate = MagCMatrix * (Mag - MagMMatrix);
    MagNorm = (MagCalibrate/norm(MagCalibrate));
  
    MagNorm = [MagNorm(1); MagNorm(3); MagNorm(2)];
    
    %state
    
    %% PREDICTION
    
    q0 = state(1);
    q1 = state(2);
    q2 = state(3);
    q3 = state(4);
    g4 = state(5);
    g5 = state(6);
    g6 = state(7);
    %Gyro Rotation into quatarion matrix    
    %state equation
    q0 = q0 + (-1/2 * wx * q1 - 1/2 * wy * q2 - 1/2 * wz * q3) * dT;
    q1 = q1 + (1/2 * wx * q0 - 1/2 * wy * q3 + 1/2 * wz * q2) * dT;
    q2 = q2 + (1/2 * wx * q3 + 1/2 * wy * q0 - 1/2 * wz * q1) * dT;
    q3 = q3 + (-1/2 * wx * q2 + 1/2 * wy * q1 + 1/2 * wz * q0) * dT;
    g4 = g4 + 0;
    g5 = g5 + 0;
    g6 = g6 + 0;
    %xk: state quaternion
    quaternion = quatnormalize([q0 q1 q2 q3]);
    
    state(1:4) = quaternion;
    state(5:7) = [g4, g5, g6];
    
    %quaternion';
    %phi = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2))*180/3.14159;
    %theta = asin(2*(q0*q2 - q1*q3))*180/3.14159;
    %psi = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*180/3.14159;
    %dcm = quat2dcm(quaternion);
    
    %[yaw, pitch, roll] = q2a(q0,q1,q2,q3);
    %[yaw, pitch, roll] = quat2angle(quaternion);
    %([yaw, pitch, roll] * 180/pi)'
    
    %v = [phi; theta; psi;]

    %assemble jacobian matrix
    F = [1,         -1/2*wx*dT, -1/2*wy*dT, -1/2*wz*dT,  1/2*q1*dT,  1/2*q2*dT,  1/2*q3*dT;
         1/2*wx*dT,  1,          1/2*wz*dT, -1/2*wy*dT, -1/2*q0*dT,  1/2*q3*dT, -1/2*q2*dT;
         1/2*wy*dT, -1/2*wz*dT,  1,          1/2*wx*dT,  1/2*q3*dT, -1/2*q0*dT,  1/2*q1*dT;
         1/2*wz*dT,  1/2*wy*dT, -1/2*wx*dT,  1,          1/2*q2*dT, -1/2*q1*dT, -1/2*q0*dT;
         0,          0,          0,          0,          1,          0,          0        ;
         0,          0,          0,          0,          0,          1,          0        ;
         0,          0,          0,          0,          0,          0,          1        ];
     
     % predict covariance matrix from the last one
    P = F*P*(F') + Q;
    
    
    %% CORRECTION
    % normalized accelerometer measurement matrix
    Zm = AccNorm;
    
    % predicted z-earth vector
    Ze = [2*(q1*q3 - q0*q2);
          2*(q2*q3 + q0*q1);
          1 - 2*(q1*q1 + q2*q2)];
      
    % roll-pitch estimation error
    Epr = Zm - Ze;
    
    %Epr
    
    % roll-pitch observation matrix
    Hpr = [-2*q2,   2*q3,   -2*q0,  2*q1,   0,  0,  0;
            2*q1,   2*q0,    2*q3,  2*q2,   0,  0,  0;
            0,      -4*q1,  -4*q2,  0,      0,  0,  0];
        
    % roll-pitch estimation error covariance matrix
    Ppr = Hpr * P * (Hpr') + Rpr;
    
    % roll-pitch kalman gain
    Kpr = P * (Hpr') / Ppr;
    
    % update system state
    state = state + Kpr*Epr;
      
    % update system state covariance matrix
    P = P - Kpr * Hpr * P;
    
    
      %% magnetometer updates
    % compute predicted X body over ground in earth xy plane using the
    % already filtered quaternion
    q0 = state(1);
    q1 = state(2);
    q2 = state(3);
    q3 = state(4);
    
     sh1 = ae;
     sh2 = cross(ae,me)/norm(cross(ae,me));
     sh3 = cross(sh1, sh2);
    
    
     r1 = Ze;
     r2 = cross(Ze,MagNorm)/norm(cross(Ze,MagNorm));
     r3 = cross(r1, r2);
    
     mm = [r1,r2,r3];
     mr = [sh1,sh2, sh3];
     ma = (mm*mr');
    
     %%
     %alignment calibration code
     %dcm = ma;
     %dcmstr =  [num2str(dcm(1,1), '%10.6f') ' '  num2str(dcm(1,2), '%10.6f') ' '  num2str(dcm(1,3), '%10.6f') ' ' num2str(dcm(2,1), '%10.6f') ' '  num2str(dcm(2,2), '%10.6f') ' '  num2str(dcm(2,3), '%10.6f') ' ' num2str(dcm(3,1), '%10.6f') ' '  num2str(dcm(3,2), '%10.6f') ' ' num2str(dcm(3,3), '%10.6f')];
     %fwrite(u, dcmstr);
     %continue;
    
    
    Xog = [1-2*(q2*q2 + q3*q3);
           2*(q1*q2 - q0*q3);
           2*(q0*q2 + q1*q3)];
       
    %VERmag INTERSTING...
    Xogmag = [ma(1,1); ma(2,1); ma(3,1)];
    
    Ey = Xogmag - Xog;
    
    Hy = [    0,     0, -4*q2, -4*q3, 0, 0, 0;
            -2*q3,    2*q2,    2*q1,   -2*q0, 0, 0, 0;
          	  2*q2, 2*q3, 2*q0,     2*q1, 0, 0, 0];
    
    % yaw estimation error covariance matrix
    Py = Hy * P * (Hy') + Rmag;
    
    % yaw kalman gain
    Ky = P * (Hy') / Py;
    
    % update the system state
    state = state + Ky * Ey;
    
    % update system state covariance matrix
    P = P - Ky*Hy*P;
    
    %% END MAGNETOMETER
    
    %end kalman
    
    %%
    
    
    [yaw, pitch, roll] = quat2angle(state(1:4)');
    ([yaw, pitch, roll] * 180/pi)';
    
    
    
    
    
    dcm = quat2dcm(state(1:4)');
    
    dcmstr =  [num2str(dcm(1,1), '%10.6f') ' '  num2str(dcm(1,2), '%10.6f') ' '  num2str(dcm(1,3), '%10.6f') ' ' num2str(dcm(2,1), '%10.6f') ' '  num2str(dcm(2,2), '%10.6f') ' '  num2str(dcm(2,3), '%10.6f') ' ' num2str(dcm(3,1), '%10.6f') ' '  num2str(dcm(3,2), '%10.6f') ' ' num2str(dcm(3,3), '%10.6f')];
   
     fwrite(u, dcmstr);

    
    
    %%
    iteration = iteration + 1;
  
end
fclose(u);
try
catch exception
    fclose(s1);                 % always, always want to close s1
    throw (exception);
end             
