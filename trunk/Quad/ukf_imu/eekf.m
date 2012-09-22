%Kaiwen Gu
%Extended Kalman IMU Code

%ACCLERATION REFERENCE:  x = 0.00593512389196179  
%                        y = 0.0587215182616255   
%                        z = 0.998233616389715
accrx = 0.00593;  
accry = 0.05872;   
accrz = 0.99823;
 

%GYRO REFERENCE:       
%                        x = 446.448215330603         
%                        y = 490.442949093037
%                        z = 407.889994148625
gyrorx = 447.4316;         
gyrory = 491.5522;
gyrorz = 442.2947;


MagCMatrix = [ 0.1715140830387  -0.0000859337545  -0.0052504060719;
                   0   0.1774386523288  -0.0019587190928;
                   0                   0   0.2079265640424];

MagMMatrix = [  -177.6259881867028
    15.2099
   8.1726];


 %generate refererence magnetomer vector
 ae = [-0.0213;
        0.0116;
        0.9997];
 me = [-0.7639;
      -0.0691;
      -0.7639];
 
 sh1 = ae;
 sh2 = cross(ae,me)/norm(cross(ae,me));
 sh3 = cross(sh1, sh2);

%491.5522  447.4316  442.2947
 
%SERIAL PORT
s1 = serial('com7');    % define serial port
s1.BaudRate=115200;               % define baud rate
set(s1, 'terminator', 'CR');    % define the terminator for println
fopen(s1);
%END SERIAL PORT
%DEFINES
%1 degree = 0.0174532925 radian
deg2rad = 0.0174532925;
iteration = 1;
                         
                             
 t0 = clock;
 
 GyroIntegrated = [0 0 0];
 quaternion = [0 0 0 1];

 % initial body offsets
bx = 0;
by = 0;
bz = 0;

% SET PROCESS VAR
process_var = 0.0000001 ;
acc_var = 2.0;
mag_var = 2.0;

% predicted estimate covariance
% %P = [process_var*0.00001, 0, 0, 0, 0, 0, 0;
%      0, process_var*0.00001, 0, 0, 0, 0, 0;
%      0, 0, process_var*0.00001, 0, 0, 0, 0;
%      0, 0, 0, process_var*0.00001, 0, 0, 0;
%      0, 0, 0, 0, process_var*0.00001, 0, 0; 
%      0, 0, 0, 0, 0, process_var*0.00001, 0;
%      0, 0, 0, 0, 0, 0, process_var*0.00001];
P = [0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0.2, 0, 0; 
     0, 0, 0, 0, 0, 0.2, 0;
     0, 0, 0, 0, 0, 0, 0.2];
 
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
     0, 0, 0, 0, 0.000001, 0, 0; 
     0, 0, 0, 0, 0, 0.000001, 0;
     0, 0, 0, 0, 0, 0, 0.000001];

% accelerometer covariance matrix
% Rpr = [acc_var*0.00001, 0, 0;
%        0, acc_var*0.00001, 0;
%        0, 0, acc_var*0.00001];

Rpr = [0.3, 0, 0;
       0, 0.3, 0;
       0, 0, 0.3];
% magnetometer covariance matrix
Rmag = [0.7, 0, 0;
       0, 0.7, 0;
       0, 0, 0.7];
   
   
% Quaternion matrix
state = [1 0 0 0 0 0 0]';

 u=udp('127.0.0.1',9091);
    fopen(u);
    
    z=[0 0 0];
    
while(true)
  
    dT = etime(clock,t0);
    t0 = clock;
   
    w=fscanf(s1, '%d %d %d %d %d %d %d %d %d');             % must define the input % d or %s, etc.
  
    
    Acc = [w(1) w(2) w(3)];
    Acc = Acc/norm(Acc);
    acc_x = Acc(1); %HACK! need to fix the quatarions...
    acc_y = Acc(2);
    acc_z = Acc(3);
    
    
    AccNorm =  [acc_x; acc_y; acc_z];
    
    
    %gyro data - this case is 9.1mV/degree, we are at 3.3V, 1024 steps,
    %each step is 3300/1024 = 3.22265625mV/step -> 0.354138049 degree/step
   
    %correct gyro with reference (need temp correction here), multiply by
    %step to get degrees, and then multiply by rad
    Gyro = ([w(4) w(5) w(6)] - [gyrorx gyrory gyrorz]) * 0.354138049 * 2.57 * deg2rad;
    gyro_x = -Gyro(1);
    gyro_y = -Gyro(2);
    gyro_z = -Gyro(3);
    
    
%     z = z + [w(4) w(5) w(6)];
%     z/iteration;
    
    
    wx = gyro_x - bx;
    wy = gyro_y - by;
    wz = gyro_z - bz;
    
    gyroz = 0;
    wz = 0;
    
    
    Mag = [w(7) w(8) w(9)]';

    
         
    MagCalibrate = MagCMatrix * (Mag - MagMMatrix);
    MagNorm = (MagCalibrate/norm(MagCalibrate));
    

    

    %GyroIntegrated = GyroIntegrated + Gyro * dT;
    
    %GyroIntegrated'
    
    
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
    Zm = [acc_x; acc_y; acc_z];
    
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
    
    
    
      r1 = Ze;
     r2 = cross(Ze,MagNorm)/norm(cross(Ze,MagNorm));
     r3 = cross(r1, r2);
    
     mm = [r1,r2,r3];
     mr = [sh1,sh2, sh3];
     ma = (mm*mr');
    
     dcm = ma;
     
    dcmstr =  [num2str(dcm(1,1), '%10.6f') ' '  num2str(dcm(1,2), '%10.6f') ' '  num2str(dcm(1,3), '%10.6f') ' ' num2str(dcm(2,1), '%10.6f') ' '  num2str(dcm(2,2), '%10.6f') ' '  num2str(dcm(2,3), '%10.6f') ' ' num2str(dcm(3,1), '%10.6f') ' '  num2str(dcm(3,2), '%10.6f') ' ' num2str(dcm(3,3), '%10.6f')];
   
    %fwrite(u, dcmstr);
     
    
    
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
