gyrocali500 = [ -0.0938;
                -0.0015;
                0.0323];

MagCMatrix = [0   1   0
              -1  0   0
             0  0   1];

MagMMatrix =   [ 67.0335
  -55.9988
  131.3037];


 
%SERIAL PORT
s1 = serial('com15');    % define serial port
s1.BaudRate=115200;               % define baud rate
set(s1, 'terminator', 'LF');    % define the terminator for println
fopen(s1);
%END SERIAL PORT

deg2rad = 0.0174532925;
iteration = 1;
                         
                             
 t0 = clock;

 
% SET PROCESS VAR

% predicted estimate covariance
P = [0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0.2, 0, 0; 
     0, 0, 0, 0, 0, 0.2, 0;
     0, 0, 0, 0, 0, 0, 0.2];
 
% Innovation (or residual) covariance

Q = [0.00001, 0, 0, 0, 0, 0, 0;
     0, 0.00001, 0, 0, 0, 0, 0;
     0, 0, 0.00001, 0, 0, 0, 0;
     0, 0, 0, 0.00001, 0, 0, 0;
     0, 0, 0, 0, 0.00001, 0, 0; 
     0, 0, 0, 0, 0, 0.00001, 0;
     0, 0, 0, 0, 0, 0, 0.00001];

Racc = [0.3, 0, 0;
       0, 0.3, 0;
       0, 0, 0.3];
   
% magnetometer covariance matrix
Rmag = [0.3, 0, 0, 0;
       0, 0.3, 0, 0;
       0, 0, 0.3, 0
       0, 0, 0, 0.3];
   
   
% Quaternion matrix
state = [1 0 0 0 0 0 0]';

 u=udp('127.0.0.1',9091);
 fopen(u);
 
 %generate refererence magnetomer vector
 ae = [-0.0213;
        0.0116;
        0.9997];
 me = [ -0.9209
   -0.2977
    0.2515];
 
 sh1 = ae;
 sh2 = cross(ae,me)/norm(cross(ae,me));
 sh3 = cross(sh1, sh2);
 z=[0;0;0];
 while(true)
  
    dT = etime(clock,t0);
    t0 = clock;
   
    w=fscanf(s1, '%d %d %d %d %d %d %d %d %d');              % must define the input % d or %s, etc.
  
    Acc = [w(1);
           w(2);
           w(3)];
    Acc = Acc/norm(Acc);
    
    
    
    acc_x = Acc(1); 
    acc_y = Acc(2);
    acc_z = Acc(3);
    
    AccNorm = [acc_x;
               acc_y;
               acc_z];
    %gyro data - this case is 9.1mV/degree, we are at 3.3V, 1024 steps,
    %each step is 3300/1024 = 3.22265625mV/step -> 0.354138049 degree/step
   
    %correct gyro with reference (need temp correction here), multiply by
    %step to get degrees, and then multiply by rad
    Gyro = ([w(4); w(5); w(6)] - gyrocali500) * 1/14.375 * deg2rad;
    gyro_x = Gyro(1);
    gyro_y = Gyro(2);
    gyro_z = Gyro(3);
    
  
    %calibrate the magnetomer mesasurement
    Mag = [w(7);
           w(8);
           w(9)];

   
       
    MagCalibrate = MagCMatrix * (Mag - MagMMatrix);
    MagNorm = (MagCalibrate/norm(MagCalibrate));
  
  
    %AccNorm

   
    %z = MagNorm+z;
    %z/iteration
    
    % PREDICTION
    
    q0 = state(1);
    q1 = state(2);
    q2 = state(3);
    q3 = state(4);
    g4 = state(5);
    g5 = state(6);
    g6 = state(7);
    %Gyro Rotation into quatarion matrix    
    %state equation
    q0 = q0 + (-1/2 * gyro_x * q1 - 1/2 * gyro_y * q2 - 1/2 * gyro_z * q3) * dT;
    q1 = q1 + (1/2 * gyro_x * q0 - 1/2 * gyro_y * q3 + 1/2 * gyro_z * q2) * dT;
    q2 = q2 + (1/2 * gyro_x * q3 + 1/2 * gyro_y * q0 - 1/2 * gyro_z * q1) * dT;
    q3 = q3 + (-1/2 * gyro_x * q2 + 1/2 * gyro_y * q1 + 1/2 * gyro_z * q0) * dT;
    g4 = g4 + 0;
    g5 = g5 + 0;
    g6 = g6 + 0;
    %xk: state quaternion
    quaternion = quatnormalize([q0 q1 q2 q3]);
    
    state(1:4) = quaternion;
    state(5:7) = [g4, g5, g6];
   
    %assemble jacobian matrix
    F = [1,         -1/2*gyro_x*dT, -1/2*gyro_y*dT, -1/2*gyro_z*dT,  1/2*q1*dT,  1/2*q2*dT,  1/2*q3*dT;
         1/2*gyro_x*dT,  1,          1/2*gyro_z*dT, -1/2*gyro_y*dT, -1/2*q0*dT,  1/2*q3*dT, -1/2*q2*dT;
         1/2*gyro_y*dT, -1/2*gyro_z*dT,  1,          1/2*gyro_x*dT,  1/2*q3*dT, -1/2*q0*dT,  1/2*q1*dT;
         1/2*gyro_z*dT,  1/2*gyro_y*dT, -1/2*gyro_x*dT,  1,          1/2*q2*dT, -1/2*q1*dT, -1/2*q0*dT;
         0,          0,          0,          0,          1,          0,          0        ;
         0,          0,          0,          0,          0,          1,          0        ;
         0,          0,          0,          0,          0,          0,          1        ];
     
     % predict covariance matrix from the last one
    P = F*P*(F') + Q;
 
     r1 = AccNorm;
     r2 = cross(AccNorm,MagNorm)/norm(cross(AccNorm,MagNorm));
     r3 = cross(r1, r2);
    
     mm = [r1,r2,r3];
     mr = [sh1,sh2, sh3];
     ma = (mm*mr');
    
    %% magnetometer updates
    % compute predicted X body over ground in earth xy plane using the
    % already filtered quaternion
    q0 = state(1);
    q1 = state(2);
    q2 = state(3);
    q3 = state(4);
    
    %output from triad algo
    q0n = 0.5*sqrt(1+ma(1,1)+ma(2,2)+ma(3,3));
    q1n = 1/(4*q0n) * (ma(2,3) - ma(3,2));
    q2n = 1/(4*q0n) * (ma(3,1) - ma(1,3));
    q3n = 1/(4*q0n) * (ma(1,2) - ma(2,1));
    
    Ey = [q0 q1 q2 q3]' - [q0n q1n q2n q3n]';
    
    Hy = [1 0 0 0 0 0 0
          0 1 0 0 0 0 0
          0 0 1 0 0 0 0
          0 0 0 1 0 0 0];
    
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
    
    %Our DCM is transposed, as opposed to the one listed in wikipedia
    q0n = 0.5*sqrt(1+ma(1,1)+ma(2,2)+ma(3,3));
    q1n = 1/(4*q0n) * (ma(2,3) - ma(3,2));
    q2n = 1/(4*q0n) * (ma(3,1) - ma(1,3));
    q3n = 1/(4*q0n) * (ma(1,2) - ma(2,1));
    
    
    
    
    % FOR PYTHON PLOTTING
    dcm = quat2dcm(state(1:4)');
 % dcm = ma;
   [ dcm2quat(ma);state(1:4)';[q0n, q1n, q2n, q3n]]'
  
    [r1 r2 r3] = (quat2angle(state(1:4)'));
    
    [r1;r2;r3]*(1/deg2rad);
    
    dcmstr =  [num2str(dcm(1,1), '%10.6f') ' '  num2str(dcm(1,2), '%10.6f') ' '  num2str(dcm(1,3), '%10.6f') ' ' num2str(dcm(2,1), '%10.6f') ' '  num2str(dcm(2,2), '%10.6f') ' '  num2str(dcm(2,3), '%10.6f') ' ' num2str(dcm(3,1), '%10.6f') ' '  num2str(dcm(3,2), '%10.6f') ' ' num2str(dcm(3,3), '%10.6f')];
    
   
    fwrite(u, dcmstr);
    
    
    %%
    iteration = iteration + 1;
  
end
fclose(u);
