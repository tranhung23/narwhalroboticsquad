%% Unscented Kalman Filter
%% Define Constants
% accRef used in init
accRef = [2075.03720930233, 2076.99069767442, 2076.99069767442];

% gyroRef used in init
gyroRef = [1668 1688 1721];

% magnetometer soft and hard iron calibration matrix
magC = [0.001651600913498, 0.000164539043519, -0.000175247708272;
    0,                 0.001767259192109,  0.000056402873714;
    0,                 0,                  0.001577958642801];
magM = [-31.2612766202469;
    -10.7910288640105;
    -227.2095173856283];

% deg2rad
deg2rad = 0.0174532925;

% gravity
GRAVITY = 9.8;

%% Define serial port
s1 = serial('com4');    % define serial port
s1.BaudRate=115200;               % define baud rate
set(s1, 'terminator', 'LF');    % define the terminator for println
%fopen(s1);

%% Define Iteration Constants
iteration = 1;
t0 = clock;

%% Start Reading
u=udp('127.0.0.1',9091);
fopen(u);


%%
%UKF Vars
n = 7;
m = 3;

% initializing 17 state
q = [1;0;0;0]; % quaternion

x = [1;0;0;0;0;0;0];

% initializing noise matrices
% process noise
Q = [0.000001, 0, 0, 0, 0, 0, 0;
     0, 0.000001, 0, 0, 0, 0, 0;
     0, 0, 0.000001, 0, 0, 0, 0;
     0, 0, 0, 0.000001, 0, 0, 0;
     0, 0, 0, 0, 0.0001, 0, 0; 
     0, 0, 0, 0, 0, 0.0001, 0;
     0, 0, 0, 0, 0, 0, 0.0001];
 Q = 3.4962e-005 * eye(n) * 10000;

% measurement noise
R = [0.3, 0, 0;
       0, 0.3, 0;
       0, 0, 0.3];
   R = 0.0383 * eye(m)*10;


P = [1, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0.02, 0, 0; 
     0, 0, 0, 0, 0, 0.02, 0;
     0, 0, 0, 0, 0, 0, 0.02];

%% Start UKF
while(true)
    dt = 0.0002249;
    t0 = clock;
    
    % Gyro (3), Acclermeter (3), Magnotometer (3), Temperature (1), Time (1)
    line = mika2(iteration,:);
    if length(line) < 9
        continue;
    end
    %line = fscanf(s1, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
    [accNorm, gyroFixed, magNorm] = readSensor(line, accRef, gyroRef, magC, magM);
    u = [accNorm; gyroFixed];
    
    
    %% estimation
    
    [Xi W Wc] = SigmaPoints(x, P, 1e-6);
    Xi
    iteration = iteration+1;
    continue;
    fXi = zeros(n, 2*n+1);
    for k = 1:2*n+1
        fXi(:, k) = fx(Xi(:,k), Q, u, dt, GRAVITY);
    end
    [x P] = UT(fXi, W,Wc, Q);
    %% update
    [x P] = accUpdate(x, P, R, accNorm(1), accNorm(2), accNorm(3), m, n);
    x
%     [x P] = magUpdate(x, P, magNorm(1), magNorm(2), magNorm(3));
    
    
    %% quaternion to euler
    q0 = x(1);
    q1 = x(2);
    q2 = x(3);
    q3 = x(4);
    roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1^2+q2^2)) * 180 / pi;
    pitch = asin(2*(q0*q2-q3*q1))* 180 / pi;
    yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2^2+q3^2))* 180 / pi;

%     q = x(1:4)';
%     [yaw, pitch, roll] = quat2angle(q);
%     yaw = yaw * 180 / pi;
%     pitch = pitch * 180 / pi;
%     roll = roll * 180 / pi;
    
    angleYaw(iteration) = yaw;
    anglePitch(iteration) = pitch;
    angleRoll(iteration) = roll;
    gyros(iteration) = gyroFixed(1);

livePlot('anglePitch', anglePitch, 'angleRoll', angleRoll, 'angleYaw', angleYaw, iteration);
    %, pitch, roll]
    
    %% END
    iteration = iteration + 1;
end
fclose(u);
try
catch exception
    fclose(s1);
    throw (exception);
end
