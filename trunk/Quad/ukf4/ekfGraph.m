%% plot constants
XMIN = -500;
XMAX = 200;
YMIN = -30;
YMAX = 30;

%% Define serial port
s1 = serial('COM4');    % define serial port
s1.BaudRate=115200;               % define baud rate
set(s1, 'terminator', 'LF');    % define the terminator for println
fopen(s1);

%% Define Iteration Constants
iteration = 1;
t0 = clock;

%% Start Reading
u=udp('127.0.0.1',9091);
fopen(u);

while(true)
    line = fscanf(s1, '%f,%f,%f');
    
    angleYaw(iteration) = line(1);
    anglePitch(iteration) = line(2);
    angleRoll(iteration) = line(3); 
    
    subplot(3,1,1);
    plot(angleYaw, 'r');
    title('angleYaw');
    axis([iteration+XMIN iteration+XMAX YMIN YMAX]);
    
    subplot(3,1,2);
    plot(anglePitch, 'g');
    title('anglePitch');
    axis([iteration+XMIN iteration+XMAX YMIN YMAX]);
    
    subplot(3,1,3);
    plot(angleRoll, 'b');
    title('angleRoll');
    axis([iteration+XMIN iteration+XMAX YMIN YMAX]);
    iteration = iteration+1;
    
    drawnow
end