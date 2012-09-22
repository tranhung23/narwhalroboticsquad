
close all;%close previously open figures
clear all;

s1 = serial('com30');    % define serial port
s1.BaudRate=115200;               % define baud rate
set(s1, 'terminator', 'LF');    % define the terminator for println
fopen(s1);
count = 1;  
z = [0 0 0 0 0 0 0 0 0];
c(1) = 0;
while(count < 20000) 
   
    
    w=fscanf(s1, '%d %d %d %d %d %d %d %d %d')'             % must define the input % d or %s, etc.
    
    z = z + w(1:9);
    GyroAvg = z(1:3)/count
    AccAvg = z(4:6)/count
    MagAvg = z(7:9)/count
 
   % b(count) = w(2)-  1709.76393442623 ;
   
   
    count = count +1;
    % c(count) = b(count-1) + c(count-1);
end
