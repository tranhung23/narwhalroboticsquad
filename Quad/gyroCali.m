clear all;
close all;%close previously open figures


s1 = serial('com7');    % define serial port
s1.BaudRate=115200;               % define baud rate
set(s1, 'terminator', 'LF');    % define the terminator for println
fopen(s1);


count = 1;  
z = [0 0 0];

while(count < 2000) 
   
    
    w=fscanf(s1, '%d %d %d %d %d %d %d %d %d');              % must define the input % d or %s, etc.
  w(4:6)'
    z = z + w(4:6)';
    z/count
    count = count +1;
    end
    
