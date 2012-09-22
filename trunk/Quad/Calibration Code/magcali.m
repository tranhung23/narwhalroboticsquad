function CalibrationTest()
% main routine : calls MgnCalibration
%                        and an auxilliary function 
%                        which performs 3D rotations
%
% n                    : nomber of simulated data (default 1000)
% noiselevel     : level of noise (default 0.05)
%
%   Author Alain Barraud  2008
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;%close previously open figures


s1 = serial('com31');    % define serial port
s1.BaudRate=115200;               % define baud rate
set(s1, 'terminator', 'LF');    % define the terminator for println
fopen(s1);


count = 1;  
z = [0 0 0];

while(count < 2000) 
   
    
    w=fscanf(s1, '%d %d %d %d %d %d %d %d %d')             % must define the input % d or %s, etc.
    
    x(count) = w(7);
    y(count) = w(8);
    z(count) = w(9);
    
    count
    count = count +1;
end

data = [x; y; z];

[m,n] = size(data)

figure;plot3(data(1,:),data(2,:),data(3,:),'*');axis equal
grid on;
% performs calibration
[A,c] = MgnCalibration(data)
Caldata=A*(data-repmat(c,1,n));% calibrated data
figure;plot3(Caldata(1,:),Caldata(2,:),Caldata(3,:),'*');axis equal
set(gca,'xlim',[-1 1],'ylim',[-1 1],'zlim',[-1 1])
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function M=Mqv(v)
x=v(1);y=v(2);z=v(3);t=v(4);
M=[(x^2+y^2-z^2-t^2) , (2*y*z+2*x*t)       , (-2*x*z+2*y*t);
      (-2*x*t+2*y*z)       , (z^2-y^2+x^2-t^2), (2*x*y+2*z*t) ;
       (2*x*z+2*y*t)       , (-2*x*y+2*z*t)      , (-y^2+x^2-z^2+t^2)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%