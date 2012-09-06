gyrocali500 = [ 37.9415;
                18.4904;
                21.9358];

MagCMatrix = [0.2013789977353   0.0023528392492  -0.0049776402801;
              0                   0.1996831018998  -0.0009618099551;
              0                   0                   0.1953393843802];

MagMMatrix = [-110.4814;
              -0.9313;
               1.2041];


 
%SERIAL PORT
s1 = serial('com7');    % define serial port
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
Rmag = [0.3, 0, 0;
       0, 0.3, 0;
       0, 0, 0.3];
   
   
% Quaternion matrix
state = [1 0 0 0 0 0 0]';

 u=udp('127.0.0.1',9091);
 fopen(u);
 
 %generate refererence magnetomer vector
 ae = [-0.0213;
        0.0116;
        0.9997];
 me = [0.0054;
      -0.2910;
      -0.9566];
 
 sh1 = ae;
 sh2 = cross(ae,me)/norm(cross(ae,me));
 sh3 = cross(sh1, sh2);
 
 while(true)
  
    w=fscanf(s1, '%f %f %f %f')            % must define the input % d or %s, etc.
  
   
    
    state(1) = w(1);
     state(2) = w(2);
      state(3) = w(3);
       state(4) = w(4);
    % FOR PYTHON PLOTTING
    dcm = quat2dcm(state(1:4)');
    [r1 r2 r3] = (quat2angle(state(1:4)'));
    
    [r1;r2;r3]*(1/deg2rad)
    
    dcm = dcm * [0.7071 -0.7071 0;
                 0.7071 0.7071  0;
                 0 0 1];
    dcmstr =  [num2str(dcm(1,1), '%10.6f') ' '  num2str(dcm(1,2), '%10.6f') ' '  num2str(dcm(1,3), '%10.6f') ' ' num2str(dcm(2,1), '%10.6f') ' '  num2str(dcm(2,2), '%10.6f') ' '  num2str(dcm(2,3), '%10.6f') ' ' num2str(dcm(3,1), '%10.6f') ' '  num2str(dcm(3,2), '%10.6f') ' ' num2str(dcm(3,3), '%10.6f')];
    
   
    fwrite(u, dcmstr);
    
    
    %%
    iteration = iteration + 1;
  
end
fclose(u);
