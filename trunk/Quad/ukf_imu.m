
%%
%Calibration
gyrocali500 = [ -0.0938;
                -0.0015;
                0.0323];

MagCMatrix = [0   1   0
              -1  0   0
             0  0   1];

MagMMatrix =   [ 67.0335
  -55.9988
  131.3037];

%%
%Serial port

 
%SERIAL PORT
s1 = serial('com15');    % define serial port
s1.BaudRate=115200;               % define baud rate
set(s1, 'terminator', 'LF');    % define the terminator for println
fopen(s1);
%END SERIAL PORT

%%
%Declations
deg2rad = 0.0174532925;
iteration = 1;                
t0 = clock;

%%
%udp
u=udp('127.0.0.1',9091);
fopen(u);
 
%%
% magnotometer stuff
%generate refererence magnetomer vector
ae = [-0.0213;
       0.0116;
       0.9997];
me = [-0.9209
      -0.2977
       0.2515];
 
sh1 = ae;
sh2 = cross(ae,me)/norm(cross(ae,me));
sh3 = cross(sh1, sh2);

Acc = [0 0 1];

%%
%UKF

state = [1;0;0;0];

n=4;      %number of state
q=100;    %std of process 
r=0.0001;    %std of measurement

Q=q^2*eye(n); % covariance of process
R=r^2;        % covariance of measurement 
P = eye(n);   % initial state covraiance

fstate=@(q,gyro,dT)[q(1) + (-1/2 * gyro(1) * q(2) - 1/2 * gyro(2) * q(3) - 1/2 * gyro(3) * q(4)) * dT;
       q(2) + (1/2 * gyro(1) * q(1) - 1/2 * gyro(2) * q(4) + 1/2 * gyro(3) * q(3)) * dT;
       q(3) + (1/2 * gyro(1) * q(4) + 1/2 * gyro(2) * q(1) - 1/2 * gyro(3) * q(2)) * dT;
       q(4) + (-1/2 * gyro(1) * q(3) + 1/2 * gyro(2) * q(2) + 1/2 * gyro(3) * q(1)) * dT]; 
% nonlinear state equations


% h=@(x)[2*(q1*q3 - q0*q2);
%        2*(q2*q3 + q0*q1);
%        1 - 2*(q1*q1 + q2*q2)]; 

%Acclermeter data
h=@(q)[2*(q(2)*q(4) - q(1)*q(3));
       2*(q(3)*q(4) + q(1)*q(2));
       1 - 2*(q(2)*q(2) + q(3)*q(3))]; 

    L=numel(state);                                 %numer of states
    m=numel(Acc);                                 %numer of measurements
    
    alpha=1e-3;                                 %default, tunable
    ki=0;                                       %default, tunable
    beta=2;                                     %default, tunable
    lambda=alpha^2*(L+ki)-L;                    %scaling factor
    
    c=L+lambda;                                 %scaling factor
    
    Wm=[lambda/c 0.5/c+zeros(1,2*L)];           %weights for means
    Wc=Wm;
    
    Wc(1)=Wc(1)+(1-alpha^2+beta);               %weights for covariance
    c=sqrt(c);          




while(true)
  
    dT = etime(clock,t0);
    t0 = clock;
    w=fscanf(s1, '%d %d %d %d %d %d %d %d %d')              % must define the input % d or %s, etc.
  
    Acc = [w(1);
           w(2);
           w(3)];
    Acc = Acc/norm(Acc);

    %gyro data - this case is 9.1mV/degree, we are at 3.3V, 1024 steps,
    %each step is 3300/1024 = 3.22265625mV/step -> 0.354138049 degree/step
   
    %correct gyro with reference (need temp correction here), multiply by
    %step to get degrees, and then multiply by rad
    gyro = ([w(4); w(5); w(6)] - gyrocali500) * 1/14.375 * deg2rad;
        
    %%
    %UKF step
    
    %Unscented transformation set up
   
    X=sigmas(state,P,c);                            %sigma points
    %Z=sigmas(state,P,c);                            %sigma points
    [x1,X1,P1,X2]=utf(fstate,gyro,dT,X,Wm,Wc,L,Q);       %unscented transformation of process
    
   % X1=sigmas(x1,P1,c);                         %sigma points around x1
	%X2=X1-x1(:,ones(1,size(X1,2)));             %deviation of X1
    
    [z1,Z1,P2,Z2]=ut(h,X1,Wm,Wc,m,R);       %unscented transformation of measurments
    P12=X2*diag(Wc)*Z2';                        %transformed cross-covariance
    K=P12*inv(P2);
    x=x+K*(Acc-z1);                              %state update
    P=P1-K*P12';                                %covariance update
    
    
    %sV(:,k)= s;                             % save actual state
    %zV(k)  = z;                             % save measurment
    %[x, P] = ukf(f,x,P,h,z,Q,R);            % ukf 
    %xV(:,k) = x;                            % save estimate
    x = quatnormalize(x')';
    state = fstate(x, gyro, dT);                % update process 
    
    %%
%     DCM UDP 
    dcm = quat2dcm(state(1:4)');
    dcmstr =  [num2str(dcm(1,1), '%10.6f') ' '  num2str(dcm(1,2), '%10.6f') ' '  num2str(dcm(1,3), '%10.6f') ' ' num2str(dcm(2,1), '%10.6f') ' '  num2str(dcm(2,2), '%10.6f') ' '  num2str(dcm(2,3), '%10.6f') ' ' num2str(dcm(3,1), '%10.6f') ' '  num2str(dcm(3,2), '%10.6f') ' ' num2str(dcm(3,3), '%10.6f')];
    fwrite(u, dcmstr);
    iteration = iteration + 1;
    
end

