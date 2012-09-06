function [ yaw, pitch, roll ] = q2a(x,y,z,w)


    test = w*y - x*z;



% 	test = x*y + z*w;
% 	if (test > 0.499)  % singularity at north pole
% 		yaw = 2 * atan2(x,w);
% 		pitch = pi/2;
% 		roll = 0;
% 		return;
%     end
% 	if (test < -0.41)  % singularity at south pole
% 		yaw = -2 * atan2(x,w);
% 		pitch = - pi/2;
% 		roll = 0;
% 		return;
%     end

    yaw = atan2(2*(w*z+x*y),1-2*(y*y+z*z));
	pitch = asin(2*(w*y - x*z));	
    roll = atan2(2*(w*x + y*z),1-2*(x*x+y*y));


end

