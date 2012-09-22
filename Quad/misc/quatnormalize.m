function [ qOut ] = quatnormalize( qIn )
%QUATNORMALIZE Summary of this function goes here
%   Detailed explanation goes here

qOut = qIn / norm(qIn);

end

