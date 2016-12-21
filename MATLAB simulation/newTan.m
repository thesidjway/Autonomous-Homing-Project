function [ phi ] = newTan( sinTheta, cosTheta )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

 phi = atan2(sinTheta, cosTheta) + (atan2(sinTheta, cosTheta) < 0)*2*pi;

end

