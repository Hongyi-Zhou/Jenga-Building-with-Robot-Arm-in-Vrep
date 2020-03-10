function [ bracket ] = bracket( S )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
w = S(1:3);
v = S(4:6);
bracket = [skew(w) v; zeros(1,4)];

end

