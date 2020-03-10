function [ S ] = screw( a, q )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

S = [a; -skew(a)*q];



end
