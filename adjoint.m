function [ adjoint ] = adjoint( M )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

R = M(1:3, 1:3);
p = M(1:3, 4);
adjoint = [R zeros(3,3); skew(p)*R R];

end

