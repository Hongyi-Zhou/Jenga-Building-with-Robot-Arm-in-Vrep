function [skew] = skew(w)
% Calculate the skew-symmetric 3*3 matrix of w
skew = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
end
