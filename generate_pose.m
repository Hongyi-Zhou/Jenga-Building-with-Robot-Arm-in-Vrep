function [T] = generate_pose(S, M)
% Generate pose that is reachable
n = size(S);
n = n(2);
theta = randn(n,1);
T = forward(S, theta, M);
end