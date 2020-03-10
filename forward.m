function [T] = forward(S, theta, M)
% Calculate the Pose of forward kinematics
n = size(S);
n = n(2);

e = eye(4);

for i = 1:n
    e = e * expm(bracket(S(:,i))*theta(i));
    
end

T = e*M;
end
