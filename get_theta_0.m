function [theta_goal] = get_theta_0(T_goal, S, M)
% starting from zero position only!!!
flag = 1;

while flag == 1

    theta_goal = inverse(T_goal, M, S);
    flag = check_path( theta_goal, S);
    
end

end





function [flag] = check_path(theta, S)
flag = 0;
flag = check_joint3(theta, S);
if flag == 1
    return
end
    

end

function [flag] = check_joint3(theta, S)
% check the position of joint3
% flag = 0 if joint3 is higher than joint 2, i.e. no collision
flag = 0;

% initial position of each joint
p1 = [-0.5001; 0.00012; 0.8965];
p2 = [-0.5001; -0.1115; 0.9008];
p3 = [-0.5001; -0.1117; 1.1445];
p4 = [-0.5001; -0.1118; 1.3577];
p5 = [-0.5001; -0.1126; 1.4419];
p6 = [-0.5001; -0.1119; 1.4431];
p_start = [p1 p2 p3 p4 p5 p6; ones(1,6)]; % p_start is 4*6

% p_goal = zeros(4,6);
% p_goal(:, 1) = p_start(:, 1);

p3_goal = forward(theta(1:2), S(:, 1:2), p_start(:, 3)); % goal position of joint3

if p3_goal(3) < 0.9008 % joint3 is lower than joint2
    flag = 1;
    disp('bug');
end

end

function [T] = forward(theta, S, M)
% Calculate the Pose of forward kinematics
n = size(S);
n = n(2);

e = eye(4);

for i = 1:n
    e = e * expm(bracket(S(:,i))*theta(i));
    
end

T = e*M;
end
