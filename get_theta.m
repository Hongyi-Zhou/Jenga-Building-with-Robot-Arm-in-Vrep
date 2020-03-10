function [ theta_target, theta_abvtarget, flag] = get_theta( T_1in0, M, S, gap)
% set tol = 1 when moving vertical
% move the arm to the target position
flag = 0;


% check the target position is above the ground
if T_1in0(3,4) < 0
    flag = 1;
    return
end

% set the position above the target
T_abvtarget = T_1in0;
T_abvtarget(3,4) = T_abvtarget(3,4)+gap;

[theta_target] = inverse(T_1in0, M, S);
[theta_abvtarget] = inverse(T_abvtarget, M, S);
%[theta_target, count] = inverse(T_1in0, M, S);
%[theta_abvtarget, count] = inverse(T_abvtarget, M, S);



% % make sure the movement is "small" instead of achieving in totally another way
% dtheta = 1000;
% 
% while dtheta > 0.6 
% %     theta_target = [0; 0; 1; 0; 0; 0];
% %     theta_abvtarget = theta_target;
% %     while theta_target(3) > 0 && theta_abvtarget(3) > 0
%     theta_target = [1; -1; 1; 0; 0; 0];
%     theta_abvtarget = theta_target;
%     %while theta_target(3) < 0 || theta_abvtarget(3) < 0 
%     %while theta_target(1)>0 && theta_target(3)>0 || theta_target(1)<0 && theta_target(3)>0
%     while theta_target(2)<0
%         [theta_target, count] = inverse(T_1in0, M, S);
% 
%         if count == 1001
%     %         flag = 1;
%             return
%         end
% 
%         [theta_abvtarget, count] = inverse(T_abvtarget, M, S);
% 
%         if count == 1001
%     %         flag = 1;
%             return
%         end
%         
%     end
%     dtheta = norm(theta_abvtarget - theta_target);
% end
% disp(theta_abvtarget/pi*180);
% disp(theta_target/pi*180);

end