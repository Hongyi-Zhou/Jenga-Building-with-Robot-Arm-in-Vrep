function [returnCode] = move_block( T_2in0, T_1in0, M, S, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, suction )
% move the block to the target position

% make sure joint3 is always above joint2

[theta_target_1, theta_abvtarget_1, flag] = get_theta( T_1in0, M, S, 0.1);
if flag == 1
    disp('Cannot reach target postion!')
    return
end
if (T_2in0(3,4)<0.80)
    gap = 0.03;
else
    gap = 0.008;
end
[theta_target_2, theta_abvtarget_2, flag] = get_theta( T_2in0, M, S, gap);
if flag == 1
    disp('Cannot reach target postion!')
    return
end

theta_1 = [theta_target_1(1,1);zeros(2,1);theta_abvtarget_1(4:6,1)];
[returnCode] = move_theta( theta_1, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, 1);
[returnCode] = move_theta( theta_abvtarget_1, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, 1);
pause(1);
[returnCode] = move_theta( theta_target_1, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, 1);
pause(0.1)
% turn on the suction
[returnCode] = vrep.simxSetIntegerSignal(clientID,suction,1,vrep.simx_opmode_oneshot); 
pause(0.1);
theta_2 = [theta_target_1(1,1);theta_abvtarget_2(2:6,1)];
% for i = 1 :2
%     theta = (i-2)/2*(theta_abvtarget_1-theta_target_1)+theta_abvtarget_1;
%     [returnCode] = move_theta( theta, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 ,2);
% end
theta_3 = [theta_abvtarget_1(1:2,1);theta_target_1(3:6,1)];
[returnCode] = move_theta( theta_3, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 ,2);
%[returnCode] = move_theta( theta_abvtarget_1, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 ,2);
%[returnCode] = move_theta( theta_2, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 ,3);
%[returnCode] = move_theta( zeros(6,1), clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 ,2);

theta_4 = [theta_abvtarget_1(1,1);theta_abvtarget_2(2:6,1)];
[returnCode] = move_theta( theta_4, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, 3);
[returnCode] = move_theta( theta_abvtarget_2, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, 2);
pause(1);

[returnCode] = move_theta( theta_target_2, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 ,1);
pause(0.1);
% turn off the suction

[returnCode]=vrep.simxSetIntegerSignal(clientID,suction,0,vrep.simx_opmode_oneshot); 
pause(1);
%[returnCode] = move_theta( theta_abvtarget_2, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 );
[returnCode] = move_theta( zeros(6,1), clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 ,2);

end