function [returnCode] = move_theta( theta_target, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, mode )
% set the joints to the target angel
if (mode == 1)%down
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint1,theta_target(1),vrep.simx_opmode_streaming);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint6,theta_target(6),vrep.simx_opmode_streaming);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint5,theta_target(5),vrep.simx_opmode_streaming);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint4,theta_target(4),vrep.simx_opmode_streaming);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint2,theta_target(2),vrep.simx_opmode_streaming);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint3,theta_target(3),vrep.simx_opmode_streaming);
    pause(1);
elseif (mode == 2)%up
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint2,theta_target(2),vrep.simx_opmode_streaming);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint3,theta_target(3),vrep.simx_opmode_streaming);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint1,theta_target(1),vrep.simx_opmode_streaming);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint6,theta_target(6),vrep.simx_opmode_streaming);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint5,theta_target(5),vrep.simx_opmode_streaming);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint4,theta_target(4),vrep.simx_opmode_streaming);
    pause(1);
else
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint2,theta_target(2),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint3,theta_target(3),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint1,theta_target(1),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint6,theta_target(6),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint5,theta_target(5),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint4,theta_target(4),vrep.simx_opmode_blocking);
    pause(1);
end

end