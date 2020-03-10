
disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    
    %%%%%%%%%%%%%% DO NOT MODIFY CODE BEFORE THIS  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialization
    % initialize the handle
	[returnCode,joint1]=vrep.simxGetObjectHandle(clientID,'UR3_joint1',vrep.simx_opmode_blocking); %#ok<*ASGLU>
    [returnCode,joint2]=vrep.simxGetObjectHandle(clientID,'UR3_joint2',vrep.simx_opmode_blocking);
    [returnCode,joint3]=vrep.simxGetObjectHandle(clientID,'UR3_joint3',vrep.simx_opmode_blocking);
    [returnCode,joint4]=vrep.simxGetObjectHandle(clientID,'UR3_joint4',vrep.simx_opmode_blocking);
    [returnCode,joint5]=vrep.simxGetObjectHandle(clientID,'UR3_joint5',vrep.simx_opmode_blocking);
    [returnCode,joint6]=vrep.simxGetObjectHandle(clientID,'UR3_joint6',vrep.simx_opmode_blocking); 
    [returnCode,suction]=vrep.simxGetObjectHandle(clientID,'BaxterVacuumCup#',vrep.simx_opmode_blocking); %#ok<*ASGLU>
    [returnCode,rf0]=vrep.simxGetObjectHandle(clientID,'ReferenceFrame0',vrep.simx_opmode_blocking);
    [returnCode,rf]=vrep.simxGetObjectHandle(clientID,'ReferenceFrame',vrep.simx_opmode_blocking);
    
    % set the M and S
    % since the starting Pose and Spacial Twists are the same, just copy
    % the results from the forward_kinematis.m
    M = [1 0 0 -0.271397650241852;0 1 0 8.50856304168701e-05;0 0 1 1.05112171173096;0 0 0 1];
    S = [0 -1 -1 -1 0 -1;0 0 0 0 0 0;1 0 0 0 1 0;0 0 0 0 0 0;0 -0.44887313246727 -0.692523241043091 -0.905773341655731 0.112225346267223 -0.991123199462891;0 0 0 0 0 0];
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Set the target %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % reachable example
    % generate a random reachable pose by setting the thetas and calculate
    % the T_1in0 with forward kinematics
%     T_1in0 = generate_pose(S,M);
%     T_1in0 = [0 -1 0 0; 0 0 -1 -0.4; 1 0 0 0.4; 0 0 0 1];
%     T_1in0 = [0 0 1 0; 0 -1 0 -0.4; 1 0 0 0.4; 0 0 0 1];
    

    % unreachable example
    %T_1in0 = [0.36239977 -0.93172532 0.02354451 2.46475623; 0.07551861 0.05453331 0.99565208 0.15179767; -0.92895820 -0.35904604 0.09012546 -0.82715879; 0.00000000 0.00000000 0.00000000 1.00000000];
    
%     for i = 1:size(T_goal, 3)
%         [theta_all(:, i), count] = inverse(T_goal(:, :, i), M, S);
%         if count == 1001
%             disp('This position is unreachable!!!');
%             return
%         end
%     end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for i = 1:7
        T_1in0 = T_goal(:, :, i);
        % get the expected parameters of Reference Frame 0
        R_1 = T_1in0(1:3,1:3);
        p_1 = T_1in0(1:3,4);
        R_1_eul = rotm2eul(R_1,'XYZ');
    
%     % check the target position is above the ground
%     if T_1in0(3,4) < 0
%         disp('This position is under the ground');
%         return
%     end
    
        % set the position and orientation of Reference Frame 0
        [returnCode]=vrep.simxSetObjectOrientation(clientID,rf0, -1, R_1_eul,vrep.simx_opmode_oneshot);
        [returnCode]=vrep.simxSetObjectPosition(clientID,rf0,-1,p_1,vrep.simx_opmode_oneshot);
        pause(1)
    end
    
    % Movement
    % return to the zero postion

    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint1,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint2,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint3,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint4,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint5,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint6,0,vrep.simx_opmode_blocking);
    pause(1)
    
%     for i = 1:size(T_goal, 3)
%         theta_rad = theta_all(i);
%         % move to desired position
%         [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint1,theta_rad(1),vrep.simx_opmode_blocking);
%         [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint2,theta_rad(2),vrep.simx_opmode_blocking);
%         [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint3,theta_rad(3),vrep.simx_opmode_blocking);
%         [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint4,theta_rad(4),vrep.simx_opmode_blocking);
%         [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint5,theta_rad(5),vrep.simx_opmode_blocking);
%         [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint6,theta_rad(6),vrep.simx_opmode_blocking);
%         pause(1)
%     end
    
    % robot and the reference frame return to the zero postion

    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint1,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint2,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint3,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint4,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint5,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint6,0,vrep.simx_opmode_blocking);


    %%%%%%%%%%%%%  DO NOT MODIFY CODE AFTER THIS  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    vrep.simxFinish(-1);

else
    disp('Failed connecting to remote API server');
end

vrep.delete(); % call the destructor!

disp('Program ended');

