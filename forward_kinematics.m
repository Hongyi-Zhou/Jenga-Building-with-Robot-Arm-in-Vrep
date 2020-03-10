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
    
    % get the position of each joint
    [returnCode,q1]=vrep.simxGetObjectPosition(clientID,joint1,-1,vrep.simx_opmode_blocking);
    [returnCode,q2]=vrep.simxGetObjectPosition(clientID,joint2,-1,vrep.simx_opmode_blocking);
    [returnCode,q3]=vrep.simxGetObjectPosition(clientID,joint3,-1,vrep.simx_opmode_blocking);
    [returnCode,q4]=vrep.simxGetObjectPosition(clientID,joint4,-1,vrep.simx_opmode_blocking);
    [returnCode,q5]=vrep.simxGetObjectPosition(clientID,joint5,-1,vrep.simx_opmode_blocking);
    [returnCode,q6]=vrep.simxGetObjectPosition(clientID,joint6,-1,vrep.simx_opmode_blocking);
    [returnCode,pM]=vrep.simxGetObjectPosition(clientID,rf,-1,vrep.simx_opmode_blocking);

    % get the orientation of each joint
    a1 = [0;0;1];
    a2 = [-1;0;0];
    a3 = [-1;0;0];
    a4 = [-1;0;0];
    a5 = [0;0;1];
    a6 = [-1;0;0];
    [returnCode,RM_eul]=vrep.simxGetObjectOrientation(clientID,rf,-1,vrep.simx_opmode_blocking);
    
    % get the initial pose M
    RM = eul2rotm(RM_eul);
    M = [RM pM'; 0 0 0 1];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Set the target %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    theta_deg = [60 30 -50 100 75 60]; % target angle of joint1, joint2 ... in degrees
    theta_rad = theta_deg/180*pi; % target angle in radius
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % get the spacial twist
    S = [screw(a1,q1') screw(a2,q2') screw(a3,q3') screw(a4,q4') screw(a5,q5') screw(a6,q6')];
    
    % get the parameters of Reference Frame 0
    
    % turn this into a function!!!!
    T = forward(S, theta_rad, M);
    R = T(1:3,1:3);
    p = T(1:3,4);
    R_eul = rotm2eul(R,'XYZ');
    
%     % turn on/off suction
%     % turn on the suction
%     [returnCode]=vrep.simxSetIntegerSignal(clientID,suction,1,vrep.simx_opmode_blocking); 
%     % turn off the suction
%     [returnCode]=vrep.simxSetIntegerSignal(clientID,suction,0,vrep.simx_opmode_blocking); 

    % set the position and orientation of Reference Frame 0
    [returnCode]=vrep.simxSetObjectOrientation(clientID,rf0, -1, R_eul,vrep.simx_opmode_oneshot);
    [returnCode]=vrep.simxSetObjectPosition(clientID,rf0,-1,p,vrep.simx_opmode_oneshot);

    
    % Movement
    % return to the zero postion

    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint1,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint2,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint3,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint4,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint5,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint6,0,vrep.simx_opmode_blocking);
    pause(1)
    
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint1,theta_rad(1),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint2,theta_rad(2),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint3,theta_rad(3),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint4,theta_rad(4),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint5,theta_rad(5),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,joint6,theta_rad(6),vrep.simx_opmode_blocking);
    pause(2)
    


    
    %%%%%%%%%%%%%  DO NOT MODIFY CODE AFTER THIS  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    vrep.simxFinish(-1);

else
    disp('Failed connecting to remote API server');
end

vrep.delete(); % call the destructor!

disp('Program ended');

