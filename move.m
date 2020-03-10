clear
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
    [returnCode,c1]=vrep.simxGetObjectHandle(clientID,'Cuboid1',vrep.simx_opmode_blocking);
    [returnCode,pos]=vrep.simxGetObjectHandle(clientID,'Position',vrep.simx_opmode_blocking);
    [returnCode,suction]=vrep.simxGetObjectHandle(clientID,'BaxterVacuumCup',vrep.simx_opmode_blocking);
    suctioncup = 'BaxterVacuumCup_active';
    
    %% read vision sensor
    [returnCode] = move_theta( zeros(6,1), clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 ,1 );
    [returnCode,ints,data,strings,buffers]=vrep.simxCallScriptFunction(clientID,'Position',1,'getData',[],[],[],[],vrep.simx_opmode_blocking);
    [returnCode,cam_pos]=vrep.simxGetObjectPosition(clientID,pos,-1,vrep.simx_opmode_blocking);
    [returnCode,z_block]=vrep.simxGetObjectPosition(clientID,c1,-1,vrep.simx_opmode_blocking);
    camera_side = (cam_pos(3)-z_block(3))*tan(35/180*pi)*sqrt(2)+0.414;
    [r,c] = size(data);
    c = 195; %problem here
    ori = zeros(1,c/5);
    pos_block = zeros(2,c/5); %x,y position in word frame
    for i = 1:c/5
        if (data(5*i-1)>data(5*i))%width>height
            ori(i) = 180+data(5*i-4)/pi*180;
        elseif (data(5*i-1)<data(5*i))
            ori(i) = 90+data(5*i-4)/pi*180;
        end% orientation with respect to longer side of table
        pos_block(1,i) = -data(5*i-3)*camera_side+cam_pos(1)+0.5*camera_side;% x 
        pos_block(2,i) = -data(5*i-2)*camera_side+cam_pos(2)+0.5*camera_side;% y
    end
    
    %% start,target pose
    [T_start,T_target] = tower(ori,pos_block);
        
    %% move theta test
    [returnCode] = move_theta( zeros(6,1), clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 ,1 );
    
    M = [1 0 0 -0.5001;0 1 0 -0.2577;0 0 1 1.4431;0 0 0 1];
    S = [0 0 0 0 0 0;0 -1 -1 -1 0 -1;1 0 0 0 1 0;0.000124603509902954 0.900820851325989 1.14448392391205 1.3577333688736 -0.112495973706245 1.44307839870453;0.500085592269897 0 0 0 0.500092089176178 0;0 0.50005042552948 0.500130951404572 0.500089764595032 0 0.500092506408691];
    
    for i =1:39
        [returnCode] = move_block( T_target(:,:,i), T_start(:,:,i), M, S, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, suctioncup);
    end
%      T_1in0 = T_start(:,:,28);
%           T_1in0(3,4) = T_1in0(3,4)+0.14;
% %     [ 0 0 1 -0.3995; 1 0 0 -0.45; 0 1 0 0.764+0.024/2+0.003; 0 0 0 1];
% %     [returnCode] = move_block( T_target(:,:,2), T_start(:,:,2), M, S, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, suctioncup);
%      [theta_rad] = inverse(T_1in0, M,S);

    
    %theta_rad =[-2;0.720126733884008;1.71893638191425;-0.868266789002227;-1.5707963267949;-0.285657362765744];
    %tt = [-2.98357722278659;-0.497133774488862;-0.539484690850834;-0.534176058226377;1.57079488750792;-2.98357726930855];
    %[theta_target_1, theta_abvtarget_1, flag] = get_theta( T_1in0, M, S, joint1, joint2, joint3, joint4, joint5, joint6 );
    %T_2in0 = [0 -1 0 0.8995; 0 0 -1 0.05; 1 0 0 0.764; 0 0 0 1];
    %pause(2);
    theta_rad = [6;1.57;0;0;0;0];
    [returnCode] = move_theta( theta_rad, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, 1 );
    %turn on suction cup
    %pause(2);
    %[returnCode] = vrep.simxSetIntegerSignal(clientID, suctioncup, 1, vrep.simx_opmode_oneshot)
    %pause(1);
    %[returnCode] = move_theta( tt, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6 );
    %[returnCode] = vrep.simxSetIntegerSignal(clientID, suctioncup, 0, vrep.simx_opmode_oneshot);
    %pause(1);
    %[returnCode] = move_theta( theta_rad, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, 1 );
    %[returnCode] = move_block( T_2in0, T_1in0, M, S, clientID,  vrep, joint1, joint2, joint3, joint4, joint5, joint6, suction);

    


    
    %%%%%%%%%%%%%  DO NOT MODIFY CODE AFTER THIS  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    vrep.simxFinish(-1);

else
    disp('Failed connecting to remote API server');
end

vrep.delete(); % call the destructor!

disp('Program ended');

