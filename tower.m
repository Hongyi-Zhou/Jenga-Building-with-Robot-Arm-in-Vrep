function [T_start,T_goal] = tower(ori,pos_block)

%% parameters from the vision block
ori = ori/180*pi;
num_block = size(ori, 2); % the number of blocks
%pos_block = [-0.67433774471283 -0.821375370025635 -0.895617842674255 -0.774388194084167 -0.898430109024048 -0.699217677116394 -0.403197646141052;0.24876856803894 0.183375179767609 0.0506212115287781 -0.0268100500106812 -0.19967246055603 -0.273376405239105 -0.447740435600281];
%% parameters don't need to modify
height_table = 0.764;
height_block = 0.024;
width_block = 0.04;
length_block = 0.12;
margin = 0.003; % how close would the blocks be each other

%% get the initial pose of each block
pos_block(3, :) = ones(1, num_block)*(0.764+0.024/2+0.004); % every block is on table

T_start = zeros(4, 4, num_block); 
for i = 1:num_block
    T_start(4, 4, i) = 1;
    T_start(3, 1:3, i) = [0 1 0];
    T_start(1:2, 1:3, i) = [cos(ori(i)) 0 sin(ori(i)); sin(ori(i)) 0 -cos(ori(i))]; 
    T_start(1:3, 4, i) = pos_block(:,i);
end

%% calculate the goal pose of each block
% define the target position of the tower
pos_tower = [-0.18; 0]; % goal position of the first block of tower
pos_tower_even = [pos_tower(1) + (width_block + margin); pos_tower(2) + length_block/2 + margin - width_block/2];

T_goal = zeros(4, 4, num_block); 

% total_tier = floor(num_block/3) + 1; % total tiers if we use all the blocks

for i = 1:num_block
    T_goal(4, 4, i) = 1;
    tier = floor((i-1)/3) + 1; % which tier would this block be
    
    height = height_table + height_block/2 + (tier - 1)*height_block+0.008;
    T_goal(3, 4, i) = height;
    
    if mod(tier, 2) == 1 % ture for odd tier, false for even tier
        % perpendicular to the long side of the table
        T_goal(1:3, 1:3, i) = [0 0 1; 1 0 0; 0 1 0]; 
        T_goal(1:2, 4, i) = [pos_tower(1) + (width_block + margin)*mod(i-1, 3); pos_tower(2)];
        
    else % even tier
        % along the long side of the table
        T_goal(1:3, 1:3, i) = [-1 0 0; 0 0 1; 0 1 0]; 
        T_goal(1:2, 4, i) = [pos_tower_even(1); pos_tower_even(2) - (width_block + margin)*mod(i-1, 3)];
        
    end
end
end

    

    

    
    