%%
% Robotics Lab 1. Main function



disp('Choose between forward an backward kinematics.');
input1 = input('\n Type f for forward, b for backward:', 's');
%% Forward kinematics
if( input1 == 'f')
    input2 = 1;
    angles = zeros(1,6);
    
    disp('\nType in 6 angles in degrees . Type exit to quit');
    for i = 1:1:6
        input2 = input('Another one ;D');

        angles(i) = input2;
    end
    
    %%position = foward_kinematics(angles);
    
        
%% 
elseif(input1 == 'b')
    
else
    disp('Program ended');
end
%%