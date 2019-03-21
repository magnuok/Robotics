function pose = forward_kinematics(angles)
% returns [x,y,z,alph,beta,gamma]'.

% a, alpha, d, theta
DH_values_2 = [0 0 101 angles(1)*(pi/180);
    35 pi/2 0 angles(2)*(pi/180);
    0 -pi/2 120 0;
    0 pi/2 0 angles(3)*(pi/180);
    0 0 0 pi/2; %Usikker hvor joint 3 er til
    23 pi/2 0 angles(4)*(pi/180);
    0 0 120 0;
    0 -pi/2 0 angles(5)*(pi/180);
    0 pi/2 0 0
    0 0 29 angles(6)*(pi/180);
    ];

% Save T_i matrixes to verify result and calculate partial results. T
% initial 4x4 identity matrix
T_i = zeros(4,4,10); 
T = eye(4,4);
for i = 1:1:10
    T_i(:,:,i) = transform_matrix(DH_values_2(i,:));
    T = T * T_i(:,:,i);
end

% Assuming all joints is initial centered in origo for the respective
% coord.system. Testing for joint 1,2,3 and 6.
joint_positions(:,1) = T_i(:,:,1) * T_i(:,:,2) * [0 0 0 1]';
joint_positions(:,2) = T_i(:,:,1) * T_i(:,:,2) * T_i(:,:,3) *[0 0 0 1]'; 
joint_positions(:,3) = T_i(:,:,1) * T_i(:,:,2)  * T_i(:,:,3)...
    * T_i(:,:,4) * T_i(:,:,5) * T_i(:,:,6) *[0 0 0 1]'; 
joint_positions(:,4) = T_i(:,:,1) * T_i(:,:,2) * T_i(:,:,3)...
    * T_i(:,:,4) * T_i(:,:,5) * T_i(:,:,6) * T_i(:,:,7) * T_i(:,:,8) * T_i(:,:,9) *T_i(:,:,10) *[0 0 0 1]'; % same as T * [0001]';

% Each column containing x-y-z position for respective joint
position = zeros(3,4);
for i= 1:4
position(:,i) = joint_positions(1:3,i);
end


% Calculate orientation by euler angles, x-y-z
beta = rad2deg(atan2(-T(3,1), sqrt(T(1,1)^2 + T(2,1)^2)));
if (cos(beta) ~= 0)
alpha = rad2deg(atan2(T(2,1)/cos(beta),T(1,1)/cos(beta)));
gamma = rad2deg(atan2(T(3,2)/cos(beta),T(3,3)/cos(beta)));
end

pose = [position(:,4)', alpha, beta, gamma]';

end

%% Transform matrix from i-1 to i
function T = transform_matrix(parameters)
% % a, alpha, d, theta
T = [     cos(parameters(4))                  -sin(parameters(4))                 0                   parameters(1);
sin(parameters(4))*cos(parameters(2)) cos(parameters(4))*cos(parameters(2)) -sin(parameters(2)) -sin(parameters(2))*parameters(3);
sin(parameters(4))*sin(parameters(2)) cos(parameters(4))*sin(parameters(2))  cos(parameters(2))  cos(parameters(2))*parameters(3);
0                                   0                         0                       1];

end