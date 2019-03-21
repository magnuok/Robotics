function angles = inverse_kinematics(x, y, z, alpha, beta, gamma)
%returns a 6x8 matrix contaning the 8 differnt solutions.

% In total 8 solutions. Each column corresponds to 1 solution, contaning 6
% angles.
angles = zeros(6,8);

%converting the orientation position to a rotation matrix using eul2rotm
rotation_matrix = eul2rotm([deg2rad(gamma),deg2rad(beta), deg2rad(alpha)], 'XYZ');
position = [x;y;z];
T_base_tool = [rotation_matrix position; [0 0 0 1]];

%% t1. The 3 first joint angles are found trough the geometric method
t1_1 = atan2(y, x);
t1_2 = t1_1 + pi;

%% Exclude tip and lower part of system
d = 29;
b = 101;
a = 35;

%Two options for t1 leads to excluding lower part in two ways:
%1
px = x - d*T_base_tool(1,3)-a*cos(t1_1);
py = y - d*T_base_tool(2,3)-a*sin(t1_1);
pz = z - d*T_base_tool(3,3)-b;
%2
qx = x - d*T_base_tool(1,3)-a*cos(t1_2);
qy = y - d*T_base_tool(2,3)-a*sin(t1_2);
qz = z - d*T_base_tool(3,3)-b;


%% t3
% Angle caused by the small 23mm height from joint 3 to 4. Have to
% compesate. Assuming perpendicular.
ang= atan2(23,120);

t3_1 = (acos( ((px^2 + py^2 + pz^2) - 120^2 - 120^2) / (2*120*120)) - pi/2 + ang)*180/pi;
t3_2 = - t3_1;


t3_3 = (acos( ((qx^2 + qy^2 + qz^2) - 120^2 - 120^2) / (2*120*120)) - pi/2 + ang)*180/pi;
t3_4 = - t3_1;

%% t2
beta = atan2(pz,sqrt(px^2 + py^2));

% sqrt(120^2+23^2) = l2, length from joint 3 to (px,py,pz). 120^2 = l1
p_zeta = acos( (120^2+23^2 - 120^2 - (px^2+py^2+pz^2)) / (-2*(120)*sqrt(px^2+py^2+pz^2)));
q_zeta = acos( (120^2+23^2 - 120^2 - (qx^2+qy^2+qz^2)) / (-2*(120)*sqrt(qx^2+qy^2+qz^2)));

if(t3_1 < 0) % Finne ut av fortegn her!
    t2_1 = (beta + p_zeta)*180/pi;
    t2_2 = (beta - p_zeta)*180/pi;
    
    t2_3 = (beta + q_zeta)*180/pi;
    t2_4 = (beta - q_zeta)*180/pi;

else
    t2_1 = (beta + p_zeta)*180/pi;
    t2_2 = (beta - p_zeta)*180/pi;
    
    t2_3 = (beta + q_zeta)*180/pi;
    t2_4 = (beta - q_zeta)*180/pi;

end

%% Solutions. Make pretty if time
% Note that if there is 1 imaginary number, the whole matrix will convert
% to imaginary numbers a+bi, with b-value=0
% 1 and two
angles(1,1)= t1_1;
angles(1,2)= t1_1;
angles(2,1)= t2_1;
angles(2,2)= t2_1;
angles(3,1)= t3_1;
angles(3,2)= t3_1;
angles_4_5_6 = algebraic_method(t1_1, t2_1, t3_1, T_base_tool);
angles(4,1) = angles_4_5_6(1);
angles(4,2) = angles_4_5_6(1);
angles(5,1) = angles_4_5_6(2);
angles(5,2) = angles_4_5_6(3);
angles(6,1) = angles_4_5_6(4);
angles(6,2) = angles_4_5_6(4);

% 3 and 4
angles(1,3)= t1_1;
angles(1,4)= t1_1;
angles(2,3)= t2_2;
angles(2,4)= t2_2;
angles(3,3)= t3_2;
angles(3,4)= t3_2;
angles_4_5_6 = algebraic_method(t1_1, t2_2, t3_2, T_base_tool);
angles(4,3) = angles_4_5_6(1);
angles(4,4) = angles_4_5_6(1);
angles(5,3) = angles_4_5_6(2);
angles(5,4) = angles_4_5_6(3);
angles(6,3) = angles_4_5_6(4);
angles(6,4) = angles_4_5_6(4);

% 5 and 6
angles(1,5)= t1_2;
angles(1,6)= t1_2;
angles(2,5)= t2_3;
angles(2,6)= t2_3;
angles(3,5)= t3_3;
angles(3,6)= t3_3;
angles_4_5_6 = algebraic_method(t1_2, t2_3, t3_3, T_base_tool);
angles(4,5) = angles_4_5_6(1);
angles(4,6) = angles_4_5_6(1);
angles(5,5) = angles_4_5_6(2);
angles(5,6) = angles_4_5_6(3);
angles(6,5) = angles_4_5_6(4);
angles(6,6) = angles_4_5_6(4);

% 7 and 8
angles(1,7)= t1_2;
angles(1,8)= t1_2;
angles(2,7)= t2_4;
angles(2,8)= t2_4;
angles(3,7)= t3_4;
angles(3,8)= t3_4;
angles_4_5_6 = algebraic_method(t1_2, t2_4, t3_4, T_base_tool);
angles(4,7) = angles_4_5_6(1);
angles(4,8) = angles_4_5_6(1);
angles(5,7) = angles_4_5_6(2);
angles(5,8) = angles_4_5_6(3);
angles(6,7) = angles_4_5_6(4);
angles(6,8) = angles_4_5_6(4);
end

function angles_4_5_6 = algebraic_method(t1, t2, t3, T_base_tool)

DH_values_0_3 = [0 0 101 t1;
    35 pi/2 0 t2;
    0 -pi/2 120 0;
    0 pi/2 0 t3];

for i = 1:4
    T_0_3 = eye(4,4);
    for i = 1:4
        T_i = transform_matrix(DH_values_0_3(1,:));
        T_0_3 = T_0_3 * T_i;
    end

    T = inv(T_0_3)*T_base_tool;
    % T_3_6 is found by symbolab.com. Comparing the elements of T and T_3_6, we
    % can find the last three joint angles.

    %% t5
    t5_1 = acos(T(1,3))*180/pi;
    t5_2 = -t5_1;
    angles(5,1) = t5_1;
    angles(5,2) = t5_2;
    %% t4
    t4_1 = asin(T(3,3)/sin(t5_1))*180/pi;

    %% t6
    t6_1 = asin(T(1,2)/sin(t5_1))*180/pi;

end
angles_4_5_6 = [t4_1 t5_1 t5_2 t6_1];

end

%% Transform matrix from i-1 to i
function T = transform_matrix(parameters)
% % a, alpha, d, theta

T = [     cos(parameters(4))                  -sin(parameters(4))                 0                   parameters(1);
sin(parameters(4))*cos(parameters(2)) cos(parameters(4))*cos(parameters(2)) -sin(parameters(2)) -sin(parameters(2))*parameters(3);
sin(parameters(4))*sin(parameters(2)) cos(parameters(4))*sin(parameters(2))  cos(parameters(2))  cos(parameters(2))*parameters(3);
0                                   0                         0                       1];

end

% Rotation matrix using convetion XYZ
% Rotation = [cos(alpha)*cos(beta) cos(alpha)*sin(beta)*cos(gamma)-sin(alpha)*cos(gamma)  cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
%     sin(alpha)*cos(beta) sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma) sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
%     -sin(beta) cos(beta)*sin(gamma) cos(beta)*cos(gamma);
%     0 0 0]
% Position = [x;y;z;1];
% Transformation = horzcat(Rotation,Position);