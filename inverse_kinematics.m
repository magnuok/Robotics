function solutions = inverse_kinematics(parameters)
% Returns a 6x8 matrix contaning the 8 differnt solutions. 
% Each column corresponds to 1 solution.

x = parameters(1);
y = parameters(2);
z = parameters(3);
alpha = parameters(4);
beta = parameters(5);
gamma = parameters(6);

% Converting the orientation position to a rotation matrix using eul2rotm
rotation_matrix = eul2rotm([deg2rad(gamma),deg2rad(beta), deg2rad(alpha)], 'ZYX');
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
%1:
% TODO: Check for the situation of negative z-coord. 
px = x - d*T_base_tool(1,3)-a*cos(t1_1);
py = y - d*T_base_tool(2,3)-a*sin(t1_1);
pz = z - d*T_base_tool(3,3)-b;

%2:
qx = x - d*T_base_tool(1,3)-a*cos(t1_2);
qy = y - d*T_base_tool(2,3)-a*sin(t1_2);
qz = z - d*T_base_tool(3,3)-b;

%% t3
% Angle caused by the small 23mm height from joint 3 to 4. Have to
% compesate. Assuming perpendicular.
dt_3= atan2(23,120);

t3_1 = pi/2 + acos( ((px^2 + py^2 + pz^2) - 120^2 - 120^2 - 23^2) / (2*120*sqrt(120^2+23^2))) - dt_3;
t3_2 = pi/2 - acos( ((px^2 + py^2 + pz^2) - 120^2 - 120^2 - 23^2) / (2*120*sqrt(120^2+23^2))) - dt_3;

t3_3 = pi/2 + acos( ((qx^2 + qy^2 + qz^2) - 120^2 - 120^2 - 23^2) / (2*120*sqrt(120^2+23^2))) - dt_3;
t3_4 = pi/2 - acos( ((qx^2 + qy^2 + qz^2) - 120^2 - 120^2 - 23^2) / (2*120*sqrt(120^2+23^2))) - dt_3;

%% t2
p_delta = atan2(pz,sqrt(px^2 + py^2));
q_delta = atan2(qz,sqrt(qx^2 + qy^2));

p_zeta = acos( (120^2+23^2 - 120^2 - (px^2+py^2+pz^2)) / (-2 * 120 * sqrt(px^2+py^2+pz^2)));
q_zeta = acos( (120^2+23^2 - 120^2 - (qx^2+qy^2+qz^2)) / (-2 * 120 * sqrt(qx^2+qy^2+qz^2)));

if(t3_1 < 0)
    
    t2_1 = (p_delta + p_zeta) - pi/2;
    t2_2 = (p_delta - p_zeta) - pi/2;
    
    t2_3 = (q_delta + q_zeta) - pi/2;
    t2_4 = (q_delta - q_zeta) - pi/2;

elseif(t3_1 >= 0) 
    t2_1 = (p_delta - p_zeta) - pi/2;
    t2_2 = (p_delta + p_zeta) - pi/2;
    
    t2_3 = (q_delta - q_zeta) - pi/2;
    t2_4 = (q_delta + q_zeta) - pi/2;

end

%% Solutions
% Note that if there is 1 imaginary number, the whole matrix will convert
% to imaginary numbers a+bi, with b-value=0 for possible solutions.

sol1 = [t1_1 t1_1; t2_1 t2_1; t3_1 t3_1; algebraic_method(t1_1, t2_1, t3_1, T_base_tool)]*180/pi;

sol2 = [t1_1 t1_1; t2_2 t2_2; t3_2 t3_2; algebraic_method(t1_1, t2_2, t3_2, T_base_tool)]*180/pi;

sol3 = [t1_2 t1_2; t2_3 t2_3; t3_3 t3_3; algebraic_method(t1_2, t2_3, t3_3, T_base_tool)]*180/pi;

sol4 = [t1_2 t1_2; t2_4 t2_4; t3_4 t3_4; algebraic_method(t1_2, t2_4, t3_4, T_base_tool)]*180/pi;


solutions = [sol1 sol2 sol3 sol4];

end

function angles_4_5_6 = algebraic_method(t1, t2, t3, T_base_tool)
% Returns t4, t5, t6 that corresponds to t1, t2, t3

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
    % Avoiding round off errors
    T = round(T, 3);
    
    
    % T_3_6 is found by symbolab.com. Comparing the elements of T and T_3_6, we
    % can find the last three joint angles.

    %% t5
    t5_1 = acos(T(1,3));
    t5_2 = -t5_1;
    angles(5,1) = t5_1;
    angles(5,2) = t5_2;
    
    %% t4 and t6
    % Singularity. Infinit solutions for t4 and t6
    if(t5_1 == 0)
    t4_1 = 0;
    t4_2 = 0;
    t6_1 = 0;
    t6_2 = 0;
        
    else
    t4_1 = acos(T(2,3)/sin(t5_1));
    t4_2 = -t4_1;
    
    t6_1 = asin(T(1,2)/sin(t5_1));
    t6_2 = asin(T(1,2)/sin(-t5_1));
    end

end
angles_4_5_6 = [t4_1 t4_2; t5_1 t5_2; t6_1 t6_2];

end

%% Transform matrix from i-1 to i
function T = transform_matrix(parameters)
% % a, alpha, d, theta

T = [     cos(parameters(4))                  -sin(parameters(4))                 0                   parameters(1);
sin(parameters(4))*cos(parameters(2)) cos(parameters(4))*cos(parameters(2)) -sin(parameters(2)) -sin(parameters(2))*parameters(3);
sin(parameters(4))*sin(parameters(2)) cos(parameters(4))*sin(parameters(2))  cos(parameters(2))  cos(parameters(2))*parameters(3);
0                                   0                         0                       1];

end