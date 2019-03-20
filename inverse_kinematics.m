function angles = inverse_kinematics(x, y, z, alpha, beta, gamma)
%% Transformation matrix 

% Rotation matrix using convetion XYZ
% Rotation = [cos(alpha)*cos(beta) cos(alpha)*sin(beta)*cos(gamma)-sin(alpha)*cos(gamma)  cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
%     sin(alpha)*cos(beta) sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma) sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
%     -sin(beta) cos(beta)*sin(gamma) cos(beta)*cos(gamma);
%     0 0 0]
% Position = [x;y;z;1];
% Transformation = horzcat(Rotation,Position);

%converting the orientation position to a rotation matrix using eul2rotm
rotation_matrix = eul2rotm([deg2rad(alpha),deg2rad(beta), deg2rad(gamma)], 'XYZ');
position = [x;y;z];
T_base_tool = [rotation_matrix position; [0 0 0 1]];


%% t1. The 3 first joint angles are found trough the geometric method
t1_1 = atan2(y, x);
t1_2 = t1_1 + pi;

%% Exclude tip and lower part of system
d = 29;
b = 101;
a = 35;
px = x - d*Rotation(1,3)-a*cos(t1_1);
py = y - d*Rotation(2,3) -a*sin(t1_1);
pz = z - d*Rotation(3,3)-b;

%% t3
a1 = 120;
a3 = 23;
a2 = sqrt(23^2 + 120^2);

ang= atan2(23,120);

t3_1 = acos((px^2 + py^2 + pz^2 - a1^2- a2^2) /(-2*a1*a2)) - ang - pi/2; % satt på en minus her. SE PÅ hvorfor virus
t3_2 = - t3_1; % avrundingsfeil

%% t2
% antar her at lille dritvinkelen er rett og bruker pytagoras ut til hakket
% før e-e
beta = atan2(pz,sqrt(px^2 + py^2));
zeta = acos((-120^2-143^2-23^2)/(-2*120*sqrt(120^2+143^2))); % cos-setning. fra t2 til t5 TROR det er en feil her

%zeta = acos((px^2 + py^2 + pz^2 + a1^2 - a2^2) /(2*a1*sqrt(px^2+pz^2)));
if(t3_1 < 0)
    t2_1 = beta + zeta;
    t2_2 = beta - zeta;
else
    t2_1 = beta - zeta;
    t2_2 = beta + zeta;
end

%% t4. Using algebraic method on joint angle 4,5,6.

%Create T_0_3


% %Algebraic method
% T_0_3 = matrixT03(theta1_1, theta2_1, theta3_1);
% T = inv(T_0_3)*TransM;
% 
% %Theta5
% theta5_1=convert2deg(acos(T(1,3)));
% theta5_2= -theta5_1;
% 
% %Theta4 --
% theta4_1=convert2deg(asin(T(2,3)/cos(theta5_1)));
% theta4_1 = round(theta4_1);
% theta4_2= theta4_1 + 180;
% 
% %theta6
% theta6_1= convert2deg(acos(-T(1,1)/sin(theta5_1)));
% theta6_2= theta6_1 + 180;
% 
% 
% 
% %Solutions
% a1 = convert2deg(theta1_1);
% a12 = convert2deg(theta1_2);
% a2 = convert2deg(theta2_1);
% a22 = convert2deg(theta2_2);
% a3 = convert2deg(theta3_1);
% a32 = convert2deg(theta3_2);
% 
% %Solutions 1 for theta4-6
% sol1 = repmat([theta4_1 theta5_1 theta6_1],4,1);
% 
% %Solutions 2 for theta4-6
% sol2 = repmat([theta4_2 theta5_2 theta6_2],4,1);
% 
% T1_2_3 = [a1 a2 a3;a1 a22 a32;a12 a2 a3;a12 a22 a32];
% 
% fprintf('Solutions: ');
% res1 = horzcat(T1_2_3,sol1);
% res2 = horzcat(T1_2_3,sol2);
% head = [{'theta1 '},{'theta2 '},{'theta3 '},{'theta4 '},{'theta5 '},{'theta6 '}];
% result = double([res1; res2]);
% results = array2table(result,'VariableNames',head);
% 
% end
% 
% function [T03] = matrixT03(theta1, theta2, theta3)
% 
% T03 = matrix(0,0,99,theta1)*matrix(40,sym(pi)/2,0,theta2)*matrix(0,-sym(pi)/2,120,0)*matrix(0,sym(pi)/2,0,theta3);
% 
% end

end