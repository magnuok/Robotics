function [ angles ] = backward_kinematics(position, orientation)
%Install Robotics System Toolbox
angles = zeros(1,6);

x = position(1);
y = position(2);
z = position(3);

alpha = orientation(1);
beta = orientation(2);
gamma = orientation(3);

%creating an orientation matrix
orientation_matrix = [deg2rad(alpha),deg2rad(beta), deg2rad(gamma)];
%converting the orientation position to a rotation matrix using eul2rotm
rotation_matrix = eul2rotm(orientation_matrix, 'ZYZ');

position_matrix = [x y z];
pos_5 = position_matrix - 30*rotation_matrix(:, 3)';


z_5 = pos_5(3) - 99;
%calcualting the angle of joint 3 using the cosine-formula
c_3 = (pos_5(1)^2 + pos_5(2)^2 + z_5^2 - 149^2 - 120^2)/(2*149*120);
s_3_1 = sqrt(1 - c_3^2);
%there is two solutions
s_3_2 = - s_3_1;    
angles(3) = rad2deg(atan2(s_3_1, c_3));  
angle_3_1 = angles(3);
angle_3_2 = rad2deg(atan2(s_3_2, c_3));


%calculating the second angles, which have 4 possible solutions
c_2_1_1 = (sqrt(pos_5(1)^2 + pos_5(2)^2)*(120 + 149*c_3) + z_5*149*s_3_1)/(120^2 + 149^2 + 2*120*149*c_3);
c_2_2_1 = (-sqrt(pos_5(1)^2 + pos_5(2)^2)*(120 + 149*c_3) + z_5*149*s_3_1)/(120^2 + 149^2 + 2*120*149*c_3);

c_2_1_2 = (sqrt(pos_5(1)^2 + pos_5(2)^2)*(120 + 149*c_3) + z_5*149*s_3_2)/(120^2 + 149^2 + 2*120*149*c_3);
c_2_2_2 = (-sqrt(pos_5(1)^2 + pos_5(2)^2)*(120 + 149*c_3) + z_5*149*s_3_2)/(120^2 + 149^2 + 2*120*149*c_3);

s_2_1_1 = (sqrt(pos_5(1)^2 + pos_5(2)^2)*149*s_3_1 + z_5*(149*c_3 + 120))/(120^2 + 149^2 +2*120*149*c_3);
s_2_2_1 = (-sqrt(pos_5(1)^2 + pos_5(2)^2)*149*s_3_1 + z_5*(149*c_3 + 120))/(120^2 + 149^2 +2*120*149*c_3);

s_2_1_2 = (sqrt(pos_5(1)^2 + pos_5(2)^2)*149*s_3_2 + z_5*(149*c_3 + 120))/(120^2 + 149^2 +2*120*149*c_3);
s_2_2_2 = (-sqrt(pos_5(1)^2 + pos_5(2)^2)*149*s_3_2 + z_5*(149*c_3 + 120))/(120^2 + 149^2 +2*120*149*c_3);


angles(2) = - rad2deg(atan2(s_2_2_1, c_2_1_1));
%The second angle got 4 possible solutions:
angle_2_1 = - rad2deg(atan2(s_2_1_1, c_2_2_1));
angle_2_2 = - rad2deg(atan2(s_2_2_1, c_2_1_1));
angle_2_3 = - rad2deg(atan2(s_2_2_2, c_2_1_2));
angle_2_4 = - rad2deg(atan2(s_2_1_2, c_2_2_2));

%Calculating the first angel, which have 2 possible solutions
angles(1) = rad2deg(atan2(pos_5(2), pos_5(1)));
angle_1_1 = rad2deg(atan2(pos_5(2), pos_5(1)));
angle_1_2 = rad2deg(atan2(-pos_5(2), -pos_5(1)));

D_H = [  pi/2        0      0    angles(1)*pi/180;
		  0      120   0  angles(2)*pi/180;
		0       149    0     angles(3)*pi/180];
    
%4x4x3 cube of zeros.     
i_T=zeros(4,4,3);
%making a 4x4 matrix of zeros where the diagonal is 1's
T_0_5=eye(4);

for i=1:3
	i_T(:,:,i)=transform(D_H(i,:));
	T_0_5=T_0_5*i_T(:,:,i);
end

R_0_5 = T_0_5(1:3,1:3);

R_5_6 = R_0_5'*rotation_matrix;

%calcuating the 4th angel, which only got one solution
angles(4) = rad2deg(atan2(R_5_6(2,3), R_5_6(1,3)));
angle_4 = angles(4);

pos_5_x = x - pos_5(1);
pos_5_y = y - pos_5(2);
pos_5_z = z - pos_5(3);

z_5_2 = sqrt(pos_5_z^2+pos_5_y^2) * sin(3*pi/2);
angles(5) = rad2deg(atan2(z_5_2, pos_5_x));
angle_5 = angles(5);
end

function T = transform(input)

T = [     cos(input(4))                  -sin(input(4))                 0                   input(2);
	sin(input(4))*cos(input(1)) cos(input(4))*cos(input(1)) -sin(input(1)) -sin(input(1))*input(3);
	sin(input(4))*sin(input(1)) cos(input(4))*sin(input(1))  cos(input(1))  cos(input(1))*input(3);
	0                                   0                         0                       1];
end
