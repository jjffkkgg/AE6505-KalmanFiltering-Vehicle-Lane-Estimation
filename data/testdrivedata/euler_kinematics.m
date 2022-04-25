function [dangle] = euler_kinematics(gyro,euler)
%EULER_KINEMATICS Summary of this function goes here
%   Detailed explanation goes here
phi = euler(1);
theta = euler(2);
psi = euler(3);

R = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
     0, cos(phi), -sin(phi);
     0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

dangle = R * gyro;
end

