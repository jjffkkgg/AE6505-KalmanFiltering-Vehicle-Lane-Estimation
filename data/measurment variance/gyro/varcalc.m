clear;

data = importdata('Gyroscope.csv');
data = data.data;
[data_size,~] = size(data);
% x = data.data(:,2);
% y = data.data(:,3);
% z = data.data(:,4);
% figure(1)
% plot(z)
% 
% cov_mat = [std(x)^2, 0, 0;
%             0, std(y)^2, 0;
%             0, 0, std(z)^2]
for i = 1:data_size
    d_quat(i,:) = quat_kinematics(data(i,2:4), angle2quat(-pi/2, 0, pi/2,'ZYX')')';
end
std(d_quat(1,:))
std(d_quat(2,:))
std(d_quat(3,:))
std(d_quat(4,:))
% d_quat
% data