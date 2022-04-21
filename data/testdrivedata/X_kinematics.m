function [dX] = X_kinematics(X,acc,gyro)
%X_KINEMATICS get dX from current X and IMU data 
%   Detailed explanation goes here
    pos_b_post = X(1:3);
    quat_b_post = X(4:7);
    v_p_post = X(8:10);
%     omega_p_post = X(end-2:end);
    
    d_pos_b = quat2dcm(quat_b_post') * v_p_post;
    d_quat_b = quat_kinematics(gyro',quat_b_post);
    d_v_p = acc';
%     d_omega_p = gyro';
    
    dX = [d_pos_b;
          d_quat_b;
          d_v_p];
%           d_omega_p];
end

