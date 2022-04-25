function [dX] = X_kinematics(X,acc,gyro)
%X_KINEMATICS get dX from current X and IMU data 
%   Detailed explanation goes here
    g_i = [0, 0, 9.807]';
    pos_i_post = X(1:3);
    quat_i_post = X(4:7);
    v_b_post = X(8:10);

    phone_pos = [0,0,-3]; % relative position from rear wheels

    L_bi = quat2dcm(quat_i_post');
    g_b = L_bi * g_i;
    vel_b_turn = cross(gyro, phone_pos);

    d_pos_i = L_bi' * v_b_post;
    d_quat_i = quat_kinematics(gyro',quat_i_post);
    d_v_b = acc' - vel_b_turn';
    
    dX = [d_pos_i;
          d_quat_i;
          d_v_b];
end

