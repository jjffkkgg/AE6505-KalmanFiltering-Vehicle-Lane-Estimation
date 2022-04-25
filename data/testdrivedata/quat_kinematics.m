function [d_quat_i] = quat_kinematics(omega_b,quat_i)
%EULER_KINEMATICS Derivative of euler angles in body frame
%   get body(intertial) frame angle derivatives
    P = omega_b(1);
    Q = omega_b(2);
    R = omega_b(3);
    
    conv = [0, -P, -Q, -R;
            P, 0, R, -Q;
            Q, -R, 0, P;
            R, Q, -P, 0];
        
    d_quat_i = 0.5 * conv * quat_i;
end

