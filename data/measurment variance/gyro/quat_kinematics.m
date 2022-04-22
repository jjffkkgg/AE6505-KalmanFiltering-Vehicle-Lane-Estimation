function [d_quat_b] = quat_kinematics(omega_p,quat_b)
%EULER_KINEMATICS Derivative of euler angles in body frame
%   get body(intertial) frame angle derivatives
    P = omega_p(1);
    Q = omega_p(2);
    R = omega_p(3);
    
    conv = [0, -P, -Q, -R;
            P, 0, R, -Q;
            Q, -R, 0, P;
            R, Q, -P, 0];
        
    d_quat_b = 0.5 * conv * quat_b;
end

