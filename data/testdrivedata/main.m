%% Phase 1 [I-85]

clear;

gyro = importdata('./first/Gyroscope.csv');
gps = importdata('./first/GPS.csv');
acc = importdata('./first/Accelerometer.csv');

start_index = 2493;
end_index = 13475;

data_gyro = gyro.data(1:end_index,2:end);
data_gps = gps.data(1:end_index,2:end);
data_acc = acc.data(1:end_index,2:end);

[data_size,~] = size(data_gps);

dt = 0.1;    % [s]

% Initialization

% orientation
% body frame NED
angle_0 = [-pi/2, 0, pi/2]; % initial angle from NED, Z-Y-X from phone to body frame
dcm_0 = angle2dcm(angle_0(1),angle_0(2),angle_0(3),'ZYX');
quat_0 = angle2quat(angle_0(1),angle_0(2),angle_0(3),'ZYX');

X0 = [0;            % x [m] (NED)
      0;            % y [m] (NED)
      0;            % z [m] (NED)
      quat_0';     % orientation [rad] (NED quat)
      0;            % v_x [m/s] (phone frame)
      0;            % v_y [m/s] (phone frame)
      0;            % v_z [m/s] (phone frame)
      zeros(3,1)];  % angular vel [rad/s] (phone frame)
      
Y0 = [33.78207;         % Latitude
      -84.39507         % Longitude
      278];             % Altitude
  
% Variance & bias correction

g = 9.807;
  
P0 = diag([0,0,0,0,0,0,0,0.0132,0.0116,0.0099,0.0867,0.0195,0.07]);
R = diag([4.6912e-06,1.2049e-06,0.3077]);

data_acc(:,2) = data_acc(:,2) - 0.0251;
data_acc(:,3) = data_acc(:,3) + 0.1228 - g;
data_acc(:,4) = data_acc(:,4) - 0.0409;

% Unscented Kalman Filter Algorithm
for k = 1:data_size
    if k == 1
        
        pos_b = X0(1:3);
        quat_b = X0(4:7);
        v_p = X0(8:10);
        omega_p = X0(end-2:end)';
        P_post = P0;
    end
    
    acc_k = data_acc(k,2:4);
    gyro_k = data_gyro(k,2:4);
    gps_k = data_gps(k,2:4);
    

    d_pos_b = angle2dcm(angle_b(1),angle_b(2),angle_b(3),'ZYX') * v_p;
    d_quat_b = quat_kinematics(omega_p,quat_b);
    d_v_p = acc_k';
    d_omega_p = gyro_k';
    
    pos_b_prior = d_pos_b*dt + pos_b;
    quat_b_prior = d_quat_b*dt + quat_b;
    v_p_prior = d_v_p*dt + v_p;
    omega_p_prior = d_omega_p*dt + omega_p;
    
    %%%% Time update
    % 1. generate sigma point
    [root,p] = chol(n*P_post);
end

%% Phase 2 [Non-highway]
      