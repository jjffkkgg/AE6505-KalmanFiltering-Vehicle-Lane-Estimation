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

% Initialization

% orientation
% body frame NED
angle_0 = [-pi/2, 0, pi/2]; % initial angle from NED, Z-Y-X from phone to body frame
dcm_0 = angle2dcm(angle_0(1),angle_0(2),angle_0(3),'ZYX');
% quat_0 = angle2quat(angle_0(1),angle_0(2),angle_0(3),'ZYX');

X0 = [0;            % x [m] (NED)
      0;            % y [m] (NED)
      0;            % z [m] (NED)
      angle_0';     % orientation [rad] (NED euler)
      0;            % v_x [m/s] (phone frame)
      0;            % v_y [m/s] (phone frame)
      0;            % v_z [m/s] (phone frame)
      zeros(3,1)];  % angular vel [rad/s] (phone frame)
      
Y0 = [33.78207;         % Latitude
      -84.39507         % Longitude
      278];             % Altitude
  
P0 = diag([0,0,0,0,0,0,0.0132,0.0116,0.0099,0.0867,0.0195,0.07]);
R = diag([

for k = 1:data_size
    if k == 1
        
    end
end

%% Phase 2 [Non-highway]
      