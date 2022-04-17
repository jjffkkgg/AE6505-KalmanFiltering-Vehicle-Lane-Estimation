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

X0 = [0;        % x [m]
      0;        % y [m]
      0;        % v_x [m/s]
      0];       % v_y [m/s]
      
Y0 = [33.78207;         % Longitude
      -84.39507];       % Latitude
      
angle_0 = pi/2;
dcm_0 = [cos(angle_0), -sin(angle_0);
        sin(angle_0), cos(angle_0)];
    


%% Phase 2 [Non-highway]
      