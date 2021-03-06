%% Phase 1 [I-85]

clear;

gyro = importdata('./first/Gyroscope.csv');
gps = importdata('./first/GPS.csv');
acc = importdata('./first/Accelerometer.csv');

start_index = 2493;
end_index = 13475;
gps_delay = 2   ; % [s]
gps_delay_index = gps_delay*10; % [s]

data_gyro = gyro.data(1:end_index,2:end);
data_gps = gps.data(1+gps_delay_index:end_index+gps_delay_index,2:end);
data_acc = acc.data(1:end_index,2:end);

[data_size,~] = size(data_gps);

dt = 0.1;    % [s]

% Initialization

% orientation
% body frame NED
angle_0 = [-pi/2, 0, pi/2]; % initial frame angle from NED, Z-Y-X to phone body frame (NED -> phone)
dcm_0 = angle2dcm(-angle_0(1),-angle_0(2),-angle_0(3),'ZYX');  % following DCM in AircraftAndSim book
quat_0 = angle2quat(-angle_0(1),-angle_0(2),-angle_0(3),'ZYX');

X0 = [0;            % x [m] (NED)
      0;            % y [m] (NED)
      0;            % z [m] (NED)
      quat_0';      % orientation [rad] (NED quat)
      0;            % v_x [m/s] (vehicle frame)
      0;            % v_y [m/s] (vehicle frame)
      0];            % v_z [m/s] (vehicle frame)
%       zeros(3,1)];  % angular vel [rad/s] (vehicle frame)
      % (10,1)

[n,~] = size(X0);
      
Y0 = [33.78207;         % Latitude
      -84.39507         % Longitude
      278];             % Altitude
  
%%% Variance & bias correction

g = 9.807;
  
P0 = diag([100,100,100,...
            0.0001,0.00001,0.00001,0.00001,...
            0.1,0.1,0.1]);
%             deg2rad(10),deg2rad(10),deg2rad(10)]);
        
Q = diag([1.755e-2,1.352e-2,0.983e-2,...
        9.3639e-05,2.7136e-06,2.6480e-07,7.0453e-08,...
        1.755e-2,1.352e-2,0.983e-2]);
%         0.0867,0.0195,0.07]);       % State error covariance matrix
    
R = diag([1.1964e-12,1.1964e-12,0.0947]);   % measurment error covariance matrix

data_acc(:,1) = data_acc(:,1) - 0.0251;
data_acc(:,2) = data_acc(:,2) + 0.1228 - g;
data_acc(:,3) = data_acc(:,3) - 0.0409;

% data_gyro(:,1) = data_acc(:,1) + 0.0023;
% data_gyro(:,2) = data_acc(:,2) - 9.5031e-4;
% data_gyro(:,3) = data_acc(:,3) - 0.0047;

% Unscented Kalman Filter Algorithm
tic
for k = 1:data_size
    if k == 1
        X_post = X0;
        P_post = P0;
        prop_nums = 0;  % detect 1s update of gps even if it do not move
        gps_prev = Y0';
        X_est = [X0];
        y_est = [Y0];
    else
        gps_prev = data_gps(k-1,:);
        X_post = X_post_new;
        P_post = P_post_new;
    end
    
    acc_k = data_acc(k,:);      % (1,3)
    gyro_k = data_gyro(k,:);    % (1,3)
    gps_k = data_gps(k,:);      % (1,3)

    P_var = diag(P_post);
    pos_var = P_var(1:3);

    gps_diffalt = gps_k(3) - gps_prev(3);   % use diff for alt b/c lat long diff is too small
    gps_diff2d = norm(gps_k(1:2) - gps_prev(1:2));

    % GPS ignorance condition
%     propagate = 1;                          % mere IMU propagation
%     propagate = ((gps_diff2d == 0) && (prop_nums < 10));     % not filtering 
    propagate = (gps_diff2d > 3*sqrt(sum(pos_var))) ||...
            ((gps_diff2d == 0) && (prop_nums < 30));
    
    %%%%%%%%%%%%%%%%%%%%%%%%% UKF Filtering %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% Time update %%%%%%%
    % 1. generate sigma point
    [root,p] = chol(n*P_post);
    
    for j = 1:n
        sigma(:,j) = X_post + root(j,:)';
        sigma(:,j+n) = X_post - root(j,:)';
    end
    
    % 2. propagate with nonlin
     for j = 1:2*n
         dX = X_kinematics(sigma(:,j), acc_k, gyro_k)*dt;
         prop_sigma(:,j) = sigma(:,j) + dX;
     end
     
     % 3. generate apriori of x
     X_prior = zeros(n,1);
     for j = 1:2*n
         X_prior = X_prior + prop_sigma(:,j);
     end
     X_prior = X_prior./(2*n);

     % 4. generate apriori of P
     P_prior = zeros(n,n);
     for j = 1:2*n
         P_prior = P_prior + (prop_sigma(:,j) - X_prior)*...
                             (prop_sigma(:,j) - X_prior)';
     end
     P_prior = P_prior/(2*n) + Q;
     
     if propagate
         X_post_new = X_prior;
         P_post_new = P_prior;
         prop_nums = prop_nums + 1;
         
     else
         %%%%%%% Measurment update %%%%%%% (updated every 1s with gps)
         % 1. generate sigma points
         [root,p] = chol(n*P_prior);

        for j = 1:n
            sigma(:,j) = X_prior + root(j,:)';
            sigma(:,j+n) = X_prior - root(j,:)';
        end

        % 2. generate measurment to sigma points
        for j = 1:2*n
            dsigma_y = ned2gps(sigma(:,j), Y0);
            sigma_y(:,j) = Y0 + dsigma_y;
        end

        % 3. combine to generate predicted meas at time t_k
        y_hat = zeros(3,1);
         for j = 1:2*n
             y_hat = y_hat + sigma_y(:,j);
         end
         y_hat = y_hat/(2*n);

         % 4. generate measurement variance
%          R = diag([25,deg2rad(0.1)^2,deg2rad(0.1)^2]);
         Py = zeros(3,3);
         for j = 1:2*n
             Py = Py + (sigma_y(:,j) - y_hat)*...
                             (sigma_y(:,j) - y_hat)';
         end
         Py = Py/(2*n) + R; % (3*3)

         % 5. estimate cross cov between x,y
         Pxy = zeros(n,3);
         for j = 1:2*n
             Pxy = Pxy + (prop_sigma(:,j) - X_prior)*...
                             (sigma_y(:,j) - y_hat)'; % (13*1)*(1*3)
         end
         Pxy = Pxy/(2*n); % (6*3)

         % 6. Kalman Gain
         K = Pxy*pinv(Py); % (6*3)

         % 7. compute Measurement update
         X_post_new = X_prior + K*(gps_k' - y_hat);
         P_post_new = P_prior - K * Py * K';
         
         prop_nums = 0;
     end
     X_est = [X_est, X_post_new];
     dy = ned2gps(X_post_new,Y0);
     y_est = [y_est, dy+Y0];
end
toc

%%% plot the result

figure(1)
plot3(X_est(2,1:end),X_est(1,1:end),-X_est(3,1:end))
grid on
xlabel('E')
ylabel('N')
zlabel('H')
figure(2)
plot(X_est(2,1:end),X_est(1,1:end))
grid on
xlabel('E')
ylabel('N')

figure(3)
geoplot(y_est(1,:),y_est(2,:),'r','LineWidth',2)
geobasemap('satellite')
title('Satelite Plotting Path 1')

hold on
geoplot(data_gps(:,1),data_gps(:,2),'b.','LineWidth',2)
hold off
legend('Filtered','GPS plotting')

[~,y_index]=size(y_est);
num_csvs = fix(y_index / 2000);

for i=1:num_csvs-1
    y_est(:,i*2000+1:(i+1)*2000+1);
end

% for i = 1:num_csvs-1
%     writematrix(y_est(:,num_csvs*2000+1:(num_csvs+1)*2000+1),'Est_y.csv')
% end


%% Phase 2 [Non-highway]

clear;
gyro = importdata('./second/Gyroscope.csv');
gps = importdata('./second/GPS.csv');
acc = importdata('./second/Accelerometer.csv');

start_index = 1000;
end_index = 11500;
gps_delay = 0; % [s]
gps_delay_index = gps_delay*10; % [s]

data_gyro = gyro.data(1:end_index,2:end);
data_gps = gps.data(1+gps_delay_index:end_index+gps_delay_index,2:end);
data_acc = acc.data(1:end_index,2:end);

[data_size,~] = size(data_gps);

dt = 0.1;    % [s]

% Initialization

% orientation
% body frame NED
angle_0 = [pi/2, 0, deg2rad(-50)]; % initial angle from NED, Z-Y-X from phone to body frame
% dcm_0 = angle2dcm(angle_0(1),angle_0(2),angle_0(3),'ZYX');
quat_0 = angle2quat(-angle_0(1),-angle_0(2),-angle_0(3),'ZYX');

X0 = [0;            % x [m] (NED)
      0;            % y [m] (NED)
      0;            % z [m] (NED)
      quat_0';      % orientation [rad] (NED quat)
      0;            % v_x [m/s] (phone frame)
      0;            % v_y [m/s] (phone frame)
      0];            % v_z [m/s] (phone frame)
%       zeros(3,1)];  % angular vel [rad/s] (phone frame)
      % (10,1)

[n,~] = size(X0);
      
Y0 = [33.908439;         % Latitude
     -84.287319;         % Longitude
      311];             % Altitude
  
%%% Variance & bias correction

g = 9.807;
  

P0 = diag([100,100,100,...
            0.0001,0.0001,0.0001,0.0001,...
            0.1,0.1,0.1]);
%             deg2rad(10),deg2rad(10),deg2rad(10)]);
        
Q = diag([1.755e-2,1.352e-2,0.983e-2,...
        9.3639e-05,2.7136e-06,2.6480e-07,7.0453e-08,...
        1.755e-2,1.352e-2,0.983e-2]);
    
R = diag([1.1964e-12,1.1964e-12,0.0947]);   % measurment error covariance matrix

data_acc(:,1) = data_acc(:,1) - 0.0251;
data_acc(:,2) = data_acc(:,2) + 0.1228 - g;
data_acc(:,3) = data_acc(:,3) - 0.0409;

% data_gyro(:,1) = data_acc(:,1) + 0.0023;
% data_gyro(:,2) = data_acc(:,2) - 9.5031e-4;
% data_gyro(:,3) = data_acc(:,3) - 0.0047;

% Unscented Kalman Filter Algorithm
tic
for k = 1:data_size
    if k == 1
        X_post = X0;
        P_post = P0;
        prop_nums = 0;  % detect 1s update of gps even if it do not move
        gps_prev = Y0';
        X_est = [X0];
        y_est = [Y0];
    else
        gps_prev = data_gps(k-1,:);
        X_post = X_post_new;
        P_post = P_post_new;
    end
    
    acc_k = data_acc(k,:);      % (1,3)
    gyro_k = data_gyro(k,:);    % (1,3)
    gps_k = data_gps(k,:);      % (1,3)

    P_var = diag(P_post);
    pos_var = P_var(1:3);

    gps_diffalt = gps_k(3) - gps_prev(3);   % use diff for alt b/c lat long diff is too small
    gps_diff2d = norm(gps_k(1:2) - gps_prev(1:2));

%    % GPS ignorance condition
%     propagate = 1;                          % mere IMU propagation
    propagate = ((gps_diff2d == 0) && (prop_nums < 10));     % not filter out 3sigma
%     propagate = (gps_diff2d > 3*sqrt(sum(pos_var))) ||...
%             ((gps_diff2d == 0) && (prop_nums < 30));
    
    %%%%%%%%%%%%%%%%%%%%%%%%% UKF Filtering %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%% Time update %%%%%%%
    % 1. generate sigma point
    [root,p] = chol(n*P_post);
    
    for j = 1:n
        sigma(:,j) = X_post + root(j,:)';
        sigma(:,j+n) = X_post - root(j,:)';
    end
    
    % 2. propagate with nonlin
     for j = 1:2*n
         prop_sigma(:,j) = sigma(:,j) + X_kinematics(sigma(:,j), acc_k, gyro_k)*dt;
     end
     
     % 3. generate apriori of x
     X_prior = zeros(n,1);
     for j = 1:2*n
         X_prior = X_prior + prop_sigma(:,j);
     end
     X_prior = X_prior/(2*n);

     % 4. generate apriori of P
     P_prior = zeros(n,n);
     for j = 1:2*n
         P_prior = P_prior + (prop_sigma(:,j) - X_prior)*...
                             (prop_sigma(:,j) - X_prior)';
     end
     P_prior = P_prior/(2*n) + Q;
     
     if propagate
         X_post_new = X_prior;
         P_post_new = P_prior;
         prop_nums = prop_nums + 1;
         
     else
         %%%%%%% Measurment update %%%%%%% (updated every 1s with gps)
         % 1. generate sigma points
         [root,p] = chol(n*P_prior);

        for j = 1:n
            sigma(:,j) = X_prior + root(j,:)';
            sigma(:,j+n) = X_prior - root(j,:)';
        end

        % 2. generate measurment to sigma points
        for j = 1:2*n
            dsigma_y = ned2gps(sigma(:,j), Y0);
            sigma_y(:,j) = Y0 + dsigma_y;
        end

        % 3. combine to generate predicted meas at time t_k
        y_hat = zeros(3,1);
         for j = 1:2*n
             y_hat = y_hat + sigma_y(:,j);
         end
         y_hat = y_hat/(2*n);

         % 4. generate measurement variance
%          R = diag([25,deg2rad(0.1)^2,deg2rad(0.1)^2]);
         Py = zeros(3,3);
         for j = 1:2*n
             Py = Py + (sigma_y(:,j) - y_hat)*...
                             (sigma_y(:,j) - y_hat)';
         end
         Py = Py/(2*n) + R; % (3*3)

         % 5. estimate cross cov between x,y
         Pxy = zeros(n,3);
         for j = 1:2*n
             Pxy = Pxy + (prop_sigma(:,j) - X_prior)*...
                             (sigma_y(:,j) - y_hat)'; % (13*1)*(1*3)
         end
         Pxy = Pxy/(2*n); % (6*3)

         % 6. Kalman Gain
         K = Pxy*pinv(Py); % (6*3)

         % 7. compute Measurement update
         X_post_new = X_prior + K*(gps_k' - y_hat);
         P_post_new = P_prior - K * Py * K';
         
         prop_nums = 0;
     end
     X_est = [X_est, X_post_new];
     dy = ned2gps(X_post_new,Y0);
     y_est = [y_est, dy+Y0];
end
toc

figure(4)
plot3(X_est(2,1:end),X_est(1,1:end),-X_est(3,1:end))
grid on
xlabel('E')
ylabel('N')
zlabel('H')

figure(5)
plot(X_est(2,1:end),X_est(1,1:end))
grid on
xlabel('E')
ylabel('N')

figure(6)
geoplot(y_est(1,:),y_est(2,:),'r','LineWidth',2)
geobasemap('satellite')
title('Satelite Plotting Path 2')

hold on
geoplot(data_gps(:,1),data_gps(:,2),'b.','LineWidth',2)
legend('Filtered','GPS plotting')
hold off