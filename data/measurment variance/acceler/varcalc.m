clear;

x_data = importdata('x.csv');
y_data = importdata('y.csv');
z_data = importdata('z.csv');

x = x_data.data(:,2);
y = y_data.data(:,3);
z = z_data.data(:,4);

cov_mat = [std(x), 0, 0;
            0, std(y), 0;
            0, 0, std(z)];
g = 9.807;  % m/s

diff_x = mean(x)-g;
diff_y = mean(y)-g;
diff_z = mean(z)-g;
