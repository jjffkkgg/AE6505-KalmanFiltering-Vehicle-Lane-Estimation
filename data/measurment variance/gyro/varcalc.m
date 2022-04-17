clear;

data = importdata('Gyroscope.csv');

x = data.data(:,2);
y = data.data(:,3);
z = data.data(:,4);
figure(1)
plot(z)

cov_mat = [std(x), 0, 0;
            0, std(y), 0;
            0, 0, std(z)];