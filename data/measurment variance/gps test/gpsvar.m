clear;

True = linspace(33.78219,33.78203,105);
data = importdata('GPS.csv');
long_data = data.data(:,2);
figure(1)
plot(long_data)

err_std = std(long_data'-True)
err_mean = mean(long_data'-True)