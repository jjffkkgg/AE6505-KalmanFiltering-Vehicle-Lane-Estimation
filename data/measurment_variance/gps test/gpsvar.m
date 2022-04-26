clear;

True = linspace(33.78219,33.78203,105);
data = importdata('GPS.csv');
lat_data = data.data(:,2);

alt = data.data(:,4);
std_alt = std(alt - 283)^2

figure(1)
plot(lat_data)

err = lat_data'-True;

err_std = std(err(5:10:end))^2
err_mean = mean(lat_data'-True)