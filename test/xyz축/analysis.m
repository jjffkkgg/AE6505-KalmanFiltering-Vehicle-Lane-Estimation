clear;

rotvec = importdata('RotationVector.csv');

msec = rotvec.data(:,1);
x = rotvec.data(:,2);
y = rotvec.data(:,3);
z = rotvec.data(:,4);
cos = rotvec.data(:,5);

for i=1:2848
    
end