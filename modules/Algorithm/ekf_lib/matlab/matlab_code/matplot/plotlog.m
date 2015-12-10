%read raw sensor data
M = csvread('I:\0000.DAT.csv',2);
[length,line]=size(M);
%plot curve
x_aixs=1:length;
figure(1)
plot(x_aixs,-M(:,5),'g',x_aixs,M(:,16),'r',x_aixs,M(:,27),'b')
figure(2)
plot(x_aixs,-M(:,6),'g',x_aixs,M(:,17),'r',x_aixs,M(:,28),'b')
figure(3)
plot3(M(:,24),M(:,25),M(:,26))