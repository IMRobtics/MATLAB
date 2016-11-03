function [trajectory] = generateTrajectory(numberOfRobots)
turns = 1;
x = -1*pi*turns : 0.25  : pi*turns;
r = 0 : 1/(length(x)-1) : 1;

X = sin(x).*r*2;
Y = cos(x).*r*2;
x_g = transpose(X);
y_g = transpose(Y);
x_g=x_g(5:length(x_g)-5);
y_g=y_g(5:length(y_g)-5);

trajectoryOffset = 1.15;
trajectory       = zeros(17,2);
for i=1:numberOfRobots
    trajectory   = [trajectory x_g+((i-1)*trajectoryOffset) y_g+((i-1)*trajectoryOffset)];
end
trajectory=trajectory(:,3:(numberOfRobots*2+2));