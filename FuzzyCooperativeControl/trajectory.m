% % Scenario 1:
% x1 = 0:0.01:17;
% x2 = 0:0.01:20;
% x3 = 0:0.01:25;
% y1 = 10 + 1*cos(1*x1) - 0.5*cos(3*x1);    % initial condition: 9.5
% y2 = 5  + 4*cos(2*x2) + 0.5*sin(5*x2);    % initial condition: 9.5
% y3 = 1  + 3*cos(2*x3) + 1.5*sin(5*x3);    % initial condition: 9.5
% elseif(Exp == 2)
% %Scenario 2:
% x1 = 0:0.01:13;
% x2 = 0:0.01:13;
% x3 = 0:0.01:13;
% y1 = 10 + 4*cos(2*x1) + 0.5*sin(5*x1);    %[x1,y1,phi1] = [1,9,0]
% y2 = 5 + 4*cos(2*x2) + 0.5*sin(5*x2);    %[x2,y2,phi2] = [-1.5,4,0]   
% y3 = 1 + 4*cos(2*x3) + 0.5*sin(5*x3);    %[x3,y3,phi3] = [2,-1.2,0]   
% y1 = polyval(p1,x1);
% y2 = polyval(p2,x2);
% y3 = polyval(p3,x3);

clc; clear all; close all;

x1 = 0:0.01:17;
% x2 = 0:0.01:20;
% x3 = 0:0.01:25;

y1 = 10 + 1*cos(1*x1) - 0.5*cos(3*x1);    % initial condition: 9.5
% y2 = 5  + 4*cos(2*x2) + 0.5*sin(5*x2);    % initial condition: 9.5
% y3 = 1  + 3*cos(2*x3) + 1.5*sin(5*x3);    % initial condition: 9.5

% p = [5 4 3 2 1];
% y1=y1+polyval(p,x1);
% plot(y1)

p1 = polyfit(x1,y1,5);
y1 = polyval(p1,x1);
plot(x1,y1);