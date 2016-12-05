clc; clear all; close all; color = 'kbgrcmy'; colorVal=1;

figure; hold on; grid on;
axis([-8 35 -8 35]); axis square;

x0 = 20;
y0 = 20;
v1=3; v2=2;
rectWidth =v1*2;
rectHeight=v2*2;

%Draw a rectangle
rectangle('Position',[x0-v1 y0-v2 rectWidth rectHeight]);%,'FaceColor',[1 0 0]);
ellipse(x0,y0,3,2);
ellipse(x0,y0,6,4);

x=[-1:.1:1];
y=[sqrt(1-x.^2);-sqrt(1-x.^2)]
plot(x,y)

return