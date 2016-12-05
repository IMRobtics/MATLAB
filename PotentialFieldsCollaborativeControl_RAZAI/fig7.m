clc; clear all; close all; color = 'kbgrcmy'; colorVal=1;

figure; hold on; grid on;
axis([10 55 10 55]); axis square;

% Generate Random Start Points for Robots
numberOfRobots=4;

robot=[ ([15,15])
        ([15,20])
        ([20,20])
        ([20,15])];

robotTminus1 = robot;
robotTminus2 = robot;

for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2);
end

% Create Animated Line Objects for Each robot, different colors
robotTrajectory = [animatedline('Color',color(colorVal),'LineWidth',2)];
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal=1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end

M=1;
B=1;
kd=9;
ksk=1;
kr=0.15;
alpha=3;
qk=10;

%Charges on each robot
q(1:numberOfRobots)=qk;
%Mass of each robot
m(1:numberOfRobots)=M;
%Electrostatic Constant
%K=10;

% Force Threshold, robots stop moving if force < threshold
FORCE_THRESHOLD = 0.01;
FORCE_MAX = 100;

xdot = zeros(numberOfRobots,1);
ydot = zeros(numberOfRobots,1);

% Load First Trajectory point and Draw Circle Around it
VirtualBot=[0,0];
circle(VirtualBot(1,1),VirtualBot(1,2),alpha);
VirtualTrajectory = animatedline('Color','r','LineWidth',2,'LineStyle','-.')

Fxk_dist = zeros(numberOfRobots,numberOfRobots);
Fxk = zeros(1,numberOfRobots);
Attractive_Force_x = zeros(1,numberOfRobots);
FxkVS = zeros(1,numberOfRobots);

Fyk_dist = zeros(numberOfRobots,numberOfRobots);
Fyk = zeros(1,numberOfRobots);
Attractive_Force_y = zeros(1,numberOfRobots);
FykVS = zeros(1,numberOfRobots);

