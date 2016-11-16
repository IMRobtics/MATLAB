clc; clear all; close all; color = 'kbgrcmy'; colorVal=0;

numberOfRobots = 4;
trajectory = generateTrajectory(numberOfRobots);

figure;
hold on; grid on;
axis([-6 6 -6 6]);
axis square; axis equal;

for i=1:2:numberOfRobots*2
   colorVal = colorVal+1;
   if(colorVal>7)
      colorVal=1;
   end
   plot(trajectory(:,i),trajectory(:,i+1),color(colorVal),'LineWidth',2);
end

% Robot and Simulation Constants
R = 41/2;  % in mm
L = 88.41; % in mm
dt=0.05;   % 50ms
maxVel=2*pi;
v = maxVel;

xyz    = zeros(numberOfRobots,3);
eAngle = zeros(numberOfRobots,3);
pose   = zeros(numberOfRobots,3);

vLeft  = zeros(1,numberOfRobots);
vRight = zeros(1,numberOfRobots);


%=================================================
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
else
    disp('Failed connecting to remote API server');
end
%=================================================
[~,r1_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#', vrep.simx_opmode_blocking);
[~,r2_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#0', vrep.simx_opmode_blocking);
[~,r3_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#1', vrep.simx_opmode_blocking);
[~,r4_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#2', vrep.simx_opmode_blocking);
r_handle = [r1_handle r2_handle r3_handle r4_handle];

for i=1:numberOfRobots
    [~,xyz(i,:)]=vrep.simxGetObjectPosition(clientID, r_handle(i), -1, vrep.simx_opmode_streaming);
    [~,eAngle(i,:)]=vrep.simxGetObjectOrientation(clientID,r_handle(i), -1, vrep.simx_opmode_streaming);
    pose(i,:) = [xyz(i,1) xyz(i,2) atan2(sin(eAngle(i,3)), cos(eAngle(i,3)))];
end

robotTrajectory = [animatedline('Color',color(colorVal),'LineWidth',2)];
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal=1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end

LOOP=0;

while LOOP < 200
    LOOP = LOOP+1;
    for i=1:numberOfRobots
        [~,xyz(i,:)]=vrep.simxGetObjectPosition(clientID, r_handle(i), -1, vrep.simx_opmode_buffer);
        [~,eAngle(i,:)]=vrep.simxGetObjectOrientation(clientID,r_handle(i), -1, vrep.simx_opmode_buffer);
        pose(i,:) = [xyz(i,1) xyz(i,2) atan2(sin(eAngle(i,3)), cos(eAngle(i,3)))];

        x = pose(i,1);
        y = pose(i,2);
        theta=pose(i,3) + (pi/2);
        % x,y, theta is CURRENT pose info of ith ROBOT
        addpoints(robotTrajectory(i),x,y);
        
        % DO CALCULATIONS HERE
        
        
        % Set NEW Velocity for ith ROBOT
        [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft(1,i) vRight(1,i)],[],'',[],vrep.simx_opmode_blocking);
    end
    drawnow
end

for i=1:numberOfRobots
    vLeft(1,i) = 0;
    vRight(1,i) = 0;
    [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft(1,i) vRight(1,i)],[],'',[],vrep.simx_opmode_blocking);
end

vrep.delete();