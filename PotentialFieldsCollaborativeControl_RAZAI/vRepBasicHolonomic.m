clear all; close all; color = 'kbgrcmy'; colorVal=1;
figure;
hold on; grid on;
FLOORSIZE = 7.5; % in meters HALF
axis([-FLOORSIZE FLOORSIZE -FLOORSIZE FLOORSIZE]);
axis square; axis equal;

numberOfRobots = 2;
vX = zeros(1,numberOfRobots);
vY = zeros(1,numberOfRobots);

maxVel=1000;
stopVel=500;
% scalling is done on V-Rep side, here MUST not go above 1000
% for forward and backward movement - 500 is STOP - 0 is BACK full speed
% and 1000 is forward full speed.
% similar for Left (0) and Right(1000)

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
[~,r1_handle]=vrep.simxGetObjectHandle(clientID,'youBot#', vrep.simx_opmode_blocking);
[~,r2_handle]=vrep.simxGetObjectHandle(clientID,'youBot#0', vrep.simx_opmode_blocking);
% [~,r3_handle]=vrep.simxGetObjectHandle(clientID,'youBot#1', vrep.simx_opmode_blocking);
% [~,r4_handle]=vrep.simxGetObjectHandle(clientID,'youBot#2', vrep.simx_opmode_blocking);
r_handle = [r1_handle r2_handle]; % r3_handle r4_handle];

for i=1:numberOfRobots
    [~,xyz(i,:)]=vrep.simxGetObjectPosition(clientID, r_handle(i), -1, vrep.simx_opmode_blocking);
    [~,eAngle(i,:)]=vrep.simxGetObjectOrientation(clientID,r_handle(i), -1, vrep.simx_opmode_blocking);
    pose(i,:) = [xyz(i,1) xyz(i,2) atan2(sin(eAngle(i,3)), cos(eAngle(i,3)))];
    
    x = pose(i,1);
    y = pose(i,2);
    theta=pose(i,3) + (pi/2);

    robot(i,:) = [x y theta];
end

robotTrajectory = [animatedline('Color',color(colorVal),'LineWidth',2)];
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal=1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end     
    
for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2,2);
end

i=1;

pause
vX(1,i) = stopVel;
vY(1,i) = maxVel;
% set vl and vr of each robot
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setVelocity_function',[i vX(1,i) vY(1,i)],[],'',[],vrep.simx_opmode_blocking);

pause
vX(1,i) = maxVel;
vY(1,i) = stopVel;
% set vl and vr of each robot
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setVelocity_function',[i vX(1,i) vY(1,i)],[],'',[],vrep.simx_opmode_blocking);

pause
vX(1,i) = 0;
vY(1,i) = 0;
% set vl and vr of each robot
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setVelocity_function',[i vX(1,i) vY(1,i)],[],'',[],vrep.simx_opmode_blocking);

pause
vX(1,i) = stopVel;
vY(1,i) = stopVel;
% set vl and vr of each robot
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setVelocity_function',[i vX(1,i) vY(1,i)],[],'',[],vrep.simx_opmode_blocking);