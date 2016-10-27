clc; clear all; close all;

% Generate Spiral Trajectory Pattern Points and Line Object
% =========================================================
figure
turns=5;
Floor_MAX = abs(2.5);
xPoints=[-1*pi*turns : 1.5: pi*turns];
r=[0:1/(length(xPoints)-1):1];
X=sin(xPoints).*r*(Floor_MAX-0.5);
Y=cos(xPoints).*r*(Floor_MAX-0.5);
plot(X,Y,'-r','LineWidth',2);
hold on; grid on; 
position = animatedline;
axis([-2.5 2.5 -2.5 2.5]); axis square;

x_g = X; % goal x-co-ordinate
y_g = Y; % goal y-co-ordinate
startPoint = 2;
point = startPoint;
steps = 0;

% Robot / Simulation / Control Related Constants/Variables
% ========================================================
R = 41/2; % in mm K3
L = 88.41; % in mm K3
maxVel=2*pi; % max velocity, v-rep, K3
vLeft = pi;  % initial vLeft
vRight = -pi; % initial vRight
v = maxVel; % constant velocity

LOOP=1;
a=1;
i=1; % for 1 robot

dt=0.05; % 50ms v-rep default simulation time-step

distThresh = 0.05;
angleThresh = 0.2;

% v-Rep remote API Initialization Code
% ====================================
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
else
    disp('Failed connecting to remote API server');
end

% Robot 1 Initialization Code - set initial velocity (zero), get pose
% ==================================================
[~,r1_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#', vrep.simx_opmode_blocking);
setVelocity(i,vLeft,vRight,vrep,clientID);


flag=true;
while flag
    [x,y,theta] = getPose(vrep,clientID,r1_handle);
    formatSpec = 'LOOP=%03d, X=%4.2f, Y=%4.2f, Th=%4.2f\n';
    fprintf(formatSpec,LOOP,x,y,theta)
    flag=false;
end

vLeft = 0;
vRight = 0;
setVelocity(i,vLeft,vRight,vrep,clientID);
vrep.delete();
%disp('Program ended');
