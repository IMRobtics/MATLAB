clc; clear all; close all;
disp('Started');

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
else
    disp('Failed connecting to remote API server');
end

R = 41/2; % in mm
L = 88.41; % in mm

%=================================================

LOOP = 1;
maxVel=2*pi;
xyz = zeros(1,3);
eAngle = zeros(1,3);
pose = zeros(1,3);

a=1;
i=1;
vLeft = 0;
vRight = 0;
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
[~,r1_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#', vrep.simx_opmode_blocking);
[~,xyz(a,:)]=vrep.simxGetObjectPosition(clientID, r1_handle, -1, vrep.simx_opmode_streaming);
[~,eAngle(a,:)]=vrep.simxGetObjectOrientation(clientID,r1_handle, -1, vrep.simx_opmode_streaming);
pose(a,:) = [xyz(a,1) xyz(a,2) eAngle(a,3)];

position = animatedline;
axis([-2.5 2.5 -2.5 2.5])
grid on

while LOOP<300
    [~,xyz(a,:)]=vrep.simxGetObjectPosition(clientID, r1_handle, -1, vrep.simx_opmode_buffer);
    [~,eAngle(a,:)]=vrep.simxGetObjectOrientation(clientID,r1_handle, -1, vrep.simx_opmode_buffer);
    pose(a,:) = [xyz(a,1) xyz(a,2) eAngle(a,3)];
  
    % Get estimate of current pose
    x = pose(a,1);
    y = pose(a,2);
    theta = pose(a,3)+(pi/2);
    theta = atan2(sin(theta), cos(theta));
    vRight = 1; %0.14; %(2*v+w*L)/(2*R);
    vLeft = -1;  %0.48; %(2*v-w*L)/(2*R);
    
    formatSpec = 'X=%4.2f, Y=%4.2f, Th=%4.2f, e1=%4.2f, e2=%4.2f, e3=%4.2f\n';
    fprintf(formatSpec,x,y,theta,eAngle(a,1),eAngle(a,2),eAngle(a,3))
    
    [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
    addpoints(position,x,y);
    drawnow
    LOOP = LOOP+1;
end

vLeft = 0;
vRight = 0;
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
vrep.delete();
%disp('Program ended');
