clc;
clear all;
close all;
disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
else
    disp('Failed connecting to remote API server');
end

LOOP=100;
pos = zeros(LOOP,3);
eAngle = zeros(LOOP,3);

i=1;
vLeft = 0;
vRight = 0;
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
[~,r1_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#', vrep.simx_opmode_blocking);

a=1;
[~,pos(a,:)]=vrep.simxGetObjectPosition(clientID, r1_handle, -1, vrep.simx_opmode_blocking);
[~,eAngle(a,:)]=vrep.simxGetObjectOrientation(clientID,r1_handle, -1, vrep.simx_opmode_blocking);
eAngle(1,:)
while (pos(a,1) < 2) && (pos(a,2) < 2) && (a<LOOP)
    a=a+1;
    vLeft = 20;
    vRight = 5;
    [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);    
    [~,pos(a,:)]=vrep.simxGetObjectPosition(clientID, r1_handle, -1, vrep.simx_opmode_blocking);
    [~,eAngle(a,:)]=vrep.simxGetObjectOrientation(clientID,r1_handle, -1, vrep.simx_opmode_blocking);
end
    vLeft = 0;
    vRight = 0;
    [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);

    plot(pos(:,1),pos(:,2))
    axis([-2.5 2.5 -2.5 2.5])
    grid on


    figure
    plot(eAngle)
    % Velocity MAX = 2*pi
vrep.delete(); % call the destructor!
%disp('Program ended');
