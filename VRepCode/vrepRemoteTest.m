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

[~,k3_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot', vrep.simx_opmode_blocking);
pos = [0 0 0];
[~,pos]=vrep.simxGetObjectPosition(clientID, k3_handle, -1, vrep.simx_opmode_streaming);
pause(1);
for t=1:20  %total simulation time
    [~,pos]=vrep.simxGetObjectPosition(clientID, k3_handle, -1, vrep.simx_opmode_buffer);
    pause(0.1);
end
[res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[k3_handle 2*pi pi],[],'',[],vrep.simx_opmode_blocking);
pause(1);

for t=1:10  %total simulation time
    [~,pos]=vrep.simxGetObjectPosition(clientID, k3_handle, -1, vrep.simx_opmode_buffer);
    pause(0.1);
end
[res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[k3_handle 2*pi 2*pi],[],'',[],vrep.simx_opmode_blocking);
pause(1);

for t=1:20  %total simulation time
    [~,pos]=vrep.simxGetObjectPosition(clientID, k3_handle, -1, vrep.simx_opmode_buffer);
    pause(0.1);
end
[res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[k3_handle pi 2*pi],[],'',[],vrep.simx_opmode_blocking);
pause(1);

vrep.delete(); % call the destructor!
disp('Program ended');