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

maxVel=2*pi
LOOP=1000;
xyz = zeros(LOOP,3);
eAngle = zeros(LOOP,3);
pose = zeros(LOOP,3);
ir = zeros(LOOP,9);
us = zeros(LOOP,5);

a=1;
i=1;
vLeft = 0;
vRight = 0;
[res ir(a,:) Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getk3FrontIR_function',[1],[],'',[],vrep.simx_opmode_blocking);
[res Ints us(a,:) Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getk3FrontSonar_function',[1],[],'',[],vrep.simx_opmode_blocking);
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
[~,r1_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#', vrep.simx_opmode_blocking);
[~,xyz(a,:)]=vrep.simxGetObjectPosition(clientID, r1_handle, -1, vrep.simx_opmode_streaming);
[~,eAngle(a,:)]=vrep.simxGetObjectOrientation(clientID,r1_handle, -1, vrep.simx_opmode_streaming);
pose(a,:) = [xyz(a,1) xyz(a,2) eAngle(a,3)];

position = animatedline;
axis([-2.5 2.5 -2.5 2.5])
grid on


while (a<LOOP)
    [res ir(a,:) Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getk3FrontIR_function',[1],[],'',[],vrep.simx_opmode_blocking);
    [res Ints us(a,:) Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getk3FrontSonar_function',[1],[],'',[],vrep.simx_opmode_blocking);
    [~,xyz(a,:)]=vrep.simxGetObjectPosition(clientID, r1_handle, -1, vrep.simx_opmode_buffer);
    [~,eAngle(a,:)]=vrep.simxGetObjectOrientation(clientID,r1_handle, -1, vrep.simx_opmode_buffer);
    pose(a,:) = [xyz(a,1) xyz(a,2) eAngle(a,3)];

    if (us(a,1)<0.5) || (us(a,2)<0.5) || (us(a,3)<0.5) || (us(a,4)<0.5) || (us(a,5)<0.5)
        test = rand(1);
        if test<0.5
            vRight = maxVel*0.25;
        else
            vRight = maxVel*-0.25;
        end
    else
        vRight = maxVel;
    end
    vLeft = maxVel;
    [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
    addpoints(position,pose(a,1),pose(a,2));
    drawnow
    a = a+1;
end

vLeft = 0;
vRight = 0;
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
vrep.delete();
%disp('Program ended');
