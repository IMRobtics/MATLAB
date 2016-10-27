function [] = setVelocity(i,vLeft,vRight,vrep,clientID)
    [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
end