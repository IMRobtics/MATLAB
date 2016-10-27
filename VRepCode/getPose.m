function [x,y,theta] = getPose(vrep,clientID,r1_handle)
    xyz = zeros(1,3);
    eAngle = zeros(1,3);
    [~,xyz(1,:)]=vrep.simxGetObjectPosition(clientID, r1_handle, -1, vrep.simx_opmode_streaming);
    [~,eAngle(1,:)]=vrep.simxGetObjectOrientation(clientID,r1_handle, -1, vrep.simx_opmode_streaming);
    x = xyz(1,1); y = xyz(1,2);
    ey = eAngle(1,3);
    ey = ey + (pi/2);
    theta = atan2(sin(ey), cos(ey));
end