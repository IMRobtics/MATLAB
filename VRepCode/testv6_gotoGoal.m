hold on;
clc; clear all; %close all;
disp('Started');

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
else
    disp('Failed connecting to remote API server');
end

x_g = 0.75;
y_g = .5;
Kp = 4;
Ki = 0.01;
Kd = 0.01;
E_k = 0;
e_k_1 = 0;

%=================================================
R = 41/2; % in mm
L = 88.41; % in mm
dt=0.05; %50ms
maxVel=2*pi;
xyz = zeros(1,3);
eAngle = zeros(1,3);
pose = zeros(1,3);
LOOP=1;
a=1;
i=1; % for 1 robot
vLeft = 0;
vRight = 0;

v = maxVel; %constant velocity
%=================================================
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
[~,r1_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#', vrep.simx_opmode_blocking);
[~,xyz(a,:)]=vrep.simxGetObjectPosition(clientID, r1_handle, -1, vrep.simx_opmode_streaming);
[~,eAngle(a,:)]=vrep.simxGetObjectOrientation(clientID,r1_handle, -1, vrep.simx_opmode_streaming);
pose(a,:) = [xyz(a,1) xyz(a,2) atan2(sin(eAngle(a,3)), cos(eAngle(a,3)))];

position = animatedline;
axis([-2.5 2.5 -2.5 2.5])
grid on


% while abs(pose(1,1)-goal(1,1))>0.2 && abs(pose(1,2)-goal(1,2))>0.2
% while LOOP<10
flag = true;
turnFlag = true;
StraightFlag = false;
while flag
%     [res ir(a,:) Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getk3FrontIR_function',[1],[],'',[],vrep.simx_opmode_blocking);
%     [res Ints us(a,:) Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getk3FrontSonar_function',[1],[],'',[],vrep.simx_opmode_blocking);
    [~,xyz(a,:)]=vrep.simxGetObjectPosition(clientID, r1_handle, -1, vrep.simx_opmode_buffer);
    [~,eAngle(a,:)]=vrep.simxGetObjectOrientation(clientID,r1_handle, -1, vrep.simx_opmode_buffer);
    pose(a,:) = [xyz(a,1) xyz(a,2) eAngle(a,3)];
    x = pose(a,1);
    y = pose(a,2);
    theta=pose(a,3) + (pi/2);
    theta = atan2(sin(theta), cos(theta));

    % 1. Calculate the heading (angle) to the goal.
    % distance between goal and robot in x-direction
    u_x = x_g-x;     
    % distance between goal and robot in y-direction
    u_y = y_g-y;
    % angle from robot to goal. Hint: use ATAN2, u_x, u_y here.
    theta_g = atan2(u_y,u_x);

    % 2. Calculate the heading error.
    % error between the goal angle and robot's angle
    % Hint: Use ATAN2 to make sure this stays in [-pi,pi].
    e_k = theta_g-theta;
    e_k = atan2(sin(e_k),cos(e_k));

    % 3. Calculate PID for the steering angle 
    % error for the proportional term
    e_P = e_k;

    % error for the integral term. Hint: Approximate the integral using
    % the accumulated error, obj.E_k, and the error for
    % this time step, e_k.
    e_I = E_k + e_k*dt;

    % error for the derivative term. Hint: Approximate the derivative
    % using the previous error, obj.e_k_1, and the
    % error for this time step, e_k.
    e_D = (e_k-e_k_1)/dt;
    w = Kp*e_P + Ki*e_I + Kd*e_D;

    % 4. Save errors for next time step
    E_k = e_I;
    e_k_1 = e_k;
        
    if(abs(e_k)>0.2)
        turnFlag = true;
        StraightFlag = false;
        vRight = (2*v+w*L)/(2*R);
        vLeft  = (2*v-w*L)/(2*R);    
    elseif (abs(e_k)<0.1)
        turnFlag = false;
        StraightFlag = true;
    end
    
    dist = abs(sqrt( (y_g-y)^2 + (x_g-x)^2));
    if StraightFlag
        if dist>0.02
            vRight = v;
            vLeft = v;
        else
            vRight = 0;
            vLeft = 0;
            flag = false;
        end
    end
    
    formatSpec = 'X=%4.2f, Y=%4.2f, Th=%4.2f, Thg=%4.2f, ThERR=%4.2f, DIST=%4.2f, vL=%4.2f, vR=%4.2f\n';
    fprintf(formatSpec,x,y,theta,theta_g,e_k,dist,vLeft,vRight)
    
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
