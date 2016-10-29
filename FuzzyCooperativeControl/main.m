clc; clear all; close all; color = 'kbgrcmy'; colorVal=0;

turns = 1;
x = -1*pi*turns : 0.25  : pi*turns;
r = 0 : 1/(length(x)-1) : 1;

X = sin(x).*r*2;
Y = cos(x).*r*2;
x_g = transpose(X);
y_g = transpose(Y);
x_g=x_g(5:length(x_g)-5);
y_g=y_g(5:length(y_g)-5);

noOfRobots       = 3;
trajectoryOffset = 1;
trajectory       = zeros(17,2);
for i=1:noOfRobots
    trajectory   = [trajectory x_g+((i-1)*trajectoryOffset) y_g+((i-1)*trajectoryOffset)];
end
trajectory=trajectory(:,3:(noOfRobots*2+2));

figure;
hold on; grid on;
axis([-1 10 -1 10]);
axis square; axis equal;
for i=1:2:noOfRobots*2
   colorVal = colorVal+1;
   if(colorVal>7)
      colorVal=1;
   end
   plot(trajectory(:,i),trajectory(:,i+1),color(colorVal),'LineWidth',2);
end

point = 5;
Kp    = 5;
Ki    = 0.01;
Kd    = 0.01;
E_k   = 0;
e_k_1 = 0;
steps = 0;

distThresh  = 0.05;
angleThresh = 0.2;


% Robot and Simulation Constants
R = 41/2;  % in mm
L = 88.41; % in mm
dt=0.05;   % 50ms
maxVel=2*pi;
v = maxVel;

xyz    =zeros(noOfRobots,3);
eAngle =zeros(noOfRobots,3);
pose   =zeros(noOfRobots,3);

LOOP   = 1;
a = 1;
i = 1;   % for 1 robot
vLeft  = 0;
vRight = 0;

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
%   [res ir(a,:) Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getk3FrontIR_function',[1],[],'',[],vrep.simx_opmode_blocking);
%   [res Ints us(a,:) Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getk3FrontSonar_function',[1],[],'',[],vrep.simx_opmode_blocking);
    [~,xyz(a,:)]=vrep.simxGetObjectPosition(clientID, r1_handle, -1, vrep.simx_opmode_buffer);
    [~,eAngle(a,:)]=vrep.simxGetObjectOrientation(clientID,r1_handle, -1, vrep.simx_opmode_buffer);
    pose(a,:) = [xyz(a,1) xyz(a,2) eAngle(a,3)];
    x = pose(a,1);
    y = pose(a,2);
    theta=pose(a,3) + (pi/2);
    theta = atan2(sin(theta), cos(theta));

    dist = abs(sqrt( (y_g(point)-y)^2 + (x_g(point)-x)^2));

    % 1. Calculate the heading (angle) to the goal.
    % distance between goal and robot in x-direction
    u_x = x_g(point)-x;     
    % distance between goal and robot in y-direction
    u_y = y_g(point)-y;
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
    % the accumulated error, E_k, and the error for
    % this time step, e_k.
    e_I = E_k + e_k*dt;

    % error for the derivative term. Hint: Approximate the derivative
    % using the previous error, e_k_1, and the
    % error for this time step, e_k.
    e_D = (e_k-e_k_1)/dt;
    w = Kp*e_P + Ki*e_I + Kd*e_D;

    % 4. Save errors for next time step
    E_k = e_I;
    e_k_1 = e_k;
        
    if (abs(e_k)>angleThresh) && (dist>distThresh)
        turnFlag = true;
        StraightFlag = false;
        vRight = (2*v+w*L)/(2*R);
        vLeft  = (2*v-w*L)/(2*R);    
    elseif abs(e_k)<angleThresh
        turnFlag = false;
        StraightFlag = true;
    end
    
    if StraightFlag
        if dist>distThresh
            vRight = v;
            vLeft = v;
        else
            point = point+1;
            formatSpec = 'POINT=%1d REACHED.\n';
            fprintf(formatSpec,point-1)
            if point>length(x_g)
                vRight = 0;
                vLeft = 0;
                flag = false;
            end
        end
    end
    
    formatSpec = 'X=%4.2f, Y=%4.2f, Th=%4.2f, Thg=%4.2f, ThERR=%4.2f, DIST=%4.2f, vL=%4.2f, vR=%4.2f\n';
    fprintf(formatSpec,x,y,theta,theta_g,e_k,dist,vLeft,vRight)
    
    [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
    addpoints(position,x,y);
    drawnow
    LOOP = LOOP+1;
    steps = steps+1;
end

vLeft = 0;
vRight = 0;
[res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft vRight],[],'',[],vrep.simx_opmode_blocking);
vrep.delete();