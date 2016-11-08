clc; clear all; close all; color = 'kbgrcmy'; colorVal=0;

numberOfRobots = 1;

trajectory = generateTrajectory(numberOfRobots);

figure;
hold on; grid on;
axis([-2 4 -1 4]);
axis square; axis equal;

for i=1:2:numberOfRobots*2
   colorVal = colorVal+1;
   if(colorVal>7)
      colorVal=1;
   end
   plot(trajectory(:,i),trajectory(:,i+1),color(colorVal),'LineWidth',2);
end

Kp    = 5;
Ki    = 0.01;
Kd    = 0.01;
E_k   = zeros(1,numberOfRobots);
e_k_1 = zeros(1,numberOfRobots);
steps = 0;

distThresh  = 0.05;
angleThresh = 0.2;

% Robot and Simulation Constants
R = 41/2;  % in mm
L = 88.41; % in mm
dt=0.05;   % 50ms
maxVel=2*pi;
v = maxVel;

xyz    = zeros(numberOfRobots,3);
eAngle = zeros(numberOfRobots,3);
pose   = zeros(numberOfRobots,3);

vLeft  = zeros(1,numberOfRobots);
vRight = zeros(1,numberOfRobots);


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
[~,r1_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#', vrep.simx_opmode_blocking);
[~,r2_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#0', vrep.simx_opmode_blocking);
[~,r3_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#1', vrep.simx_opmode_blocking);
r_handle = [r1_handle r2_handle r3_handle];

for i=1:numberOfRobots
    [~,xyz(i,:)]=vrep.simxGetObjectPosition(clientID, r_handle(i), -1, vrep.simx_opmode_streaming);
    [~,eAngle(i,:)]=vrep.simxGetObjectOrientation(clientID,r_handle(i), -1, vrep.simx_opmode_streaming);
    pose(i,:) = [xyz(i,1) xyz(i,2) atan2(sin(eAngle(i,3)), cos(eAngle(i,3)))];
end

robotTrajectory = [animatedline('Color',color(colorVal),'LineWidth',2)];
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal=1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end

flag = ones(1,numberOfRobots);
turnFlag = ones(1,numberOfRobots);
StraightFlag = zeros(1,numberOfRobots);
moving = ones(1,numberOfRobots);
LOOP=0;
goalVals = [1 3 5; 2 4 6]; % trajectory columns, x1, y1, x2, y2, x3, y3
while LOOP < length(trajectory)
    LOOP = LOOP+1;
    while sum(moving)>0
        for i=1:numberOfRobots
            x_g = trajectory(LOOP,goalVals(1,i));
            y_g = trajectory(LOOP,goalVals(2,i));
            [~,xyz(i,:)]=vrep.simxGetObjectPosition(clientID, r_handle(i), -1, vrep.simx_opmode_buffer);
            [~,eAngle(i,:)]=vrep.simxGetObjectOrientation(clientID,r_handle(i), -1, vrep.simx_opmode_buffer);
            pose(i,:) = [xyz(i,1) xyz(i,2) atan2(sin(eAngle(i,3)), cos(eAngle(i,3)))];

            x = pose(i,1);
            y = pose(i,2);
            theta=pose(i,3) + (pi/2);
            theta = atan2(sin(theta), cos(theta));

            dist = abs(sqrt( (y_g-y)^2 + (x_g-x)^2));

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
            % the accumulated error, E_k, and the error for
            % this time step, e_k.
            e_I = E_k(1,i) + e_k*dt;

            % error for the derivative term. Hint: Approximate the derivative
            % using the previous error, e_k_1, and the
            % error for this time step, e_k.
            e_D = (e_k-e_k_1(1,i))/dt;
            w = Kp*e_P + Ki*e_I + Kd*e_D;

            % 4. Save errors for next time step
            E_k(1,i)   = e_I;
            e_k_1(1,i) = e_k;

            if (abs(e_k)>angleThresh) && (dist>distThresh)
                turnFlag(1,i) = 1;
                StraightFlag(1,i) = 0;
                vRight(1,i) = (2*v+w*L)/(2*R);
                vLeft(1,i)  = (2*v-w*L)/(2*R);    
            elseif abs(e_k)<angleThresh
                turnFlag(1,i) = 0;
                StraightFlag(1,i) = 1;
            end

            if StraightFlag(1,i)
                if dist>distThresh
                    vRight(1,i) = v;
                    vLeft(1,i) = v;
                else
                    if LOOP>=length(trajectory)
                        vRight(1,i) = 0;
                        vLeft(1,i) = 0;
                        moving(1,i) = 0;
                    end
                end
            end
            addpoints(robotTrajectory(i),x,y);
            [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft(1,i) vRight(1,i)],[],'',[],vrep.simx_opmode_blocking);
        end
        drawnow
    end

    for i=1:numberOfRobots
        vLeft(1,i) = 0;
        vRight(1,i) = 0;
        [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft(1,i) vRight(1,i)],[],'',[],vrep.simx_opmode_blocking);
    end
end

vrep.delete();