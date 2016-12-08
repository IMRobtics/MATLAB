clc; clear all; close all; color = 'kbgrcmy'; colorVal=1;

numberOfRobots=2;
robot=[ ([-5,   -5, pi/2])
        ([-5, -6.5, pi/2])
        ];
robotT1 = robot;
%positions being input from VREP - these are just DUMMY values, but right
%ones

figure; hold on; grid on;
FLOORSIZE = 7.5; % in meters HALF
axis([-FLOORSIZE FLOORSIZE -FLOORSIZE FLOORSIZE]); axis square;

%Obstacle Info
x0 = -1.5;
y0 = -1.5;
v1=1.5; v2=1;
rectWidth =v1*2;
rectHeight=v2*2;

v1=1.5+(0.6/2); v2=1+(0.6/2);
%Draw a rectangle
rectangle('Position',[x0-v1 y0-v2 rectWidth rectHeight]);%,'FaceColor',[1 0 0]);

A=sqrt(1/(2*((v1)^2)));
B=sqrt(1/(2*((v2)^2)));

ellipse(x0,y0,1/A,1/B);

ra = 1;
rk = zeros(numberOfRobots,1);

fxkdes = ones(numberOfRobots,1);
fykdes = ones(numberOfRobots,1);
% for figure 4
fxkdes = fxkdes.*8;
fykdes = fykdes.*8;

fxkOA = zeros(numberOfRobots,1);
fykOA = zeros(numberOfRobots,1);

xdot = zeros(numberOfRobots,1);
ydot = zeros(numberOfRobots,1);

psi = zeros(numberOfRobots,1);
chi = zeros(numberOfRobots,1);

M=1;
Bz=1;
kd=9;

maxVel=1000;
stopVel=500;
% scalling is done on V-Rep side, here MUST not go above 1000
% for forward and backward movement - 500 is STOP - 0 is BACK full speed
% and 1000 is forward full speed.
% similar for Left (0) and Right(1000)
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
else
    disp('Failed connecting to remote API server');
end
%=================================================
[~,r1_handle]=vrep.simxGetObjectHandle(clientID,'youBot#', vrep.simx_opmode_blocking);
[~,r2_handle]=vrep.simxGetObjectHandle(clientID,'youBot#0', vrep.simx_opmode_blocking);
% [~,r3_handle]=vrep.simxGetObjectHandle(clientID,'youBot#1', vrep.simx_opmode_blocking);
% [~,r4_handle]=vrep.simxGetObjectHandle(clientID,'youBot#2', vrep.simx_opmode_blocking);
r_handle = [r1_handle r2_handle]; % r3_handle r4_handle];

xyz = zeros(2,3);
eAngle = zeros(2,3);
pose = zeros(2,3);
for i=1:numberOfRobots
    [~,xyz(i,:)]=vrep.simxGetObjectPosition(clientID, r_handle(i), -1, vrep.simx_opmode_blocking);
    [~,eAngle(i,:)]=vrep.simxGetObjectOrientation(clientID,r_handle(i), -1, vrep.simx_opmode_blocking);
    pose(i,:) = [xyz(i,1) xyz(i,2) atan2(sin(eAngle(i,3)), cos(eAngle(i,3)))];
    
    x = pose(i,1);
    y = pose(i,2);
    theta=pose(i,3) + (pi/2);

    robot(i,:) = [x y theta];
end

% Create Animated Line Objects for Each robot, different colors
robotTrajectory = animatedline('Color',color(colorVal),'LineWidth',2);
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal=1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end

for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2,2);
end


t = 0;
MAX_TIME = 200;
zed = zeros(MAX_TIME,numberOfRobots*2);
% xdot1, ydot1, xdot2, ydot2
while t<MAX_TIME
    t = t+1;
    
    for i=1:numberOfRobots
        [~,xyz(i,:)]=vrep.simxGetObjectPosition(clientID, r_handle(i), -1, vrep.simx_opmode_blocking);
        [~,eAngle(i,:)]=vrep.simxGetObjectOrientation(clientID,r_handle(i), -1, vrep.simx_opmode_blocking);
        pose(i,:) = [xyz(i,1) xyz(i,2) atan2(sin(eAngle(i,3)), cos(eAngle(i,3)))];

        x = pose(i,1);
        y = pose(i,2);
        theta=pose(i,3) + (pi/2);

        robot(i,:) = [x y theta];
        robotT1(i,:) = robot(i,:);
        addpoints(robotTrajectory(i),x,y);
        drawnow
    end    
    
    for i=1:numberOfRobots
        xk = robot(i,1);
        yk = robot(i,2);
        
        chi(i)=atan2(y0-yk,x0-xk);
        psi(i) = atan2(robot(i,2)-robotT1(i,2),robot(i,1)-robotT1(i,1));

        if(mod(psi(i)-chi(i),2*pi)<=pi)
            fxkrc  =  (B/A)*(yk-y0);
            fykrc  = -(A/B)*(xk-x0);            
            fxkr   = fxkrc;
            fykr   = fykrc;
        else
            fxkrcc = -(B/A)*(yk-y0);
            fykrcc =  (A/B)*(xk-x0);
            fxkr   = fxkrcc;
            fykr   = fykrcc;
        end
        
        if(i==4)
            zed
        end
        
        fxkrn = fxkr/norm([fxkr;fykr]);
        fykrn = fykr/norm([fxkr;fykr]);

        rk(i) =sqrt(((A^2*(xk-x0)^2+B^2*(yk-y0)^2-1)));
        if(rk(i)<=ra)
            fxkOA(i) = fxkdes(i) + ((g(fxkdes(i),fykdes(i))*fxkrn)/rk(i)^2)*((1/rk(i))-(1/ra));
            fykOA(i) = fykdes(i) + ((g(fxkdes(i),fykdes(i))*fykrn)/rk(i)^2)*((1/rk(i))-(1/ra));
        else
            fxkOA(i) = fxkdes(i);
            fykOA(i) = fykdes(i);
        end
        
%         if(abs(fxkOA(i))>FORCE_MAX)
%             fxkOA(i)=(fxkOA(i)/abs(fxkOA(i)))*FORCE_MAX;
%         end
%         if(abs(fykOA(i))>FORCE_MAX)
%             fykOA(i)=(fykOA(i)/abs(fykOA(i)))*FORCE_MAX;
%         end

        fx = @(t,x) [x(2); (fxkOA(i)-(Bz+kd)*x(2))/M];
        [T,X]=ode45(fx,[0,0.05],[xk;xdot(i)]);
        [m,z] = size(X);
        xk=real(X(m,1));
        xdot(i)=real(X(m,2));

        fy = @(t,y) [y(2); (fykOA(i)-(Bz+kd)*y(2))/M];
        [T,Y]=ode45(fy,[0,0.05],[yk;ydot(i)]);
        [m,z] = size(Y);
        yk=real(Y(m,1));
        ydot(i)=real(Y(m,2));
        
        % xdot1, ydot1, xdot2, ydot2
        if(i==1)
            zed(t,1) = real(xdot(i));
            zed(t,2) = real(ydot(i));
        end
        if(i==2)
            zed(t,3) = real(xdot(i));
            zed(t,4) = real(ydot(i));
        end
    end
    
    for i=1:numberOfRobots
        mult = 200;
        if(i==1)
            vX = stopVel+zed(t,1)*mult;
            vY = stopVel+zed(t,2)*mult;
        end
        if(i==2)
            vX = stopVel+zed(t,3)*mult;
            vY = stopVel+zed(t,4)*mult;
        end        
        [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setVelocity_function',[i vX vY],[],'',[],vrep.simx_opmode_blocking);
    end
    t
    zed(t,:)
end

for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2,4);
end

for i=1:numberOfRobots
    vX = stopVel;
    vY = stopVel;
    [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setVelocity_function',[i vX vY],[],'',[],vrep.simx_opmode_blocking);
end