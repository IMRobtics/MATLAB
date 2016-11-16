clc; clear all; close all; color = 'kbgrcmy'; colorVal=1;
figure;
hold on; grid on;
axis([-7 7 -7 7]);
axis square; axis equal;

% Robot and Simulation Constants
R = 41/2;  % in mm
L = 88.41; % in mm
dt=0.05;   % 50ms
maxVel=2*pi;
v = maxVel;

numberOfRobots = 4;

xyz    = zeros(numberOfRobots,3);
eAngle = zeros(numberOfRobots,3);
pose   = zeros(numberOfRobots,3);

vLeft  = zeros(1,numberOfRobots);
vRight = zeros(1,numberOfRobots);

robot  = zeros(numberOfRobots,3);   % x,y,theta

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
[~,r4_handle]=vrep.simxGetObjectHandle(clientID,'K3_robot#2', vrep.simx_opmode_blocking);
r_handle = [r1_handle r2_handle r3_handle r4_handle];

for i=1:numberOfRobots
    [~,xyz(i,:)]=vrep.simxGetObjectPosition(clientID, r_handle(i), -1, vrep.simx_opmode_blocking);
    [~,eAngle(i,:)]=vrep.simxGetObjectOrientation(clientID,r_handle(i), -1, vrep.simx_opmode_blocking);
    pose(i,:) = [xyz(i,1) xyz(i,2) atan2(sin(eAngle(i,3)), cos(eAngle(i,3)))];
    
    x = pose(i,1);
    y = pose(i,2);
    theta=pose(i,3) + (pi/2);

    robot(i,:) = [x y theta]
end

robotTrajectory = [animatedline('Color',color(colorVal),'LineWidth',2)];
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal=1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end     
    
for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2);
end

M=1;
B=1;
kd=9;
ksk=1;
kr=0.15;
alpha=3;
qk=10;

%Charges on each robot
q(1:numberOfRobots)=qk;
%Mass of each robot
m(1:numberOfRobots)=M;
%Electrostatic Constant
%K=10;

% Force Threshold, robots stop moving if force < threshold
FORCE_THRESHOLD = 0.75;
FORCE_MAX = 100;

xdot = zeros(numberOfRobots,1);
ydot = zeros(numberOfRobots,1);

% Load First Trajectory point and Draw Circle Around it
VirtualBot=[0,0];
circle(VirtualBot(1,1),VirtualBot(1,2),alpha);
VirtualTrajectory = animatedline('Color','r','LineWidth',2,'LineStyle','-.')

Fxk_dist = zeros(numberOfRobots,numberOfRobots);
Fxk = zeros(1,numberOfRobots);
Attractive_Force_x = zeros(1,numberOfRobots);
FxkVS = zeros(1,numberOfRobots);

Fyk_dist = zeros(numberOfRobots,numberOfRobots);
Fyk = zeros(1,numberOfRobots);
Attractive_Force_y = zeros(1,numberOfRobots);
FykVS = zeros(1,numberOfRobots);

VirtualBot=[0,0];
% Vector keep track of how many robots still moving
% If all go to ZERO, Virtual Bot moves to NEXT WayPoint
movement = ones(numberOfRobots,1);
iterations = 0; % Variable to track iterations in each inner loop

% To check Dynamic Model
% robot=[ ([-3,3]);([1,1]);([5,2]);([5,5])];

while sum(movement)>0
    iterations = iterations+1

    for i=1:numberOfRobots
        [~,xyz(i,:)]=vrep.simxGetObjectPosition(clientID, r_handle(i), -1, vrep.simx_opmode_blocking);
        [~,eAngle(i,:)]=vrep.simxGetObjectOrientation(clientID,r_handle(i), -1, vrep.simx_opmode_blocking);
        pose(i,:) = [xyz(i,1) xyz(i,2) atan2(sin(eAngle(i,3)), cos(eAngle(i,3)))];

        x = pose(i,1);
        y = pose(i,2);
        theta=pose(i,3) + (pi/2);

        robot(i,:) = [x y theta];
        addpoints(robotTrajectory(i),x_pos_new,y_pos_new);
    end


    %Computing distance of individual robot from all other robots
    r=zeros(numberOfRobots,numberOfRobots);
    for i=1:numberOfRobots
        for j=1:numberOfRobots
            a=robot(i,:)-robot(j,:);
            r(i,j)=sqrt(a(1)^2+a(2)^2);
        end
    end

    %Computing orientation of individual from all other robots
    thetaX=zeros(numberOfRobots,numberOfRobots);
    thetaY=zeros(numberOfRobots,numberOfRobots);
    for i=1:numberOfRobots
        for j=1:numberOfRobots
            if i==j
                thetaX(i,j)=0;thetaY(i,j)=0;
            elseif i~=j
                a=robot(i,:)-robot(j,:);
                thetaX(i,j)=a(1)/abs(r(i,j));
                thetaY(i,j)=a(2)/abs(r(i,j));
                %theta(i,j)=atan2(a(2),a(1));
            end
        end
    end

    %     Computing Electrostatic forces (Repulsive) on individual robot
    %     from all other robots
    Electrostatic_Forces=zeros(numberOfRobots,numberOfRobots);
    for i=1:numberOfRobots
        for j=1:numberOfRobots
            if i==j
                Electrostatic_Forces(i,j)=0;
            elseif i~=j
                Electrostatic_Forces(i,j)=(kr*q(i)*q(j))/(r(i,j)^2);
            end
        end
    end

    %Decomposition of Electrostatic forces in x and y components
    for i=1:numberOfRobots
        for j=1:numberOfRobots
            Fxk_dist(i,j)=Electrostatic_Forces(i,j)*thetaX(i,j);
            Fyk_dist(i,j)=Electrostatic_Forces(i,j)*thetaY(i,j);
        end
    end

    for i=1:numberOfRobots
        Fxk(i)=sum(Fxk_dist(i,1:numberOfRobots));
        Fyk(i)=sum(Fyk_dist(i,1:numberOfRobots));
    end

    %Computing x and y components of Attractive Force (Equation 8 in JP)
    for i=1:numberOfRobots
        a=robot(i,:)-VirtualBot;
        Attractive_Force_x(i)=ksk*(a(1)*(a(1)^2+a(2)^2-alpha^2));
        Attractive_Force_y(i)=ksk*(a(2)*(a(1)^2+a(2)^2-alpha^2));
    end

    %Computing resultant forces on each robot
    for i=1:numberOfRobots
        FxkVS(i)=Fxk(i)-Attractive_Force_x(i);
        FykVS(i)=Fyk(i)-Attractive_Force_y(i);
    end

    for i=1:numberOfRobots

        % ============= DYNAMIC MODEL ===========
        x_pos_new = robot(i,1);
        y_pos_new = robot(i,2);

        if(abs(FxkVS(i))>FORCE_THRESHOLD)
%             no mention of FORCE constrain in PAPER                
%             if(abs(FxkVS(i))>FORCE_MAX)
%                 FxkVS(i)=(FxkVS(i)/abs(FxkVS(i)))*FORCE_MAX;
%             end
            fx = @(t,x) [x(2); (FxkVS(i)-(B+kd)*x(2))/M];
            [T,X]=ode45(fx,[0,0.05],[robot(i,1);xdot(i)]);
            [m,z] = size(X);
            x_pos_new=X(m,1);
            xdot(i)=X(m,2);     
        end

        if(abs(FykVS(i))>FORCE_THRESHOLD)
%             no mention of FORCE constrain in PAPER                
%             if(abs(FykVS(i))>FORCE_MAX)
%                 FykVS(i)=(FykVS(i)/abs(FykVS(i)))*FORCE_MAX;
%             end
            fy = @(t,y) [y(2); (FykVS(i)-(B+kd)*y(2))/M];
            [T,Y]=ode45(fy,[0,0.05],[robot(i,2);ydot(i)]);
            [m,z] = size(Y);
            y_pos_new=Y(m,1);
            ydot(i)=Y(m,2);
        end
        
%         To chekc DYNAMIC MODEL
%         robot(i,:)=[x_pos_new y_pos_new];
%         addpoints(robotTrajectory(i),x_pos_new,y_pos_new);
        % END DYNAMIC MODEL ===================================



        if abs(FxkVS(i))<=FORCE_THRESHOLD &&  abs(FykVS(i))<=FORCE_THRESHOLD
            movement(i,1) = 0;
        elseif abs(FxkVS(i))>FORCE_THRESHOLD || abs(FykVS(i))>FORCE_THRESHOLD
            movement(i,1) = 1;
        end

%         get resultant Force - if needed
%         convert Resultant force to Vl and Vr of each robot
%         make sure vl and vr range is -2pi to 2pi
%         if(vl<-2pi)vl=-2pi else if(vl>2pi) vl=2pi

%         vLeft(1,i) = ???;
%         vRight(1,i)= ???;
%         set vl and vr of each robot
        [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setk3Velocity_function',[i vLeft(1,i) vRight(1,i)],[],'',[],vrep.simx_opmode_blocking);
    end
    drawnow
    % MIGHT need some delay here
end    

for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2);
end

border(robot(:,1),robot(:,2));