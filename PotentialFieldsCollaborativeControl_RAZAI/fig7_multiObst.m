clc; clear all; clear global; clear variables; close all;
color = 'kbgrcmy'; colorVal = 1;
figure; hold on; grid on;
axis square;
axis([10 100 10 100]);

TIMESTEP = 0.05;
TOTALTIME = 250;
DRAWCOUNT = 50+1;

FORCE_MAX = 20;
FORCE_MIN_THRESHOLD = 0;

VirtualBot = [5,5];
VirtualBotDot = [0,0];

numberOfRobots = 4;
robot = [([3, 5])
         ([2, 4])
         ([4, 4])
         ([5, 1])
        ];
robotT1 = robot;
robotDot = zeros(numberOfRobots,2);

% Obstacle centers and v1,v2 (height,width)/2
numberOfObstacles = 3;
obstacle = [([25, 25])
            ([35, 40])
            ([60, 70])
           ];
obstacleSize = [([4, 3])
                ([4, 4])
                ([5, 3])
               ];
v1 = zeros(1:numberOfObstacles);
v2 = zeros(1:numberOfObstacles);
A = zeros(1:numberOfObstacles);
B = zeros(1:numberOfObstacles);
          
           
M = 1;
Bz = 1;
kd = 9;
ksk = 100;
kr = 10;
alpha = 3;
qk = 10;
q(1:numberOfRobots) = qk;
m(1:numberOfRobots) = M;

ra = 1;
kp = 10;
Tau = 4;
           
% for Graphing and Plotting
for i = 1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2,2);
end
robotTrajectory = animatedline('Color',color(colorVal),'LineWidth',1);
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal = 1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',1)];
end
circle(VirtualBot(1,1),VirtualBot(1,2),alpha,2);
circle(VirtualBot(1,1),VirtualBot(1,2),0.2,2);
VirtualTrajectory = animatedline('Color','r','LineWidth',1,'LineStyle',':');

for i = 1:numberOfObstacles
    v1(i) = obstacleSize(i,1);
    v2(i) = obstacleSize(i,2);
    A(i) = sqrt(1/(2*((v1(i))^2)));
    B(i) = sqrt(1/(2*((v2(i))^2)));    
    
    rectangle('Position',[obstacle(i,1)-v1(i) obstacle(i,2)-v2(i) v1(i)*2 v2(i)*2]);
    ellipse(obstacle(i,1),obstacle(i,2),1/A(i),1/B(i));
end

drawnow

goalX = input('Entwer X-ordinate of Goal: ');
goalY = input('Entwer Y-ordinate of Goal: ');

goal = [goalX,goalY];
goalHeading = atan2(goalY-VirtualBot(1,2),goalX-VirtualBot(1,1));

fxvdes = 8*cos(goalHeading);
fyvdes = 8*sin(goalHeading);



obstacleFlag = zeros(1,numberOfRobots);
FxkVST1 = zeros(1,numberOfRobots);
FykVST1 = zeros(1,numberOfRobots);

FxkVS = zeros(1,numberOfRobots);
FykVS = zeros(1,numberOfRobots);

FxkVSrkra = zeros(1,numberOfRobots);
FykVSrkra = zeros(1,numberOfRobots);

FxkObs = zeros(1,numberOfRobots);
FykObs = zeros(1,numberOfRobots);

%Main Loop
zed = 0;
while zed<(TOTALTIME/TIMESTEP)
    zed = zed+1;
    
    a        = zeros(1,2); % for difference: x2-x1, y2-y1
    r        = zeros(numberOfRobots,numberOfRobots); % distance
    thetaX   = zeros(numberOfRobots,numberOfRobots);
    thetaY   = zeros(numberOfRobots,numberOfRobots);
    Fk       = zeros(numberOfRobots,numberOfRobots); % repulsive force among robots
    Fxk_dist = zeros(numberOfRobots,numberOfRobots);
    Fyk_dist = zeros(numberOfRobots,numberOfRobots);
    Fxk      = zeros(1,numberOfRobots); % sum of repulsive force among robots, X
    Fyk      = zeros(1,numberOfRobots); % sum of repulsive force among robots, X
    rk       = zeros(numberOfObstacles,numberOfRobots);
    nearObst = zeros(numberOfObstacles,numberOfRobots);
    FxkBS    = zeros(1,numberOfRobots);
    FykBS    = zeros(1,numberOfRobots);
    
    for i = 1:numberOfRobots
        for j = 1:numberOfObstacles
            a = obstacle(j,:)-robot(i,:);
            rk(j,i) = sqrt(((A(j)^2*a(1)^2+B(j)^2*a(2)^2-1)));
            if(rk(j,i)<= ra)
                nearObst(j,i) = 1;
            end
        end
        
        if(sum(nearObst(:,i))>0)
            % To check if this (ith) robot has reached near obstacle before
            % If the robot is reaching for the first time, near obstacle
            % The FxkVSrkra is set to the previosu force.
            if(obstacleFlag(i)==0)
                FxkVSrkra(i) = FxkVST1(i);
                FykVSrkra(i) = FykVST1(i);
                obstacleFlag(i)=1;
            end
            FxkBS(i) = FxkVSrkra(i);
            FykBS(i) = FykVSrkra(i);
            
            for j = 1:numberOfObstacles
                if(nearObst(j,i) ==1)
                    a = obstacle(j,:)-robot(i,:);
                    chi = atan2(a(2),a(1));
                    psi = atan2(robot(i,2)-robotT1(i,2),robot(i,1)-robotT1(i,1));            
                    if(mod(psi-chi,2*pi)<=pi)
                        Fxkrc  =  (B(j)/A(j))*(robot(i,2)-obstacle(j,2));
                        Fykrc  = -(A(j)/B(j))*(robot(i,1)-obstacle(j,1));
                        Fxkr   = Fxkrc;
                        Fykr   = Fykrc;
                    else
                        Fxkrcc = -(B(j)/A(j))*(robot(i,2)-obstacle(j,2));
                        Fykrcc =  (A(j)/B(j))*(robot(i,1)-obstacle(j,1));
                        Fxkr   = Fxkrcc;
                        Fykr   = Fykrcc;
                    end

                    Fxkrn = Fxkr/norm([Fxkr;Fykr]);
                    Fykrn = Fykr/norm([Fxkr;Fykr]);

                    % second part of eq 22, 
                    FxkObs(i) = ((g(FxkVSrkra(i),FykVSrkra(i))*Fxkrn)/rk(j,i)^2)*((1/rk(j,i))-(1/ra));
                    FykObs(i) = ((g(FxkVSrkra(i),FykVSrkra(i))*Fykrn)/rk(j,i)^2)*((1/rk(j,i))-(1/ra));

                    FxkBS(i) = FxkBS(i) + FxkObs(i);
                    FykBS(i) = FykBS(i) + FykObs(i);
                end
            end
        else
            for j = 1:numberOfRobots
                if i == j
                    r(i,j) = 0;
                    thetaX(i,j)	 = 0;
                    thetaY(i,j)	 = 0;
                    Fk(i,j) = 0;
                else
                    a = robot(i,:)-robot(j,:);
                    r(i,j) = sqrt(a(1)^2+a(2)^2);
                    
                    thetaX(i,j) = a(1)/abs(r(i,j));
                    thetaY(i,j) = a(2)/abs(r(i,j));
                    
                    % eq 6
                    Fk(i,j) = (kr*q(i)*q(j))/(r(i,j)^2);
                end
                Fxk_dist(i,j) = Fk(i,j)*thetaX(i,j);
                Fyk_dist(i,j) = Fk(i,j)*thetaY(i,j);
            end
            Fxk(i) = sum(Fxk_dist(i,1:numberOfRobots));
            Fyk(i) = sum(Fyk_dist(i,1:numberOfRobots));
            
            FxkVST1(i) = FxkVS(i);
            FykVST1(i) = FykVS(i);
            
            a = robot(i,:) - VirtualBot;
            
            % eq 8: FxkVS = Fxk - ksk*[attractive force b/w robot and virtual]
            FxkVS(i) = Fxk(i)-ksk*(a(1)*(a(1)^2+a(2)^2-alpha^2));
            FykVS(i) = Fyk(i)-ksk*(a(2)*(a(1)^2+a(2)^2-alpha^2));
            
            % eq 23
            FxkBS(i) = FxkVSrkra(i)*exp(-Tau*min(rk(:,i))) + FxkVS(i)*(1-exp(-Tau*min(rk(:,i))));
            FykBS(i) = FykVSrkra(i)*exp(-Tau*min(rk(:,i))) + FykVS(i)*(1-exp(-Tau*min(rk(:,i))));
            if(obstacleFlag(i)==1)
                obstacleFlag(i)=0;
            end
        end
    end
    
    % Feedback Force for Virtual Robot
    % dont know if to calculate this BEFORE or AFTER the movement of Robots
    % Currently calculating AFTER
    xm = sum(robot(:,1))/numberOfRobots; 
    ym = sum(robot(:,2))/numberOfRobots; 
    rm = min(min(rk));
    if(rm<=ra)
        fxvBS = fxvdes + kp*(xm-VirtualBot(1,1))*(1-exp(-Tau*rm));
        fyvBS = fyvdes + kp*(ym-VirtualBot(1,2))*(1-exp(-Tau*rm));
    else
        fxvBS = fxvdes;
        fyvBS = fyvdes;
    end
    
    % Applying Forces using the Dynamic Model
    for i=1:numberOfRobots
        [FxkBS(i),FykBS(i)] = forceConstrain(FxkBS(i),FykBS(i),FORCE_MAX);
        if(abs(FxkBS(i))>FORCE_MIN_THRESHOLD)
            fx = @(t,x) [x(2); (FxkBS(i)-(Bz+kd)*x(2))/M];
            [T,X] = ode45(fx,0:TIMESTEP/10:TIMESTEP,[robot(i,1);robotDot(i,1)]);
            robotT1(i,1) = robot(i,1);
            robot(i,1) = real(X(end,1));
            robotDot(i,1) = real(X(end,2));
        end
        
        if(abs(FykBS(i))>FORCE_MIN_THRESHOLD)
            fy = @(t,y) [y(2); (FykBS(i)-(Bz+kd)*y(2))/M];
            [T,Y] = ode45(fy,0:TIMESTEP/10:TIMESTEP,[robot(i,2);robotDot(i,2)]);
            robotT1(i,2) = robot(i,2);
            robot(i,2) = real(Y(end,1));
            robotDot(i,2) = real(Y(end,2));
        end
        
        addpoints(robotTrajectory(i),real(X(1:end-1,1)),real(Y(1:end-1,1)));
        addpoints(robotTrajectory(i),robot(i,1),robot(i,2));
    end
    
    [fxvBS,fyvBS] = forceConstrain(fxvBS,fyvBS,FORCE_MAX);
    if(abs(fxvBS)>FORCE_MIN_THRESHOLD)
        fx = @(t,x) [x(2); (fxvBS-(Bz+kd)*x(2))/M];
        [T,X] = ode45(fx,0:TIMESTEP/10:TIMESTEP,[VirtualBot(1,1);VirtualBotDot(1,1)]);
        VirtualBot(1,1) = real(X(end,1));
        VirtualBotDot(1,1) = real(X(end,2));
    end
    if(abs(fyvBS)>FORCE_MIN_THRESHOLD)
        fy = @(t,y) [y(2); (fyvBS-(Bz+kd)*y(2))/M];
        [T,Y] = ode45(fy,0:TIMESTEP/10:TIMESTEP,[VirtualBot(1,2);VirtualBotDot(1,2)]);
        VirtualBot(1,2) = real(Y(end,1));
        VirtualBotDot(1,2) = real(Y(end,2));
    end
%     addpoints(VirtualTrajectory,real(X(1:end-1,1)), real(Y(1:end-1,1)));
    addpoints(VirtualTrajectory,VirtualBot(1,1), VirtualBot(1,2));
    
    if(mod(zed,round((TOTALTIME/TIMESTEP)/DRAWCOUNT)) == 0)
        % circle(VirtualBot(1,1),VirtualBot(1,2),alpha,0.5);
        circle(VirtualBot(1,1),VirtualBot(1,2),0.2,1);
        for i = 1:numberOfRobots
            circle(robot(i,1),robot(i,2),0.2,2);
        end
%         border(robot(1:numberOfRobots,1),robot(1:numberOfRobots,2));
        zed*TIMESTEP;
        drawnow
        a = VirtualBot-goal;
        if(sqrt(a(1)^2+a(2)^2)<5)
            return
        end
    end
end