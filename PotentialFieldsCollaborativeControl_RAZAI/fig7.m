clc; clear all; close all; color = 'kbgrcmy'; colorVal = 1;
figure; hold on; grid on;
axis([10 55 10 55]); axis square;

TIMESTEP = 0.1;
TOTALTIME = 50;

numberOfRobots = 4;
robot = [([14,19])
         ([16,16])
         ([18,17])
         ([18,22])
        ];
robotT1 = robot;
robotDot = zeros(numberOfRobots,2);
 
for i = 1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2,2);
end

% Create Animated Line Objects for Each robot, different colors
robotTrajectory = [animatedline('Color',color(colorVal),'LineWidth',1)];
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal = 1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',1)];
end

obstacle = [([30,30])];
obstacleSize = [([4,3])];
A = 0;
B = 0;
ra = 1;
rk = zeros(1,numberOfRobots);
v1 = obstacleSize(1,1);
v2 = obstacleSize(1,2);
rectangle('Position',[obstacle(1,1)-v1 obstacle(1,2)-v2 v1*2 v2*2]);
A = sqrt(1/(2*((v1)^2)));
B = sqrt(1/(2*((v2)^2)));
ellipse(obstacle(1,1),obstacle(1,2),1/A,1/B);

M = 1;
Bz = 1;
kd = 9;
ksk = 1;
kr = 0.15;
alpha = 3;
qk = 10;
q(1:numberOfRobots) = qk;
m(1:numberOfRobots) = M;

% Load First Trajectory point and Draw Circle Around it
VirtualBot = [17.5, 18];
VirtualBotDot = [0,0];
circle(VirtualBot(1,1),VirtualBot(1,2),alpha,2);
circle(VirtualBot(1,1),VirtualBot(1,2),0.2,2);
VirtualTrajectory = animatedline('Color','r','LineWidth',1,'LineStyle','-.');

FORCE_MAX = 10;
FORCE_THRESHOLD = 0.1;

% Force on Virtual Bot
fxvdes = 6;
fyvdes = 6;

Tau = 4;
kp = 1;

obstacleFlag = ones(1,numberOfRobots);

FxkVST1 = zeros(1,numberOfRobots);
FykVST1 = zeros(1,numberOfRobots);

FxkVSrkra = zeros(1,numberOfRobots);
FykVSrkra = zeros(1,numberOfRobots);

FxkOA = zeros(1,numberOfRobots);
FykOA = zeros(1,numberOfRobots);

FxkBS = zeros(1,numberOfRobots);
FykBS = zeros(1,numberOfRobots);

xdot = zeros(1,numberOfRobots);
ydot = zeros(1,numberOfRobots);

FxkVS = zeros(1,numberOfRobots);
FykVS = zeros(1,numberOfRobots);

zed = 0;
while zed<(TOTALTIME/TIMESTEP)
    zed = zed+1;
    
    a        = zeros(1,2); % for difference: x2-x1, y2-y1
    r        = zeros(numberOfRobots,numberOfRobots); % distance
    thetaX   = zeros(numberOfRobots,numberOfRobots);
    thetaY   = zeros(numberOfRobots,numberOfRobots);
    Fk       = zeros(numberOfRobots,numberOfRobots);	% repulsive force among robots
    Fxk_dist = zeros(numberOfRobots,numberOfRobots);
    Fyk_dist = zeros(numberOfRobots,numberOfRobots);
    Fxk      = zeros(1,numberOfRobots); % sum of repulsive force among robots, X
    Fyk      = zeros(1,numberOfRobots); % sum of repulsive force among robots, X
    
    % Calculating FxkVS and FykVS
    % Repulsive Forces among robots and Attractive Force b/w Virtual and Robot
    for i = 1:numberOfRobots
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
        
        a = robot(i,:) - VirtualBot;
        
        % retaining previous loop value: T-TIMESTEP
        FxkVST1(i) = FxkVS(i);
        FykVST1(i) = FykVS(i);
        
        % eq 8: FxkVS = Fxk - ksk*[attractive force b/w robot and virtual]
        FxkVS(i) = Fxk(i)-ksk*(a(1)*(a(1)^2+a(2)^2-alpha^2));
        FykVS(i) = Fyk(i)-ksk*(a(2)*(a(1)^2+a(2)^2-alpha^2));
    end
    
    
    % Calculating FxkOA and FxyOA
    % Repulsive Forces among robots and Attractive Force b/w Virtual and Robot    
    for i = 1:numberOfRobots
        FxkOA(i) = 0;
        FykOA(i) = 0;
        a = robot(i,:)-obstacle(1,:);
        rk(i) = sqrt(((A^2*a(1)^2+B^2*a(2)^2-1)));
        if(rk(i)<= ra)
            chi = atan2(a(2),a(1));
            psi = atan2(robot(i,2)-robotT1(i,2),robot(i,1)-robotT1(i,1));
            
            if(mod(psi-chi,2*pi)<= pi)
                Fxkrc =  (B/A)*a(2);
                Fykrc = -(A/B)*a(1);
                Fxkr = Fxkrc;
                Fykr = Fykrc;
            else
                Fxkrcc = -(B/A)*a(2);
                Fykrcc =  (A/B)*a(1);
                Fxkr = Fxkrcc;
                Fykr = Fykrcc;
            end
            
            Fxkrn = Fxkr/norm([Fxkr;Fykr]);
            Fykrn = Fykr/norm([Fxkr;Fykr]);
            
            FxkOA(i) = ((g(FxkVSrkra(i),FykVSrkra(i))*Fxkrn)/rk(i)^2)*((1/rk(i))-(1/ra));
            FykOA(i) = ((g(FxkVSrkra(i),FykVSrkra(i))*Fykrn)/rk(i)^2)*((1/rk(i))-(1/ra));
        end
    end
    
    for i=1:numberOfRobots
        a = robot(i,:)-obstacle(1,:);
        rk(i) = sqrt(((A^2*a(1)^2+B^2*a(2)^2-1)));
        if(rk(i)<= ra)
            FxkBS(i) = FxkVSrkra(i) + FxkOA(i);
            FykBS(i) = FykVSrkra(i) + FykOA(i);
        else
            FxkBS(i) = FxkVSrkra(i)*exp(-Tau*rk(i)) + FxkVS(i)*(1-exp(-Tau*rk(i)));
            FykBS(i) = FykVSrkra(i)*exp(-Tau*rk(i)) + FykVS(i)*(1-exp(-Tau*rk(i)));
        end
    end
    
    
    
    for i=1:numberOfRobots
        [FxkBS(i),FykBS(i)] = forceConstrain(FxkBS(i),FykBS(i),FORCE_MAX);
        if(abs(FxkBS(i))>FORCE_THRESHOLD)
            fx = @(t,x) [x(2); (FxkBS(i)-(Bz+kd)*x(2))/M];
            [T,X] = ode45(fx,[0 TIMESTEP],[xk;xdot(i)]);
            robotT1(i,1) = robot(i,1);
            robot(i,1) = real(X(:,1));
            robotDot(i,1) = real(X(end,2));
        end
        if(abs(FykBS(i))>FORCE_THRESHOLD)
            fy = @(t,y) [y(2); (FykBS(i)-(Bz+kd)*y(2))/M];
            [T,Y] = ode45(fy,[0 TIMESTEP],[yk;ydot(i)]);
            robotT1(i,2) = robot(i,2);
            robot(i,2) = real(Y(:,2));
            robotDot(i,2) = real(Y(end,2));
        end
        addpoints(robotTrajectory(i),robot(i,1),robot(i,2));
    end        
    
    % Feedback Force for Virtual Robot
    % dont know if to calculate this BEFORE or AFTER the movement of Robots
    % Currently calculating AFTER
    xm = sum(robot(:,1))/numberOfRobots; 
    ym = sum(robot(:,2))/numberOfRobots; 
    rm = min(rk);
    if(rm<ra)
        fxvBS = fxvdes + kp*(xm-VirtualBot(1,1))*(1-exp(-Tau*rm));
        fyvBS = fyvdes + kp*(ym-VirtualBot(1,2))*(1-exp(-Tau*rm));
    else
        fxvBS = fxvdes;
        fyvBS = fyvdes;
    end
    
    [fxvBS,fyvBS] = forceConstrain(fxvBS,fyvBS,FORCE_MAX);
    fx = @(t,x) [x(2); (fxvBS-(Bz+kd)*x(2))/M];
    [T,X] = ode45(fx,[0 TIMESTEP],[VirtualBot(1,1);VirtualBotDot(1,1)]);
    VirtualBot(1,1) = real(X(end,1));
    VirtualBotDot(1,1) = real(X(end,2));
    fy = @(t,y) [y(2); (fyvBS-(Bz+kd)*y(2))/M];
    [T,Y] = ode45(fy,[0 TIMESTEP],[VirtualBot(1,2);VirtualBotDot(1,2)]);
    VirtualBot(1,2) = real(Y(end,1));
    VirtualBotDot(1,2) = real(Y(end,2));
    
    addpoints(VirtualTrajectory,VirtualBot(1,1), VirtualBot(1,2));
    
    
    if(mod(zed,(TOTALTIME/TIMESTEP)/50) == 0)
        zed
        drawnow
    end
    if(mod(zed,(TOTALTIME/TIMESTEP)/10) == 0)
        % circle(VirtualBot(1,1),VirtualBot(1,2),alpha,0.5);
        circle(VirtualBot(1,1),VirtualBot(1,2),0.2,1);
        border(robot(1:numberOfRobots,1),robot(1:numberOfRobots,2));
        drawnow
    end
end