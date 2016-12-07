clc; clear all; close all; color = 'kbgrcmy'; colorVal=1;
figure; hold on; grid on;
axis([0 100 0 100]); axis square;

numberOfRobots=8;
robot=[  ([10,10])
         ([50,10])
         ([90,10])
         ([90,50])
         ([90,90])
         ([50,90])
         ([10,90])
         ([10,50])
         ];
robotT1 = robot; %for previous position info - @ T-1

robotForces=[    ([10,8])
                 ([2,12])
                 ([-8,10])
                 ([-5,-10])
                 ([-12,-8])
                 ([-3,-11])
                 ([5,-13])
                 ([15,7])
                 ];

fxkdes = robotForces(:,1);
fykdes = robotForces(:,2);

for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.5,2);
end

% Create Animated Line Objects for Each robot, different colors
robotTrajectory = [animatedline('Color',color(colorVal),'LineWidth',2)];
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal=1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end

numberOfObstacles=10;
obstacle = randi([20,80],numberOfObstacles,2);
obstacleSize = [randi([3,6],numberOfObstacles,1) randi([4,8],numberOfObstacles,1)];

A =  zeros(numberOfObstacles,1);
B =  zeros(numberOfObstacles,1);
ra = ones(numberOfObstacles,1);
rk = zeros(numberOfObstacles,1);

for i = 1: numberOfObstacles
    v1 = obstacleSize(i,1);
    v2 = obstacleSize(i,2);
    rectangle('Position',[obstacle(i,1)-v1 obstacle(i,2)-v2 v1*2 v2*2]);
    A(i)=sqrt(1/(2*((v1)^2)));
    B(i)=sqrt(1/(2*((v2)^2)));
    ellipse(obstacle(i,1),obstacle(i,2),1/A(i),1/B(i));
end


fxkOA = zeros(numberOfRobots,1);
fykOA = zeros(numberOfRobots,1);

M=1;
Bz=1;
kd=9;

xdot = zeros(numberOfRobots,1);
ydot = zeros(numberOfRobots,1);

psi = zeros(numberOfRobots,1);
chi = zeros(numberOfRobots,1);

zed = zeros(3,numberOfRobots);

t = 0;
%while WayPoint<(length(trajectory)-1)
while t<1000
    t = t+1;
    for i=1:numberOfRobots
        xk = robot(i,1);
        yk = robot(i,2);

        fxkObstacle = 0;
        fykObstacle = 0;
        for j=1:numberOfObstacles
            chi(i) = atan2(obstacle(j,2)-yk,obstacle(j,1)-xk);
            psi(i) = atan2(robot(i,2)-robotT1(i,2),robot(i,1)-robotT1(i,1));

            if(mod(psi(i)-chi(i),2*pi)<=pi)
                fxkrc  =  (B(j)/A(j))*(yk-obstacle(j,2));
                fykrc  = -(A(j)/B(j))*(xk-obstacle(j,1));
                fxkr   = fxkrc;
                fykr   = fykrc;
            else
                fxkrcc = -(B(j)/A(j))*(yk-obstacle(j,2));
                fykrcc =  (A(j)/B(j))*(xk-obstacle(j,1));
                fxkr   = fxkrcc;
                fykr   = fykrcc;
            end

            fxkrn = fxkr/norm([fxkr;fykr]);
            fykrn = fykr/norm([fxkr;fykr]);

            rk(j) =sqrt(((A(j)^2*(xk-obstacle(j,1))^2+B(j)^2*(yk-obstacle(j,2))^2-1)));

            if(rk(j)<=ra(j))
                fxkObstacle = fxkObstacle + ((g(fxkdes(i),fykdes(i))*fxkrn)/rk(j)^2)*((1/rk(j))-(1/ra(j)));
                fykObstacle = fykObstacle + ((g(fxkdes(i),fykdes(i))*fykrn)/rk(j)^2)*((1/rk(j))-(1/ra(j)));
            end        
        end
        
        fxkOA(i) = fxkdes(i) + fxkObstacle;
        fykOA(i) = fykdes(i) + fykObstacle;
        
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
        
        robotT1(i,:) = robot(i,:);
        robot(i,:)=[xk yk];
        addpoints(robotTrajectory(i),xk,yk);
        if(mod(t,50)==0)
            t
            drawnow
        end
    end
end

for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),1,4);
end