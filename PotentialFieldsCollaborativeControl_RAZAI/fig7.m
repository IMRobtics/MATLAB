clc; clear all; close all; color = 'kbgrcmy'; colorVal=1;
figure; hold on; grid on;
axis([10 55 10 55]); axis square;

numberOfRobots=4;

robot=[ ([15,20])
        ([16,16])
        ([20,17])
        ([18,22])
        ];
robotT1 = robot;
 
for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2,2);
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

numberOfObstacles=1;
% obstacle = randi([20,80],numberOfObstacles,2);
% obstacleSize = [randi([3,6],numberOfObstacles,1) randi([4,8],numberOfObstacles,1)];
obstacle = [25 25];
obstacleSize = [4 3];

A =  zeros(numberOfObstacles,1);
B =  zeros(numberOfObstacles,1);
ra = ones(numberOfObstacles,1);
rk = zeros(numberOfObstacles,numberOfRobots);

for i = 1: numberOfObstacles
    v1 = obstacleSize(i,1);
    v2 = obstacleSize(i,2);
    rectangle('Position',[obstacle(i,1)-v1 obstacle(i,2)-v2 v1*2 v2*2]);
    A(i)=sqrt(1/(2*((v1)^2)));
    B(i)=sqrt(1/(2*((v2)^2)));
    ellipse(obstacle(i,1),obstacle(i,2),1/A(i),1/B(i));
end

fxkBS = zeros(numberOfRobots,1);
fykBS = zeros(numberOfRobots,1);

M=1;
Bz=1;
kd=9;
ksk=1;
kr=0.15;
alpha=3;
qk=10;
q(1:numberOfRobots)=qk;
m(1:numberOfRobots)=M;

xdot = zeros(numberOfRobots,1);
ydot = zeros(numberOfRobots,1);

psi = zeros(numberOfRobots,1);
chi = zeros(numberOfRobots,1);

% Load First Trajectory point and Draw Circle Around it
VirtualBot=[17.5, 18];
VirtualBotDot=[0,0];
circle(VirtualBot(1,1),VirtualBot(1,2),alpha,2);
circle(VirtualBot(1,1),VirtualBot(1,2),0.2,2);
VirtualTrajectory = animatedline('Color','r','LineWidth',2,'LineStyle','-.');

Fxk_dist = zeros(numberOfRobots,numberOfRobots);
Fxk = zeros(1,numberOfRobots);
Attractive_Force_x = zeros(1,numberOfRobots);
FxkVS = zeros(1,numberOfRobots);

Fyk_dist = zeros(numberOfRobots,numberOfRobots);
Fyk = zeros(1,numberOfRobots);
Attractive_Force_y = zeros(1,numberOfRobots);
FykVS = zeros(1,numberOfRobots);

FxkVST1 = zeros(1,numberOfRobots);
FykVST1 = zeros(1,numberOfRobots);
FxkVSrkra = zeros(1,numberOfRobots);
FykVSrkra = zeros(1,numberOfRobots);

FORCE_MAX = 10;
FORCE_THRESHOLD = 0.5;


% Force on Virtual Bot
fxvdes = 8;
fyvdes = 8;

Tau = 4;
kp = 5;


% movement = ones(numberOfRobots,1);
% t = 0; zed=0;
% while sum(movement)>0
%     t = t+1;
%     %Computing distance of individual robot from all other robots
%     r=zeros(numberOfRobots,numberOfRobots);
%     for i=1:numberOfRobots
%         for j=1:numberOfRobots
%             a=robot(i,:)-robot(j,:);
%             r(i,j)=sqrt(a(1)^2+a(2)^2);
%         end
%     end
% 
%     %Computing orientation of individual from all other robots
%     thetaX=zeros(numberOfRobots,numberOfRobots);
%     thetaY=zeros(numberOfRobots,numberOfRobots);
%     for i=1:numberOfRobots
%         for j=1:numberOfRobots
%             if i==j
%                 thetaX(i,j)=0;thetaY(i,j)=0;
%             elseif i~=j
%                 a=robot(i,:)-robot(j,:);
%                 thetaX(i,j)=a(1)/abs(r(i,j));
%                 thetaY(i,j)=a(2)/abs(r(i,j));
%                 %theta(i,j)=atan2(a(2),a(1));
%             end
%         end
%     end
% 
%     %     Computing Electrostatic forces (Repulsive) on individual robot
%     %     from all other robots
%     Electrostatic_Forces=zeros(numberOfRobots,numberOfRobots);
%     for i=1:numberOfRobots
%         for j=1:numberOfRobots
%             if i==j
%                 Electrostatic_Forces(i,j)=0;
%             elseif i~=j
%                 Electrostatic_Forces(i,j)=(kr*q(i)*q(j))/(r(i,j)^2);
%             end
%         end
%     end
% 
%     %Decomposition of Electrostatic forces in x and y components
%     for i=1:numberOfRobots
%         for j=1:numberOfRobots
%             Fxk_dist(i,j)=Electrostatic_Forces(i,j)*thetaX(i,j);
%             Fyk_dist(i,j)=Electrostatic_Forces(i,j)*thetaY(i,j);
%         end
%     end
% 
%     for i=1:numberOfRobots
%         Fxk(i)=sum(Fxk_dist(i,1:numberOfRobots));
%         Fyk(i)=sum(Fyk_dist(i,1:numberOfRobots));
%     end
% 
%     %Computing x and y components of Attractive Force (Equation 8 in JP)
%     for i=1:numberOfRobots
%         a=robot(i,:)-VirtualBot;
%         Attractive_Force_x(i)=ksk*(a(1)*(a(1)^2+a(2)^2-alpha^2));
%         Attractive_Force_y(i)=ksk*(a(2)*(a(1)^2+a(2)^2-alpha^2));
%     end
% 
%     %Computing resultant forces on each robot
%     for i=1:numberOfRobots
%         FxkVS(i)=Fxk(i)-Attractive_Force_x(i);
%         FykVS(i)=Fyk(i)-Attractive_Force_y(i);
%     end
% 
%     for i=1:numberOfRobots
%         if(abs(FxkVS(i))>FORCE_MAX)
%             FxkVS(i)=(FxkVS(i)/abs(FxkVS(i)))*FORCE_MAX;
%         end
%         if(abs(FykVS(i))>FORCE_MAX)
%             FykVS(i)=(FykVS(i)/abs(FykVS(i)))*FORCE_MAX;
%         end
%     end
% 
%     for i=1:numberOfRobots
%         xk = robot(i,1);
%         yk = robot(i,2);
%         fxkObstacle = 0;
%         fykObstacle = 0;
%         
% %         for j=1:numberOfObstacles
% %             rk(j) =sqrt(((A(j)^2*(xk-obstacle(j,1))^2+B(j)^2*(yk-obstacle(j,2))^2-1)));
% % 
% %             if(rk(j)<=ra(j))
% %                 chi(i) = atan2(obstacle(j,2)-yk,obstacle(j,1)-xk);
% %                 psi(i) = atan2(robot(i,2)-robotT1(i,2),robot(i,1)-robotT1(i,1));
% % 
% %                 if(mod(psi(i)-chi(i),2*pi)<=pi)
% %                     fxkrc  =  (B(j)/A(j))*(yk-obstacle(j,2));
% %                     fykrc  = -(A(j)/B(j))*(xk-obstacle(j,1));
% %                     fxkr   = fxkrc;
% %                     fykr   = fykrc;
% %                 else
% %                     fxkrcc = -(B(j)/A(j))*(yk-obstacle(j,2));
% %                     fykrcc =  (A(j)/B(j))*(xk-obstacle(j,1));
% %                     fxkr   = fxkrcc;
% %                     fykr   = fykrcc;
% %                 end
% % 
% %                 fxkrn = fxkr/norm([fxkr;fykr]);
% %                 fykrn = fykr/norm([fxkr;fykr]);
% %                 fxkObstacle = fxkObstacle + ((g(FxkVS(i),FykVS(i))*fxkrn)/rk(j)^2)*((1/rk(j))-(1/ra(j)));
% %                 fykObstacle = fykObstacle + ((g(FxkVS(i),FykVS(i))*fykrn)/rk(j)^2)*((1/rk(j))-(1/ra(j)));
% %             end       
% %         end
% 
%         fxkBS(i) = FxkVS(i) + fxkObstacle;
%         fykBS(i) = FykVS(i) + fykObstacle;
% 
%         if(abs(fxkBS(i))>FORCE_THRESHOLD)
%             if(abs(fxkBS(i))>FORCE_MAX)
%                 fxkBS(i)=(fxkBS(i)/abs(fxkBS(i)))*FORCE_MAX;
%             end
%             fx = @(t,x) [x(2); (fxkBS(i)-(Bz+kd)*x(2))/M];
%             [T,X]=ode45(fx,[0,0.05],[xk;xdot(i)]);
%             [m,z] = size(X);
%             xk=real(X(m,1));
%             xdot(i)=real(X(m,2));
%         end
% 
%         if(abs(fykBS(i))>FORCE_THRESHOLD)
%             if(abs(fykBS(i))>FORCE_MAX)
%                 fykBS(i)=(fykBS(i)/abs(fykBS(i)))*FORCE_MAX;
%             end
%             fy = @(t,y) [y(2); (fykBS(i)-(Bz+kd)*y(2))/M];
%             [T,Y]=ode45(fy,[0,0.05],[yk;ydot(i)]);
%             [m,z] = size(Y);
%             yk=real(Y(m,1));
%             ydot(i)=real(Y(m,2));
%         end
% 
%         if abs(fxkBS(i))<=FORCE_THRESHOLD &&  abs(fykBS(i))<=FORCE_THRESHOLD
%             movement(i,1) = 0;
%         elseif abs(fxkBS(i))>FORCE_THRESHOLD || abs(fykBS(i))>FORCE_THRESHOLD
%              movement(i,1) = 1;
%         end
% 
%         robotT1(i,:) = robot(i,:);
%         robot(i,:)=[xk yk];
%         addpoints(robotTrajectory(i),xk,yk);
%         if(mod(t,50)==0)
%             t;
%             drawnow
%         end
%     end
% end

TIMESTEP = 1;

obstacleFlag = ones(numberOfRobots,1);
t = 0; zed=0;
while zed<2000
    zed = zed+1;

    t = t+1;

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
        FxkVST1(i) = FxkVS(i);
        FykVST1(i) = FykVS(i);
        FxkVS(i)=Fxk(i)-Attractive_Force_x(i);
        FykVS(i)=Fyk(i)-Attractive_Force_y(i);
    end    
    
    
    for i=1:numberOfRobots
        xk = robot(i,1);
        yk = robot(i,2);

        fxkObstacle = 0;
        fykObstacle = 0;
        
        for j=1:numberOfObstacles
            rk(j,i) =sqrt(((A(j)^2*(xk-obstacle(j,1))^2+B(j)^2*(yk-obstacle(j,2))^2-1)));
            if(rk(j,i)<=ra(j))
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
                
                fxkObstacle = fxkObstacle + ((g(FxkVSrkra(i),FykVSrkra(i))*fxkrn)/rk(j,i)^2)*((1/rk(j,i))-(1/ra(j)));
                fykObstacle = fykObstacle + ((g(FxkVSrkra(i),FykVSrkra(i))*fykrn)/rk(j,i)^2)*((1/rk(j,i))-(1/ra(j)));
            end
        end
        
        
       for j=1:numberOfObstacles
            rk(j,i) =sqrt(((A(j)^2*(xk-obstacle(j,1))^2+B(j)^2*(yk-obstacle(j,2))^2-1)));
            if(rk(j,i)<=ra(j))
                if(obstacleFlag(i)==j)
                    FxkVSrkra(i) = FxkVST1(i);
                    FykVSrkra(i) = FykVST1(i);
                    obstacleFlag(i)=obstacleFlag(i)+1;
                end
                fxkBS(i) = FxkVSrkra(i) + fxkObstacle;
                fykBS(i) = FykVSrkra(i) + fykObstacle;
            else
                fxkBS(i) = FxkVSrkra(i)*exp(-Tau*rk(j,i)) + FxkVS(i)*(1-exp(-Tau*rk(j,i)));
                fykBS(i) = FykVSrkra(i)*exp(-Tau*rk(j,i)) + FykVS(i)*(1-exp(-Tau*rk(j,i)));
            end
       end

       
       xm =sum(robot(:,1))/numberOfRobots; 
       ym =sum(robot(:,2))/numberOfRobots; 
       for j=1:numberOfObstacles
           rm = min(rk);
           if(rm<ra(j))
               fxvBS = fxvdes + kp*(xm-VirtualBot(1,1))*(1-exp(-Tau*rm));
               fyvBS = fyvdes + kp*(ym-VirtualBot(1,2))*(1-exp(-Tau*rm));
           else
               fxvBS = fxvdes;
               fyvBS = fyvdes;
           end
       end
       
       

        [fxkBS(i),fykBS(i)] = forceConstrain(fxkBS(i),fykBS(i),FORCE_MAX);

        
        % %============== Solves simple pendulum differential equations =============
        % deq1=@(t,x) [x(2); -g/l * sin(x(1))]; % Pendulum equations uncoupled
        % [t,sol] = ode45(deq1,[0 runtime],[initialangle1 initialangle2]);  % uses a numerical ode solver
        % sol1 = sol(:,1)'; % takes the transpose for plots
        % sol2 = sol(:,2)';        
        
        if(abs(fxkBS(i))>FORCE_THRESHOLD)
            fx = @(t,x) [x(2); (fxkBS(i)-(Bz+kd)*x(2))/M];
            [T,X]=ode45(fx,[0 TIMESTEP],[xk;xdot(i)]);
            [m,z] = size(X);
            xk=real(X(m,1));
            xdot(i)=real(X(m,2));
        end

        if(abs(fykBS(i))>FORCE_THRESHOLD)
            fy = @(t,y) [y(2); (fykBS(i)-(Bz+kd)*y(2))/M];
            [T,Y]=ode45(fy,[0 TIMESTEP],[yk;ydot(i)]);
            [m,z] = size(Y);
            yk=real(Y(m,1));
            ydot(i)=real(Y(m,2));
        end

        robotT1(i,:) = robot(i,:);
        robot(i,:)=[xk yk];
        addpoints(robotTrajectory(i),xk,yk);
        if(mod(t,50)==0)
            t;
            drawnow
        end
    end

    

    [fxvBS,fyvBS] = forceConstrain(fxvBS,fyvBS,FORCE_MAX);

    fx = @(t,x) [x(2); (fxvBS-(Bz+kd)*x(2))/M];
    [T,X]=ode45(fx,[0 TIMESTEP],[VirtualBot(1,1);VirtualBotDot(1,1)]);
    [m,z] = size(X);
    VirtualBot(1,1)=real(X(m,1));
    VirtualBotDot(1,1)=real(X(m,2));

    fy = @(t,y) [y(2); (fyvBS-(Bz+kd)*y(2))/M];
    [T,Y]=ode45(fy,[0 TIMESTEP],[VirtualBot(1,2);VirtualBotDot(1,2)]);
    [m,z] = size(Y);
    VirtualBot(1,2)=real(Y(m,1));
    VirtualBotDot(1,2)=real(Y(m,2)); 
    addpoints(robotTrajectory(i),xk,yk);
    if(mod(zed,250)==0 || zed==1)
%         circle(VirtualBot(1,1),VirtualBot(1,2),alpha,0.5);
        circle(VirtualBot(1,1),VirtualBot(1,2),0.2,2);
        border(robot(1:numberOfRobots,1),robot(1:numberOfRobots,2)); 
        zed
        drawnow
    end  
    
end