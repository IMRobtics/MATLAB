clc; clear all; close all; color = 'kbgrcmy'; colorVal=1;

% Generate Random Start Points for Robots
numberOfRobots=7;
%robot=cell(1,numberOfRobots+1);

% robot=[ randi(robotX) ...
%         randi(robotY)]
     
 robot=[ ([14,14])
         ([14,12])
         ([10,15])
         ([10,20])
         ([20,10])
         ([14,12])
         ([15,10])
         ];
robotT1 = robot;
 
figure; hold on; grid on;
axis([8 35 8 35]); axis square;

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

x0 = 20;
y0 = 20;
v1=3; v2=2;
rectWidth =v1*2;
rectHeight=v2*2;

%Draw a rectangle
rectangle('Position',[x0-v1 y0-v2 rectWidth rectHeight]);%,'FaceColor',[1 0 0]);
% ellipse(x0,y0,3,2);
% ellipse(x0,y0,6,4);

A=sqrt(1/(2*((v1)^2)));
B=sqrt(1/(2*((v2)^2)));

% ellipse(x0,y0,1/A,1/B);

ra = 1; % ZP, 1
rk = zeros(numberOfRobots,1);

fxkdes = zeros(numberOfRobots,1);
fykdes = zeros(numberOfRobots,1);

fxkdes = ones(numberOfRobots,1);
fykdes = ones(numberOfRobots,1);
% for figure 4
fxkdes = fxkdes.*8;
fykdes = fykdes.*8;

fxkdes(4) = 8;
fykdes(4) = 0;
fxkdes(5) = 0;
fykdes(5) = 8;

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
while t<300
    t = t+1;
%     if(t>100)
%         pause
%     end
    for i=1:numberOfRobots
        xk = robot(i,1);
        yk = robot(i,2);
        rk(i) =sqrt(((A^2*(xk-x0)^2+B^2*(yk-y0)^2-1)));


        if(rk(i)<=ra)

            chi(i)=atan2(y0-yk,x0-xk);

    %         xkdot =  (B/A)*(yk-y0);
    %         ykdot = -(A/B)*(xk-x0);
    %         psi(i)=atan2(ykdot,xkdot);

    %         psi(i) = atan2((-A/B)*(robot(i,2)-robotT1(i,2)),(B/A)*(robot(i,1)-robotT1(i,1)));
            psi(i) = atan2(robot(i,2)-robotT1(i,2),robot(i,1)-robotT1(i,1));

            zed(1,i) = psi(i);
            zed(2,i) = chi(i);

            zed(i) = mod(psi(i)-chi(i),2*pi);
            if(zed(i)<=pi)
    %         if(psi(i)>=chi(i))
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

    %         if(i>2)x
    %             fxkrcc = -(B/A)*(yk-y0);
    %             fykrcc =  (A/B)*(xk-x0);
    %             fxkr   = fxkrcc;
    %             fykr   = fykrcc;
    %         else
    %             fxkrc  =  (B/A)*(yk-y0);
    %             fykrc  = -(A/B)*(xk-x0);            
    %             fxkr   = fxkrc;
    %             fykr   = fykrc;    
    %         end

            fxkrn = fxkr/norm([fxkr;fykr]);
            fykrn = fykr/norm([fxkr;fykr]);
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
        xk=X(m,1);
        xdot(i)=X(m,2);

        fy = @(t,y) [y(2); (fykOA(i)-(Bz+kd)*y(2))/M];
        [T,Y]=ode45(fy,[0,0.05],[yk;ydot(i)]);
        [m,z] = size(Y);
        yk=Y(m,1);
        ydot(i)=Y(m,2);
        
        robotT1(i,:) = robot(i,:);
        robot(i,:)=[xk yk];
        addpoints(robotTrajectory(i),xk,yk);
        if(mod(t,50)==0)
            t;
            drawnow
        end
    end
end

for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2,4);
end