clc; clear all; close all; color = 'kbgrcmy'; colorVal=1;

% Generate Random Start Points for Robots
numberOfRobots=4;
%robot=cell(1,numberOfRobots+1);

% robot=[ randi(robotX) ...
%         randi(robotY)]
     
 robot=[ ([-5,5])
         ([6,6])
         ([5,4])
         ([5,5])];
     
figure; hold on; grid on;
axis([-6 6 -6 6]); axis square;

for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2);
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

x0 = 1;
y0 = 1;
v1=2; v2=2;

%Draw a rectangle
rectangle('Position',[x0-v1 y0-v2 v1*2 v2*2]);%,'FaceColor',[1 0 0]);

A=sqrt(1/(2*((v1)^2)));
B=sqrt(1/(2*((v2)^2)));

ra = 1.7;
rk = zeros(numberOfRobots,1);

fxkdes = zeros(numberOfRobots,1);
fykdes = zeros(numberOfRobots,1);

% for figure 4
fxkdes = [8;8];
fykdes = [8;8];

fxkOA = zeros(numberOfRobots,1);
fykOA = zeros(numberOfRobots,1);

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
FORCE_THRESHOLD = 0.01
FORCE_MAX = 100

xdot = zeros(numberOfRobots,1);
ydot = zeros(numberOfRobots,1);

psi = zeros(numberOfRobots,1);
chi = zeros(numberOfRobots,1);

rktest=0;
numtest=0;
%Main LOOP
t = 0;
%while WayPoint<(length(trajectory)-1)
% Load First Trajectory point and Draw Circle Around it
VirtualBot=[-0,-5];
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

force=[0 0];
rate=[0 0];
%Main LOOP
WayPoint = 1;
%while WayPoint<(length(trajectory)-1)

while WayPoint<3
    WayPoint = WayPoint+1
    % VirtualBot=trajectory(WayPoint,:);
    movement = ones(numberOfRobots,1);
    
    iterations = 0; % Variable to track iterations in each inner loop
    
    % Inner Loop
    while sum(movement)>0
        iterations = iterations+1;
        
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
            if i==1
                force = [force;FxkVS(i) FykVS(i)];
               
            end
        end
        
        
        
        
        
        
        for i=1:numberOfRobots

            
        xk = robot(i,1);
        yk = robot(i,2);
        
        rk(i)=sqrt(((A^2*(xk-x0)^2+B^2*(yk-y0)^2-1)))
        if(i==1)
            rktest = [rktest;rk(i)];
        end

            psi(i)=atan2(ydot(i),xdot(i));
            chi(i)=atan2(y0-yk,x0-xk);

            if(mod(psi(i)-chi(i),2*pi)<=pi)            
                fxkrc  =  (B/A)*(yk-y0);
                fykrc  = -(A/B)*(xk-x0);            
                fxkr = fxkrc;
                fykr = fykrc;
            else
                fxkrcc = -(B/A)*(yk-y0);
                fykrcc =  (A/B)*(xk-x0);
                fxkr = fxkrcc;
                fykr = fykrcc;
            end
            
            fxkrn = fxkr/norm([fxkr;fykr]);
            fykrn = fykr/norm([fxkr;fykr]);

            
                  
            x_pos_new = robot(i,1);
            y_pos_new = robot(i,2);
            
            
        if(rk(i)<=ra)
%             display("nOW stopped")
            FxkVS(i) = FxkVS(i) + ((g(FxkVS(i),FykVS(i))*fxkrn)/rk(i)^2)*((1/rk(i))-(1/ra));
            FykVS(i) = FykVS(i) + ((g(FxkVS(i),FykVS(i))*fykrn)/rk(i)^2)*((1/rk(i))-(1/ra));
        else
            FxkVS(i) = FxkVS(i);
            FykVS(i) = FykVS(i);
        end
            
            
            
            if(abs(FxkVS(i))>FORCE_THRESHOLD)
                if(abs(FxkVS(i))>FORCE_MAX)
                    FxkVS(i)=(FxkVS(i)/abs(FxkVS(i)))*FORCE_MAX;
                end
                fx = @(t,x) [x(2); (FxkVS(i)-(B+kd)*x(2))/M];
                [T,X]=ode45(fx,[0,0.05],[robot(i,1);xdot(i)]);
                [m,z] = size(X);
                x_pos_new=X(m,1);
                xdot(i)=X(m,2);
            end
            
            if(abs(FykVS(i))>FORCE_THRESHOLD)
                if(abs(FykVS(i))>FORCE_MAX)
                    FykVS(i)=(FykVS(i)/abs(FykVS(i)))*FORCE_MAX;
                end
                fy = @(t,y) [y(2); (FykVS(i)-(B+kd)*y(2))/M];
                [T,Y]=ode45(fy,[0,0.05],[robot(i,2);ydot(i)]);
                [m,z] = size(Y);
                y_pos_new=Y(m,1);
                ydot(i)=Y(m,2);
            end
            
            if abs(FxkVS(i))<=FORCE_THRESHOLD &&  abs(FykVS(i))<=FORCE_THRESHOLD
                movement(i,1) = 0;
            elseif abs(FxkVS(i))>FORCE_THRESHOLD || abs(FykVS(i))>FORCE_THRESHOLD
                 movement(i,1) = 1;
            end
            
            if i ==1
                rate=[rate;(x_pos_new - xdot(i)) (y_pos_new - ydot(i))];
            end
            robot(i,:)=[x_pos_new y_pos_new];
            addpoints(robotTrajectory(i),x_pos_new,y_pos_new);
            addpoints(VirtualTrajectory,VirtualBot(1,1),VirtualBot(1,2));
            pause
        end
    end
end
for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.2);
end        
        