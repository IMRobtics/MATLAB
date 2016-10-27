clc;
clear all;
close all;

number_of_robots=3;
robot=cell(1,number_of_robots+1);
POSITIONS=zeros(2,number_of_robots);

%Initial Position of the robots
robot{1}=[-4,6];
robot{2}=[-8,6];
robot{3}=[6,8];
robot{4}=[8,-8];
robot{5}=[2,-8];
robot{6}=[0,7];
robot{7}=[-6,-6];
robot{8}=[0,0];
robot{number_of_robots+1}=robot{1};

figure;
hold on; grid on; 
rLine = [animatedline('Color','r','LineWidth',3);
         animatedline('Color','b','LineWidth',3);
         animatedline('Color','k','LineWidth',3);
         animatedline('Color','g','LineWidth',3);
         ];
axis([-8 8 -8 8]); axis square;

xdot = zeros(number_of_robots,1);
ydot = zeros(number_of_robots,1);

M=1;
B=1;
kd=9;
ksk=1;
kr=0.15;

xc=0;
yc=0;
hyp_robot=[xc,yc];

alpha=2;
qk=2.5;

%Vector to hold the position history
robot_Vec=[];

%Charges on each robot
q(1:number_of_robots)=qk;

%Mass of each robot
m(1:number_of_robots)=M;

%Electrostatic Constant
K=10;

%drawing a circle of radius=alpha
count=1;
for theta=0:0.1:2*pi
x_circle(count)=(alpha+0.5)*cos(theta);
y_circle(count)=(alpha+0.5)*sin(theta);
count=count+1;
end

x_circle=x_circle+hyp_robot(1);
y_circle=y_circle+hyp_robot(2);
plot(x_circle,y_circle,'r-')

for i=1:number_of_robots+1
    dummy=robot(i);
    xy_pos=cell2mat(dummy);
    x_pos=xy_pos(1);
    y_pos=xy_pos(2);
    POSITIONS(i,1) = x_pos;
    POSITIONS(i,2) = y_pos;
end

xPOINTS = POSITIONS(:,1);
yPOINTS = POSITIONS(:,2);
plot(xPOINTS,yPOINTS,'X')
k = boundary(xPOINTS,yPOINTS,0);
plot(xPOINTS(k),yPOINTS(k));

pause
for t=1:150  %total simulation time
    %Computing distance of individual robot from all other robots
    r=zeros(number_of_robots,number_of_robots);
    for i=1:number_of_robots
        for j=1:number_of_robots
            a=robot{i}-robot{j};
            r(i,j)=sqrt(a(1)^2+a(2)^2);
        end
    end

    %Computing orientation of individual from all other robots
    theta=zeros(number_of_robots,number_of_robots);
    for i=1:number_of_robots
        for j=1:number_of_robots
            if i==j
                theta(i,j)=0;
            elseif i~=j
                a=robot{i}-robot{j};
                theta(i,j)=atan2(a(2),a(1));
            end
        end
    end

%     Computing Electrostatic forces (Repulsive) on individual robot
%     from all other robots
    Electrostatic_Forces=zeros(number_of_robots,number_of_robots);
    for i=1:number_of_robots
        for j=1:number_of_robots
            if i==j
                Electrostatic_Forces(i,j)=0;
            elseif i~=j
                Electrostatic_Forces(i,j)=(K*q(i)*q(j))/(r(i,j)^2);
            end
        end
    end
  
    %Decomposition of Electrostatic forces in x and y components
    for i=1:number_of_robots
        for j=1:number_of_robots
            Fxk_dist(i,j)=Electrostatic_Forces(i,j)*cos(theta(i,j));
            Fyk_dist(i,j)=Electrostatic_Forces(i,j)*sin(theta(i,j));
        end
    end
    
    for i=1:number_of_robots
        Fxk(i)=sum(Fxk_dist(i,1:number_of_robots));
        Fyk(i)=sum(Fyk_dist(i,1:number_of_robots));
    end    
    
    %Computing x and y components of Attractive Force (Equation 8 in JP)
   
    for i=1:number_of_robots
        a=robot{i}-hyp_robot;
        Attractive_Force_x(i)=ksk*(a(1)*(a(1)^2+a(2)^2-alpha^2));
        Attractive_Force_y(i)=ksk*(a(2)*(a(1)^2+a(2)^2-alpha^2));
    end
    
    %Computing resultant forces on each robot
    for i=1:number_of_robots
        FxkVS(i)=Fxk(i)-Attractive_Force_x(i);
        FykVS(i)=Fyk(i)-Attractive_Force_y(i);
    end
     
    for i=1:number_of_robots
        dummy=robot(i);
        xy_pos=cell2mat(dummy);
        x_pos=xy_pos(1);
        y_pos=xy_pos(2);
        
        fx = @(t,x) [x(2); (FxkVS(i)-(B+kd)*x(2))/M];
        [T,X]=ode45(fx,[0,0.05],[x_pos;xdot(i)]);
        fy = @(t,y) [y(2); (FykVS(i)-(B+kd)*y(2))/M];
        [T,Y]=ode45(fy,[0,0.05],[y_pos;ydot(i)]);
        [m,z] = size(X);
        x_pos_new=X(m,1);
        xdot(i)=X(m,2);
        [m,z] = size(Y);
        y_pos_new=Y(m,1);
        ydot(i)=Y(m,2);
        robot{i}=[x_pos_new y_pos_new];
        addpoints(rLine(i),x_pos_new,y_pos_new);
        robot_Vec=[robot_Vec;robot];
    end
    drawnow
end

robot{number_of_robots+1}=robot{1};
for i=1:number_of_robots+1
    dummy=robot(i);
    xy_pos=cell2mat(dummy);
    x_pos=xy_pos(1);
    y_pos=xy_pos(2);
    POSITIONS(i,1) = x_pos;
    POSITIONS(i,2) = y_pos;
end
% scatter(POSITIONS(:,1),POSITIONS(:,2),'O')
% line(POSITIONS(:,1),POSITIONS(:,2),'color','m','LineWidth',3)

xPOINTS = POSITIONS(:,1);
yPOINTS = POSITIONS(:,2);
plot(xPOINTS,yPOINTS,'X')
k = boundary(xPOINTS,yPOINTS,0);
plot(xPOINTS(k),yPOINTS(k));



%plot(simout.signals.values,simout1.signals.values);
