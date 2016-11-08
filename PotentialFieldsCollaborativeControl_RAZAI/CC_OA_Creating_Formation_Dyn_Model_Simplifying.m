clc;
clear all;
close all;

%attraction and repulsion between charges

%A negative charge is places at the centre and four chrages are placed at
%different location

%-----------------------------------------------------------------------
%Initialization of parameters
%-----------------------------------------------------------------------
number_of_robots=5;

%Dynamic characteristic of the robot
%My values with which code is working
M=0.1;
B_model=0.1;
kd=2;
ksk=1;
kr=0.15;
alpha=3; %radius of the circle
kp=10;
tau=50;
fxvdes=1.5;  %force on the virtual robot
fyvdes=1.5;
ra=ones(1:number_of_robots);
tspan_ode45=[0.0 0.1];

%%%%  Values from paper
% % % M=1;
% % % B_model=1;
% % % kd=9;
% % % ksk=100;
% % % kr=10;
% % % alpha=3; %radius of the circle
% % % kp=10;
% % % tau=4;
% % % fxvdes=8;  %force on the virtual robot
% % % fyvdes=8;
% % % ra=ones(1:number_of_robots);
% % % tspan_ode45=[0.0 0.009];



%Charges on each robot
q(1:number_of_robots)=5;
% test=zeros(1,number_of_robots);
% OBSTACLE=zeros(1,number_of_robots);



%flag for testing whether the robots have developed a Swarm on not
%Initially the SWARM_FLAG is set to ZERO because the robots are not in a
%swarm formation

SWARM_FLAG=0;

%Defining and initializing angles for obstacle avoidance
Chi=zeros(1,number_of_robots);
Psi=zeros(1,number_of_robots);

%Forces on robots when using behavioural structure with swarm formation
fxkVS_rk_ra=zeros(1,number_of_robots);
fykVS_rk_ra=zeros(1,number_of_robots);
fkVS_rk_ra=zeros(1,number_of_robots);
FLAG=zeros(1,number_of_robots);


%Initialization of robots
robot=cell(1,number_of_robots);

%Initial Position of the robots
robot{1}=[5,5];
robot{2}=[2,4];
robot{3}=[4,4];
robot{4}=[5,1];
robot{5}=[1,-1];
robot{6}=[10,6];
hyp_robot=[4,5];  %position of the hypothetical (Virtual Leader) Robot


xdot = zeros(number_of_robots,1);
ydot = zeros(number_of_robots,1);
xdot_virtual=zeros(number_of_robots,1);
ydot_virtual=zeros(number_of_robots,1);


r=zeros(number_of_robots,number_of_robots);
theta=zeros(number_of_robots,number_of_robots);
Electrostatic_Forces=zeros(number_of_robots,number_of_robots);

%Vector to hold the position history
robot_Trajectory_Vec=[];
virtual_robot_Trajectory_Vec=[];

%-----------------------------------------------------------------------
%                   Code for Animated LInes
%------------------------------------------------------------------------

color = 'kbgrcmy'; colorVal=1;
robotTrajectory_ZP = [animatedline('Color',color(colorVal),'LineWidth',2)];
for i = 1: number_of_robots-1
    colorVal = colorVal+1;
    if(colorVal>7)
        colorVal=1;
    end
    robotTrajectory_ZP = [robotTrajectory_ZP;animatedline('Color',color(colorVal),'LineWidth',2)];
end

Virtual_robotTrajectory_ZP = [animatedline('Color',color(7),'LineWidth',2)];
% Virtual_robotTrajectory_ZP = [Virtual_robotTrajectory_ZP;animatedline('Color',color(7),'LineWidth',2)];

%--------------------------------------------------------------------------
%    Developing the Obstacle
%--------------------------------------------------------------------------
%Drawing ellipses arouond the obstacle
%xa,ya is the origin of the rectangle (bottom left corner as according to matlab rectangle command)
%(x0,y0) is the centre of the rectangle/obstacle
%the ellipse envelops a rectangle with (x0+-v1,y0+-v2) as its vertices

%defining some rectangular obstacles
origin_of_rectangle=[20,20];
width=4;
height=3;

figure(1);
rectangle('Position',[origin_of_rectangle width height],'Curvature',[0.05 0.05]),...
    axis square,hold on
axis ([0 25 0 25])
% axis([-10 20 -10 20])

xa=origin_of_rectangle(1);
ya=origin_of_rectangle(2);

v1=width/2;
v2=height/2;
%Center of the obstacle
x0=xa+width/2;
y0=ya+height/2;

%Plotting the ellipse encircling the obstacle
%defining range for x
x=-20:0.1:40;

A=sqrt(1/(2*(v1^2)));
B=sqrt(1/(2*(v2^2)));

y1=real(sqrt((1-A^2*(x-x0).^2)/B^2)+y0);
y2=real(-sqrt((1-A^2*(x-x0).^2)/B^2)+y0);
plot(x,y1,'g',x,y2,'g'),axis square,grid on,hold on

% B1=0.1:0.1:0.5
% A1=(A/B)*B1

%Defining ellipses around the obstacles
v1_expansion=v1:v1/10:v1*1.25;
A1=sqrt(1./(2*(v1_expansion.^2)));
B1=(B/A)*A1;

for i=1:length(B1)
    %     if(B1(i))>B
    %         disp('I am here')
    %     else
    y1=real(sqrt((1-A1(i)^2*(x-x0).^2)/B1(i)^2)+y0);
    y2=real(-sqrt((1-A1(i)^2*(x-x0).^2)/B1(i)^2)+y0);
    plot(x,y1,'--k',x,y2,'--k'),axis square,grid on,hold on
    %     end
    
end
%------------------------------------------------------------------------


for t=1:700  %total simulation time [The main BIG Loop]
    %     robot{i}=[x_pos_new y_pos_new];
    robot_Trajectory_Vec=[robot_Trajectory_Vec;robot];
    t
    
    for i=1:number_of_robots
        dummy=robot(i);
        xy_pos=cell2mat(dummy);
        x_pos(i)=xy_pos(1);
        y_pos(i)=xy_pos(2);
    end
    
    
    %----------------------------------------------------------------------
    %Calculation of the angle with which robot is approaching the obstacle
    %We are using condition t>3 because the robot must have moved a little
    %distance in order to calculate the heading angle. Heading angle is
    %based on its current and previous position.
    if(t>3)
        for i=1:number_of_robots
            
            final_pos=cell2mat(robot_Trajectory_Vec(t-1,i));
            initial_pos=cell2mat(robot_Trajectory_Vec(t-2,i));
            %         robot_Trajectory_Vec{t-2}
            %         robot_Trajectory_Vec{t-1}
            %         xy_coordinate=robot_Trajectory_Vec{t-1}-robot_Trajectory_Vec{t-2}
            xy_coordinate=final_pos-initial_pos;
            Chi(i)=atan2(y0-y_pos(i),x0-x_pos(i));    %Angle between the obstacle centre-robot  and the horizontal
            Psi(i)=atan2(xy_coordinate(2),xy_coordinate(1));   %Robot Heading angle with the horizontal
        end
        %     Chi, Psi
    end
    %---------------------------------------------------------------------
    %Testing whether the robot is approaching the obstacle
    %If the 'test' value is approx 1e-3 it means that the robot uis very
    %near the ellipse. If the robot is exactly on the ellipse then 'test'
    %must be ZERO. [This was the first approach I was using for obstacle avoidance]
    
    %The second approach for testing whetner the robot is approaching the
    %obstacle or not is based on the journal paper, it compares ra with rk
    %and then takes a decision.
    
    for i=1:number_of_robots
        %     test(i)=A1(end)^2*(x_pos(i)-x0)^2+B1(end)^2*(y_pos(i)-y0)^2-1;
        
        rk(i)=sqrt(A^2*(x_pos(i)-x0)^2+B^2*(y_pos(i)-y0)^2-1);
        
        
        
        %---------------------------------------------------------------------
        %   Defining forces to avoid obstacle
        %---------------------------------------------------------------------
  
        if (rk(i)<=ra(i))
            %         fxkOA=fxkdes;
            %         fykOA=fykdes;
            %
            %             SWARM_FLAG=0;
            fxkrc(i)=(B/A)*(y_pos(i)-y0);
            fykrc(i)=-(A/B)*(x_pos(i)-x0);
            fxkrcc(i)=-(B/A)*(y_pos(i)-y0);
            fykrcc(i)=(A/B)*(x_pos(i)-x0);
            
            %             fkdes(i)=sqrt(fxkdes(i)^2+fykdes(i)^2);
            
            if (mod(Psi(i)-Chi(i),2*pi)<=pi)   %Psi>=Chi
                fxkr(i)=fxkrc(i);
                fykr(i)=fykrc(i);
            elseif (mod(Psi(i)-Chi(i),2*pi)>pi) %Psi<Chi
                fxkr(i)=fxkrcc(i);
                fykr(i)=fykrcc(i);
            end
            
            
            mod_fkr(i)=sqrt(fxkr(i)^2+fykr(i)^2);
            fxkrn(i)=fxkr(i)/mod_fkr(i);
            fykrn(i)=fykr(i)/mod_fkr(i);
            
        end
    end
    
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %            Computing the new position of Virtual Robot
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    
    %Code for updating the Virtual robot position when I am using Obstacles
    rm=min(rk);
    
    
    %     if (sum(OBSTACLE)==0)
    if (rm>ra)
        fxvBS_virtual=fxvdes;
        fyvBS_virtual=fyvdes;
        
    elseif (rm<=ra)
        xm=mean(x_pos);
        ym=mean(y_pos);
        
        fxvBS_virtual=fxvdes+kp*(xm-hyp_robot(1))*(1-exp(-tau*rm));
        fyvBS_virtual=fyvdes+kp*(ym-hyp_robot(2))*(1-exp(-tau*rm));
    end
    
    
    fx_virtual = @(t,x) [x(2); (fxvBS_virtual-(B_model+kd)*x(2))/M];
    [T,X_virtual]=ode45(fx_virtual,tspan_ode45,[hyp_robot(1);xdot_virtual(i)]);
    [m,z] = size(X_virtual);
    x_pos_new_virtual=real(X_virtual(m,1));
    xdot_virtual(i)=real(X_virtual(m,2));
    
    
    fy_virtual = @(t,y) [y(2); (fyvBS_virtual-(B_model+kd)*y(2))/M];
    [T,Y_virtual]=ode45(fy_virtual,tspan_ode45,[hyp_robot(2);ydot_virtual(i)]);
    [m,z] = size(Y_virtual);
    y_pos_new_virtual=real(Y_virtual(m,1));
    ydot_virtual(i)=real(Y_virtual(m,2));
    
    
    hyp_robot=[x_pos_new_virtual y_pos_new_virtual];
    virtual_robot_Trajectory_Vec=[virtual_robot_Trajectory_Vec;hyp_robot];
    
    
    addpoints(Virtual_robotTrajectory_ZP,x_pos_new_virtual,y_pos_new_virtual);
    drawnow
    %---------------------------------------------------------------------
    %----------------------------------------------------------------------
    %            End of code Computing the new position of Virtual Robot
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    
    %Computing the force for the first robot
    %Equation (6) of the paper
    
    %Computing distance of individual robot from all other robots
    
    for i=1:number_of_robots
        for j=1:number_of_robots
            a=robot{i}-robot{j};
            r(i,j)=sqrt(a(1)^2+a(2)^2);
            if i==j
                theta(i,j)=0;
                Electrostatic_Forces(i,j)=0;
            elseif i~=j
                a=robot{i}-robot{j};
                theta(i,j)=atan2(a(2),a(1));
                Electrostatic_Forces(i,j)=(q(i)*q(j))/(r(i,j)^2);
            end
            EF_x(i,j)=Electrostatic_Forces(i,j)*cos(theta(i,j));
            EF_y(i,j)=Electrostatic_Forces(i,j)*sin(theta(i,j));
        end
    end
    
    
    
    for i=1:number_of_robots
        a=robot{i}-hyp_robot;
        Attractive_Force_x(i)=ksk*(a(1)*(a(1)^2+a(2)^2-alpha^2));
        Attractive_Force_y(i)=ksk*(a(2)*(a(1)^2+a(2)^2-alpha^2));
        fxkVS(i)=kr*sum(EF_x(i,1:number_of_robots))-Attractive_Force_x(i);
        fykVS(i)=kr*sum(EF_y(i,1:number_of_robots))-Attractive_Force_y(i);
        fkVS(i)=sqrt(fxkVS(i)^2+fykVS(i)^2);
        
        dummy=robot(i);
        xy_pos=cell2mat(dummy);
        x_pos(i)=xy_pos(1);
        y_pos(i)=xy_pos(2);
        
        
        %initializing x_pos_new and y_pos_new
        x_pos_new=x_pos(i);
        y_pos_new=y_pos(i);
        
        distance_from_hyp_robot=sqrt((x_pos(i)-hyp_robot(1))^2+(y_pos(i)-hyp_robot(2))^2);
        
        %---------------------------------------------------------------
        %    Forces with obstacles (Behavioral structure)
        %---------------------------------------------------------------
        
        %Force when the robot is approaching the obstacle
        
        if ((rk(i)-ra(i))<1e-3)
            %             disp('force when rk=ra');
            
            
            if (FLAG(i)==0)
                fxkVS_rk_ra(i)=fxkVS(i);
                fykVS_rk_ra(i)=fykVS(i);
                fkVS_rk_ra(i)=sqrt(fxkVS_rk_ra(i)^2+fykVS_rk_ra(i)^2);
                FLAG(i)=FLAG(i)+1;
            end
        end
        
        if(rk(i)<ra(i))
            
            FxkBS(i)= fxkVS_rk_ra(i)+((fkVS_rk_ra(i)*fxkrn(i))/(rk(i)^2))*(1/rk(i)-1/ra(i));
            FykBS(i)= fykVS_rk_ra(i)+((fkVS_rk_ra(i)*fykrn(i))/(rk(i)^2))*(1/rk(i)-1/ra(i));
            
            %Force when the robot has passed the obstacle
        elseif (rk(i)>ra(i))
     
            FxkBS(i)= fxkVS_rk_ra(i)*exp(-tau*rk(i))+fxkVS(i)*(1-exp(-tau*rk(i)));
            FykBS(i)= fykVS_rk_ra(i)*exp(-tau*rk(i))+fykVS(i)*(1-exp(-tau*rk(i)));
                     
            
        end
        fx = @(t,x) [x(2); (FxkBS(i)-(B_model+kd)*x(2))/M];
        [T,X]=ode45(fx,tspan_ode45,[x_pos(i);xdot(i)]);
        [m,z] = size(X);
        x_pos_new=real(X(m,1));
        xdot(i)=real(X(m,2));
        
        fy = @(t,y) [y(2); (FykBS(i)-(B_model+kd)*y(2))/M];
        [T,Y]=ode45(fy,tspan_ode45,[y_pos(i);ydot(i)]);
        [m,z] = size(Y);
        y_pos_new=real(Y(m,1));
        ydot(i)=real(Y(m,2));
        robot{i}=[x_pos_new y_pos_new];
        addpoints(robotTrajectory_ZP(i),x_pos_new,y_pos_new);
        %         addpoints(Virtual_robotTrajectory_ZP,x_pos_new,y_pos_new);
        drawnow
        
    end
    
    
    robots_trajectory=cell2mat(robot_Trajectory_Vec);
    k=1;
    for i=1:number_of_robots
        
        X_for_polygon(i)= robots_trajectory(end,k);
        Y_for_polygon(i)=robots_trajectory(end,k+1);
        k=k+2;
    end
    k=convhull(X_for_polygon,Y_for_polygon);
    figure(1);
    if t>200
        pause
%         delete(h1);
    end
%     h1=plot(X_for_polygon(k),Y_for_polygon(k));
    robots_trajectory=cell2mat(robot_Trajectory_Vec);
    
    
end %end of the bigger loop

% figure(1);
% plot(robots_trajectory(:,1),robots_trajectory(:,2),'k-'),hold on, grid on,
% plot(robots_trajectory(:,3),robots_trajectory(:,4),'r-'),hold on, grid on,
% plot(robots_trajectory(:,5),robots_trajectory(:,6),'b-'),hold on, grid on,
% plot(robots_trajectory(:,7),robots_trajectory(:,8),'c-'),hold on, grid on,
% plot(robots_trajectory(:,9),robots_trajectory(:,10),'g-'),hold on, grid on,

% plot(robots_trajectory(:,11),robots_trajectory(:,12),'o-<'),hold on, grid on,

% plot(virtual_robot_Trajectory_Vec(:,1),virtual_robot_Trajectory_Vec(:,2),'o-'),hold on, grid on,




% %drawing a circle of radius=alpha
% count=1;
% for theta=0:0.1:2*pi
% x_circle(count)=(alpha+0.5)*cos(theta);
% y_circle(count)=(alpha+0.5)*sin(theta);
% count=count+1;
% end

% x_circle=x_circle+hyp_robot(1);
% y_circle=y_circle+hyp_robot(2);
% plot(hyp_robot(1),hyp_robot(2),'o'),hold on
% plot(x_circle,y_circle,'r-'),hold on,

