clc;
clear all;
close all;

%attraction and repulsion between charges
%A negative charge is places at the centre and four chrages are placed at
%different location
%-----------------------------------------------------------------------
%Initialization of parameters
%-----------------------------------------------------------------------

number_of_robots=4;
robot=cell(1,number_of_robots);

%Initial Position of the robots
robot{1}=[5,-5];
robot{2}=[8,-10];
robot{3}=[-4,10];
robot{4}=[0,-5];
% robot{5}=[10,-6];

hyp_robot=[5,5];  %position of the hypothetical (Virtual Leader) Robot

%Vector to hold the position history
robot_Vec=[];

%Charges on each robot
q(1:number_of_robots)=1;

%Mass of each robot
m(1:number_of_robots)=1;

%Electrostatic Constant
K=10;

for t=1:250  %total simulation time
    
    %Compute the attractive forces between the center (hypothetical robot)
    %and the actual robots
    %-----------------------------------------------------------
    %-----------------------------------------------------------
    %Electrostatic force by Coloumb's Law
    %F=K(q1q2)/r^2;
    %-----------------------------------------------------------
    %-----------------------------------------------------------
    
    
    %Computing the force for the first robot
    %Equation (6) of the paper
    
    %Computing distance of individual robot from all other robots
    r=zeros(number_of_robots,number_of_robots);
    for i=1:number_of_robots
        for j=1:number_of_robots
            a=robot{i}-robot{j};
            r(i,j)=sqrt(a(1)^2+a(2)^2);
        end
    end
    
%     r
    
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
    
%     theta*180/pi
    
    
    %Computing Electrostatic forces (Repulsive) on individual robot
    %from all other robots
    
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
%     Electrostatic_Forces
    
    
    
    %Decomposition of Electrostatic forces in x and y components
    for i=1:number_of_robots
        for j=1:number_of_robots
            EF_x(i,j)=Electrostatic_Forces(i,j)*cos(theta(i,j));
            EF_y(i,j)=Electrostatic_Forces(i,j)*sin(theta(i,j));
        end
    end
    
    %Computing x and y components of Attractive Force (Equation 8 in JP)
    alpha=3; %radius of the circle
    ksk=.1;   %variable from JP
    
    for i=1:number_of_robots
        a=robot{i}-hyp_robot;
        Attractive_Force_x(i)=ksk*(a(1)*(a(1)^2+a(2)^2-alpha^2));
        Attractive_Force_y(i)=ksk*(a(2)*(a(1)^2+a(2)^2-alpha^2));
    end
    
    
    
    %Computing resultant forces on each robot
    for i=1:number_of_robots
        F_x_res(i)=sum(EF_x(i,1:number_of_robots))-Attractive_Force_x(i);
        F_y_res(i)=sum(EF_y(i,1:number_of_robots))-Attractive_Force_y(i);
    end
    
    for i=1:number_of_robots
        F_res(i)=sqrt(F_x_res(i)^2+F_y_res(i)^2);
        theta_res(i)=atan2(F_y_res(i),F_x_res(i));
    end
    F_res
    
    for i=1:number_of_robots
        i
        dummy=robot(i);
        xy_pos=cell2mat(dummy);
        x_pos=xy_pos(1);
        y_pos=xy_pos(2);
        
        distance_from_hyp_robot=sqrt((x_pos-hyp_robot(1))^2+(y_pos-hyp_robot(2))^2)
        
        if (F_res(i)<=1e-1)
            %don't move
            x_pos_new=x_pos;
            y_pos_new=y_pos;
        elseif (abs(distance_from_hyp_robot-alpha)<=5e-1)
            %don't move
            x_pos_new=x_pos;
            y_pos_new=y_pos;
        else
            %update robot position
            
            x_pos_new=x_pos+0.5*(F_res(i)*cos(theta_res(i)))*((t/10)^2);   %s=ut+0.5(F/m)t^2  we assume u=0 and t is the number of times loop is executed
            y_pos_new=y_pos+0.5*(F_res(i)*sin(theta_res(i)))*((t/10)^2);
        end
        robot{i}=[x_pos_new y_pos_new];
        
        robot_Vec=[robot_Vec;robot];
    end
    
    
end %end of the bigger loop


%Plotting Initial position
%     for i=1:number_of_robots
%         dummy=robot(i);
%         xy_pos=cell2mat(dummy);
%         x_pos=xy_pos(1);
%         y_pos=xy_pos(2);
%         figure(1),plot(x_pos,y_pos,'*'), grid on,hold on, axis tight;
%     end
%
robots_trajectory=cell2mat(robot_Vec);

figure(1);
plot(robots_trajectory(:,1),robots_trajectory(:,2),'k-<'),hold on, grid on,
plot(robots_trajectory(:,3),robots_trajectory(:,4),'r-<'),hold on, grid on,
plot(robots_trajectory(:,5),robots_trajectory(:,6),'b-<'),hold on, grid on,
plot(robots_trajectory(:,7),robots_trajectory(:,8),'c-<'),hold on, grid on,
% plot(robots_trajectory(:,9),robots_trajectory(:,10),'y-<'),hold on, grid on,

%drawing a circle of radius=alpha
count=1;
for theta=0:0.1:2*pi
x_circle(count)=(alpha+0.5)*cos(theta);
y_circle(count)=(alpha+0.5)*sin(theta);
count=count+1;
end

x_circle=x_circle+hyp_robot(1);
y_circle=y_circle+hyp_robot(2);

plot(x_circle,y_circle,'r-'),hold on,



% % % % %plotting a line between the final points
% % % % for i=1:2:(number_of_robots*2)
% % % %     
% % % %     x1=robots_trajectory(end,i);
% % % %     y1=robots_trajectory(end,i+1);
% % % %     
% % % %     if i<(number_of_robots*2-2)
% % % %         x2=robots_trajectory(end,i+2);
% % % %         y2=robots_trajectory(end,i+3);
% % % %     else
% % % %         x2=robots_trajectory(end,mod(i+2,number_of_robots*2));
% % % %         y2=robots_trajectory(end,mod(i+3,number_of_robots*2));
% % % %     end
% % % %     
% % % %     if (x1<=x2)
% % % %         
% % % %         x_points=x1:0.1:x2;
% % % %     elseif (x1>x2)
% % % %         x_points=x2:0.1:x1;
% % % %         %     elseif x1==x2
% % % %         %         x_points=x1*ones(1,100);
% % % %     end
% % % %     
% % % %     if (abs(x1-x2)<=1e-3)
% % % %         disp('I am here')
% % % %         if y1<y2
% % % %             y_points=y1:0.1:y2;
% % % %         elseif y1>y2
% % % %             y_points=y2:0.1:y1;
% % % %         end
% % % %         
% % % %         x_points=x1*ones(1,length(y_points));
% % % %     else
% % % %         
% % % %         slope=(y2-y1)/(x2-x1);
% % % %         y_points=slope*(x_points-x1)+y1;
% % % %     end
% % % %      
% % % %     
% % % %     figure(1)
% % % %     plot(x_points,y_points,'b--'),hold on,
% % % % end
