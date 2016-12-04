clc; clear all; close all;

number_of_robots=3; %Initialization of parameters
M=0.1; B_model=0.1; kd=2; ksk=1; kr=0.15; alpha=3; kp=10; tau=50; fxvdes=1.5; fyvdes=1.5; ra=ones(1:number_of_robots); tspan_ode45=[0.0 0.1];
%M=1; B_model=1; kd=9; ksk=100; kr=10; radius kp=10; tau=4; fxvdes=8; fyvdes=8; ra=ones(1:number_of_robots); tspan_ode45=[0.0 0.009];

q(1:number_of_robots)=4; %Charges on each robot
SWARM_FLAG=0; %Initially SWARM_FLAG=ZERO because the robots are not in a swarm formation
Chi=zeros(1,number_of_robots); Psi=zeros(1,number_of_robots); %Angles for obstacle avoidance

%Forces on robots when using behavioural structure with swarm formation
fxkVS_rk_ra=zeros(1,number_of_robots); fykVS_rk_ra=zeros(1,number_of_robots); fkVS_rk_ra=zeros(1,number_of_robots); 
FLAG=zeros(1,number_of_robots);

robot=cell(1,number_of_robots); %Initialization of robots
robot{1}=[-1,-5]; robot{2}=[-3.2,-5.2]; robot{3}=[0,-3]; robot{4}=[0.5,0.1]; robot{5}=[0.1,-0.1]; robot{6}=[0.10,0.6]; %Initial Position of the robots
hyp_robot=[-5,-5]; %position of the hypothetical (Virtual Leader) Robot

xdot = zeros(number_of_robots,1); ydot = zeros(number_of_robots,1); %new position of ith robot%
xdot_virtual=zeros(number_of_robots,1); ydot_virtual=zeros(number_of_robots,1); %new position of virtual robot%

r=zeros(number_of_robots,number_of_robots); %distance between two electric charges
theta=zeros(number_of_robots,number_of_robots); %angle between two electric charges
Electrostatic_Forces=zeros(number_of_robots,number_of_robots); %Force between two electrostatic charges
robot_Trajectory_Vec=[]; virtual_robot_Trajectory_Vec=[]; %Vector to hold the position history

color = 'kbgrcmy'; colorVal=1; %Code for Animated LInes
robotTrajectory_ZP = [animatedline('Color',color(colorVal),'LineWidth',2)];

for i = 1: number_of_robots-1
    colorVal = colorVal+1;
    if(colorVal>7)
        colorVal=1;
    end
    robotTrajectory_ZP = [robotTrajectory_ZP;animatedline('Color',color(colorVal),'LineWidth',2)];
end

Virtual_robotTrajectory_ZP = [animatedline('Color',color(7),'LineWidth',2)];
%Developing the Obstacle %Drawing ellipses arouond the obstacle %xa, ya is the origin of the rectangle (bottom left corner 
%as according to matlab rectangle command) %(x0,y0) is the centre of the obstacle %the ellipse envelops a rectangle with 
%(x0+-v1,y0+-v2) as its vertices
origin_of_rectangle=[-.75,-.75]; width=1.5; height=1.5; 

figure(1); %To view Simulation
rectangle('Position',[origin_of_rectangle width height],'Curvature',[0.05 0.05]),...
axis square,hold on
axis ([-5 5 -5 5]) %The simulation window comprises of -5,-5 to 5,5
xa=origin_of_rectangle(1); ya=origin_of_rectangle(2);

v1=width/2; v2=height/2;%sides
x0=xa+width/2; y0=ya+height/2; %Center of the obstacle

x=-20:0.1:40; %Plotting the ellipse encircling the obstacle %defining range for x
A=sqrt(1/(2*(v1^2))); B=sqrt(1/(2*(v2^2))); %AB and (AB)^2 should be maximum%

y1=real(sqrt((1-A^2*(x-x0).^2)/B^2)+y0); y2=real(-sqrt((1-A^2*(x-x0).^2)/B^2)+y0);
plot(x,y1,'g',x,y2,'g'),axis square,grid on,hold on

v1_expansion=v1:v1/10:v1*1.5; %Defining ellipses around the obstacles
A1=sqrt(1./(2*(v1_expansion.^2))); B1=(B/A)*A1;

%concentric elliptical lines
for i=1:length(B1)
    y1=real(sqrt((1-A1(i)^2*(x-x0).^2)/B1(i)^2)+y0); y2=real(-sqrt((1-A1(i)^2*(x-x0).^2)/B1(i)^2)+y0); 
    plot(x,y1,'--b',x,y2,'--b'),axis square,grid on,hold on    
end

for t=1:700  %total simulation time [The main BIG Loop]
    robot_Trajectory_Vec=[robot_Trajectory_Vec;robot];
    t
    for i=1:number_of_robots
        dummy=robot(i); xy_pos=cell2mat(dummy); x_pos(i)=xy_pos(1); y_pos(i)=xy_pos(2);
    end
     %Calculation of the angle with which robot is approaching the obstacle. We are using condition t>3 because the robot 
     %must have moved a little distance in order to calculate the heading angle. Heading angle is based on its current and 
     %previous position.
    if(t>3)        
        for i=1:number_of_robots
            final_pos=cell2mat(robot_Trajectory_Vec(t-1,i));
            initial_pos=cell2mat(robot_Trajectory_Vec(t-2,i));
            xy_coordinate=final_pos-initial_pos;
            Chi(i)=atan2(y0-y_pos(i),x0-x_pos(i)); %Angle between the obstacle centre-robot and the horizontal
            Psi(i)=atan2(xy_coordinate(2),xy_coordinate(1)); %Robot Heading angle with the horizontal
        end
    end
%Testing whether the robot is approaching the obstacle %If the 'test' value is approx 1e-3 it means that the robot is very 
%near the ellipse. If the robot is exactly on the ellipse then 'test' %must be ZERO. [This was the first approach I was using 
%for obstacle avoidance] The second approach for testing whetner the robot is approaching the obstacle or not is based on the 
%journal paper, it compares ra with rk and then takes a decision.

    for i=1:number_of_robots 
        rk(i)=sqrt(A^2*(x_pos(i)-x0)^2+B^2*(y_pos(i)-y0)^2-1); %Comes after Eq 20 on paper
        if (rk(i)<=ra(i)) %Defining forces to avoid obstacle
            fxkrc(i)=(B/A)*(y_pos(i)-y0); fykrc(i)=-(A/B)*(x_pos(i)-x0); fxkrcc(i)=-(B/A)*(y_pos(i)-y0); fykrcc(i)=(A/B)*(x_pos(i)-x0);
				if (mod(Psi(i)-Chi(i),2*pi)<=pi) %Psi>=Chi
					fxkr(i)=fxkrc(i); fykr(i)=fykrc(i);
				elseif (mod(Psi(i)-Chi(i),2*pi)>pi) %Psi<Chi
					fxkr(i)=fxkrcc(i); fykr(i)=fykrcc(i);
				end
            mod_fkr(i)=sqrt(fxkr(i)^2+fykr(i)^2); fxkrn(i)=fxkr(i)/mod_fkr(i); fykrn(i)=fykr(i)/mod_fkr(i); %Eq. 20
        end
    end
	
    rm=min(rk); %Computing the new position of Virtual Robot. Code for updating the Virtual robot position when I am using Obstacles if (sum(OBSTACLE)==0)
		if (rm>ra)
			fxvBS_virtual=fxvdes; fyvBS_virtual=fyvdes; %After Eq. 23   
		elseif (rm<=ra)
			xm=mean(x_pos); ym=mean(y_pos);
			fxvBS_virtual=fxvdes+kp*(xm-hyp_robot(1))*(1-exp(-tau*rm)); fyvBS_virtual=fyvdes+kp*(ym-hyp_robot(2))*(1-exp(-tau*rm));
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
    
    hyp_robot=[x_pos_new_virtual y_pos_new_virtual]; virtual_robot_Trajectory_Vec=[virtual_robot_Trajectory_Vec;hyp_robot];
   
    addpoints(Virtual_robotTrajectory_ZP,x_pos_new_virtual,y_pos_new_virtual);
    drawnow
    %End of code Computing the new position of Virtual Robot. Computing the force for the first robot Equation (6) of the paper 
    %Computing distance of individual robot from all other robots
		for i=1:number_of_robots 
            for j=1:number_of_robots
				a=robot{i}-robot{j}; r(i,j)=sqrt(a(1)^2+a(2)^2);
					if i==j
						theta(i,j)=0; Electrostatic_Forces(i,j)=0;
					elseif i~=j
						a=robot{i}-robot{j}; theta(i,j)=atan2(a(2),a(1)); Electrostatic_Forces(i,j)=(q(i)*q(j))/(r(i,j)^2);
					end
				EF_x(i,j)=Electrostatic_Forces(i,j)*cos(theta(i,j)); EF_y(i,j)=Electrostatic_Forces(i,j)*sin(theta(i,j));
			end
		end
  
		for i=1:number_of_robots
			a=robot{i}-hyp_robot;
			Attractive_Force_x(i)=ksk*(a(1)*(a(1)^2+a(2)^2-alpha^2)); Attractive_Force_y(i)=ksk*(a(2)*(a(1)^2+a(2)^2-alpha^2));
			fxkVS(i)=kr*sum(EF_x(i,1:number_of_robots))-Attractive_Force_x(i); fykVS(i)=kr*sum(EF_y(i,1:number_of_robots))-Attractive_Force_y(i);
			fkVS(i)=sqrt(fxkVS(i)^2+fykVS(i)^2);
        
			dummy=robot(i); xy_pos=cell2mat(dummy);
			x_pos(i)=xy_pos(1); y_pos(i)=xy_pos(2);
			x_pos_new=x_pos(i); y_pos_new=y_pos(i); %initializing x_pos_new and y_pos_new 
			distance_from_hyp_robot=sqrt((x_pos(i)-hyp_robot(1))^2+(y_pos(i)-hyp_robot(2))^2); %Forces with obstacles (Behavioral structure). Force when the robot is approaching the obstacle
        
			if ((rk(i)-ra(i))<1e-3) %disp('force when rk=ra');
				if (FLAG(i)==0)
					fxkVS_rk_ra(i)=fxkVS(i); fykVS_rk_ra(i)=fykVS(i); fkVS_rk_ra(i)=sqrt(fxkVS_rk_ra(i)^2+fykVS_rk_ra(i)^2);
					FLAG(i)=FLAG(i)+1;
				end
			end
        
			if(rk(i)<ra(i))
				FxkBS(i)= fxkVS_rk_ra(i)+((fkVS_rk_ra(i)*fxkrn(i))/(rk(i)^2))*(1/rk(i)-1/ra(i));
				FykBS(i)= fykVS_rk_ra(i)+((fkVS_rk_ra(i)*fykrn(i))/(rk(i)^2))*(1/rk(i)-1/ra(i));
			elseif (rk(i)>ra(i)) %Force when the robot has passed the obstacle %Eq. 22
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
 
			robot{i}=[x_pos_new y_pos_new]; addpoints(robotTrajectory_ZP(i),x_pos_new,y_pos_new);
			drawnow
		end
    robots_trajectory=cell2mat(robot_Trajectory_Vec); k=1;
    for i=1:number_of_robots   
        X_for_polygon(i)= robots_trajectory(end,k); Y_for_polygon(i)=robots_trajectory(end,k+1); k=k+2;
    end
    k=convhull(X_for_polygon,Y_for_polygon); figure(1);
    if t>200
       % pause
    end
    robots_trajectory=cell2mat(robot_Trajectory_Vec);
end %end of the bigger loop