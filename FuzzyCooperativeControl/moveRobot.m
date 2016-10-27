function [x,y,xdot,ydot] = moveRobot(xc,yc,xdotp,ydotp)
	%xc and yc are current positions
	%xdotp and ydotp are previous rate of change
	robot    =[xc yc]
	robotdot =[xdotp ydotp] 

	% equation of force, see potential field paper
	% change equation inside [ ]
	fx = @(t,x) [x(2); (FxkVS(i)-(B+kd)*x(2))/M];
	[T,X]=ode45(fx,[0,0.05],[robot(1,1);robotdot(1,1)]);
	[m,z] = size(X);
	x=X(m,1);
	xdot=X(m,2);

	% equation of force, see potential field paper
	% change equation inside [ ]
	fy = @(t,y) [y(2); (FykVS(i)-(B+kd)*y(2))/M];
	[T,Y]=ode45(fy,[0,0.05],[robot(1,2);robotdot(1,2)]);
	[m,z] = size(Y);
	y=Y(m,1);
	ydot=Y(m,2);
end