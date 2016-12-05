function h = ellipse(x0,y0,a,b)
t=-pi:0.01:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
h = plot(x,y);
% h = rectangle('Position',[x0 y0 a b], 'Curvature',[1 1]);