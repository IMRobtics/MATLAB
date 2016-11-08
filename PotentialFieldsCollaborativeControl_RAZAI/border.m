function h = border(x,y)
k = boundary(x,y,0);
h = plot(x(k),y(k),'-.');
