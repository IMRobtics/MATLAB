turns=2; %The number of turns the spiral will have
x=[-1*pi*turns : 2 : pi*turns];
r=[0:1/(length(x)-1):1];

X=sin(x).*r*100;  Y=cos(x).*r*100;
plot(X,Y,'-r','LineWidth',2)
axis square