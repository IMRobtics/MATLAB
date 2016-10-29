turns=5; %The number of turns the spiral will have
x=[-1*pi*turns : 1.5: pi*turns];
r=[0:1/(length(x)-1):1];

X=sin(x).*r*2;  Y=cos(x).*r*2;
plot(X,Y,'-r','LineWidth',2)
axis square