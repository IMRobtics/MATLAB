cX = 0;
cY = 0;
cH = pi/2;

gX = 30;
gY = 30;

gTheta = atan2(gY-cY,gX-cX);
eTheta = gTheta-cH;
changeAngle = atan2(sin(eTheta),cos(eTheta));