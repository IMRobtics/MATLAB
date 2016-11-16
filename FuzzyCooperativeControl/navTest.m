cX = 10;
cY = 10;
cH = pi/2;
gX = 9;
gY = -100;
gTheta = atan2(gY-cY,gX-cX);
eTheta = gTheta-cH;
changeAngle = atan2(sin(eTheta),cos(eTheta));

robotTheta=pi/2;
magTheta=299*pi/180;
corrected = magTheta+(60*pi/180);
Angle = atan2(sin(corrected),cos(corrected));
Angle*180/pi
