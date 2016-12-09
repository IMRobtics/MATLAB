function [forceX,forceY] = forceConstrain(force_x,force_y,max_F)
ratio = 1;
if(abs(force_x)>max_F || abs(force_y)>max_F)
    if(abs(force_x)>abs(force_y))
        ratio = abs(max_F/force_x);
    else
        ratio = abs(max_F/force_y);
    end
end
forceX = force_x*ratio;
forceY = force_y*ratio;
end