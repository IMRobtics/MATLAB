function y = PhiAdjust(u)
phi = u;
%phi = abs( rem(phi,2*pi) );
phi = rem(phi,2*pi);
if phi >= 0
    if phi < pi
        y = phi;
    else
        y = phi - 2*pi;
    end
else
    if phi > -pi
        y = phi;
    else
        y = phi + 2*pi;
    end
end

end
