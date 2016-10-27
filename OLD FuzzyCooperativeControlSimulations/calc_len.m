function Len = calc_len(x,y)

dis = 0;
for i = 2:length(x)
    dx = x(i) - x(i-1);
    dy = y(i) - y(i - 1);
    
    dis = dis + sqrt(dx^2 + dy^2);
end
Len = dis;
end
