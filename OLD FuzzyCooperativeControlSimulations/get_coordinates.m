function [Abs, Ord] = get_coordinates(sl, cnt, x, y)

dis_r = sl * cnt;

dis = 0;
i = 2;
while dis < dis_r
%for i = 2:length(x)
    dx = x(i) - x(i-1);
    dy = y(i) - y(i - 1);
    
    dis = dis + sqrt(dx^2 + dy^2);
    i = i + 1;
end
Abs = x(i - 1);
Ord = y(i - 1);


end