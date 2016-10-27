function output_args = TrajGen_MMR_FUZZYTEST(input_args)

persistent xd1;
persistent yd1;
persistent xd2;
persistent yd2;
persistent xd3;
persistent yd3;

persistent x1;
persistent x2;
persistent x3;
persistent y1;
persistent y2;
persistent y3;
persistent p1;
persistent p2;
persistent p3;
persistent L1;
persistent L2;
persistent L3;
persistent Sl1;
persistent Sl2;
persistent Sl3;
persistent seg_count;
persistent tx1;%[];
persistent ty1;%[];
persistent tx2;%[];
persistent ty2;%[];
persistent tx3;%[];
persistent ty3;%[];

t = input_args(1);
tstop = input_args(2);
Exp = input_args(3);
Nos = 4000;
order_of_polynomial = 5;
step = (tstop - 10)/Nos;

if(tstop ~= 0)
    
    if (seg_count == 0)
        
        if(Exp == 1)  
            % Scenario 1:
            x1 = 0:0.01:17;
            x2 = 0:0.01:20;
            x3 = 0:0.01:25;
            y1 = 10 + 1*cos(1*x1) - 0.5*cos(3*x1);    % initial condition: 9.5
            y2 = 5 + 4*cos(2*x2) + 0.5*sin(5*x2);    % initial condition: 9.5
            y3 = 1 + 3*cos(2*x3) + 1.5*sin(5*x3);    % initial condition: 9.5
        elseif(Exp == 2)
            %Scenario 2:
            x1 = 0:0.01:13;
            x2 = 0:0.01:13;
            x3 = 0:0.01:13;
            y1 = 10 + 4*cos(2*x1) + 0.5*sin(5*x1);    %[x1,y1,phi1] = [1,9,0]
            y2 = 5 + 4*cos(2*x2) + 0.5*sin(5*x2);    %[x2,y2,phi2] = [-1.5,4,0]   
            y3 = 1 + 4*cos(2*x3) + 0.5*sin(5*x3);    %[x3,y3,phi3] = [2,-1.2,0]   
        end

        p1 = polyfit(x1,y1,order_of_polynomial);
        p2 = polyfit(x2,y2,order_of_polynomial);
        p3 = polyfit(x3,y3,order_of_polynomial);

        y1 = polyval(p1,x1);
        y2 = polyval(p2,x2);
        y3 = polyval(p3,x3);

        L1 = calc_len(x1,y1)
        L2 = calc_len(x2,y2)
        L3 = calc_len(x3,y3)
        Sl1 = L1 / Nos;
        Sl2 = L2/ Nos;
        Sl3 = L3/ Nos;
        
        for i = 1:Nos
            [a,b] = get_coordinates(Sl1,i,x1,y1);
            tx1 = [tx1,a];
            ty1 = [ty1,b];
            
            [a,b] = get_coordinates(Sl2,i,x2,y2);
            tx2 = [tx2,a];
            ty2 = [ty2,b];
            
            [a,b] = get_coordinates(Sl3,i,x3,y3);
            tx3 = [tx3,a];
            ty3 = [ty3,b];
        end
    end

    if(seg_count < Nos)
        
        if mod(t,step) == 0
            r = round(t/step + 1);
            xd1 = tx1(round(t/step + 1));  %x(t/step + 1);
            yd1 = ty1(round(t/step + 1));
            xd2 = tx2(round(t/step + 1));
            yd2 = ty2(round(t/step + 1));
            xd3 = tx3(round(t/step + 1));
            yd3 = ty3(round(t/step + 1));
            seg_count = seg_count + 1;
        end
        
    end
else

    xd1 = 0;
    yd1 = 0;
    xd2 = 0;
    yd2 = 0;
    xd3 = 0;
    yd3 = 0;
    seg_count = 0;
    
end


output_args(1) = xd1;
output_args(2) = yd1;

output_args(3) = xd2;
output_args(4) = yd2;

output_args(5) = xd3;
output_args(6) = yd3;
end